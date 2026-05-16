import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np

# --------------------------------------
# HSV Color Range Setup for Color Masks
# --------------------------------------
color_rang = {
    "red": [
        (np.array([0, 120, 120]), np.array([8, 255, 255])),
        (np.array([170, 110, 110]), np.array([179, 255, 255]))
    ],
    "yellow": [
        (np.array([15, 100, 120]), np.array([38, 255, 255]))
    ],
    "green": [
        (np.array([38, 70, 70]), np.array([88, 255, 255]))
    ]
}

def col_mask(hsv_image, color):
    mask = None
    for lower, upper in color_rang[color]:
        cur_mask = cv.inRange(hsv_image, lower, upper)
        mask = cur_mask if mask is None else cv.bitwise_or(mask, cur_mask)
    return mask

def clean_mask(mask):
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=2)
    return mask
    
def blob_exist(mask, min_area=500):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv.contourArea(contour) > min_area:
            return True
    return False


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        # Parámetros configurables
        self.declare_parameter('debug', False)
        self.declare_parameter('vel_red', 0.0)
        self.declare_parameter('vel_yellow', 0.1)
        self.declare_parameter('vel_green', 0.2)
        self.declare_parameter('vel_no_light', 0.2)  # sin semáforo = vía libre
        #Control para bandera
        self.flag_counter = 0
        self.frames_flag = 3

        self.debug = self.get_parameter('debug').value
        self.vel_red = self.get_parameter('vel_red').value
        self.vel_yellow = self.get_parameter('vel_yellow').value
        self.vel_green = self.get_parameter('vel_green').value
        self.vel_no_light = self.get_parameter('vel_no_light').value
        
        self.flag_active_count = 0      # Cuenta las banderas que se detectan
        self.pres_flag = False          # Evita que la bandera se sume varias veces
        self.robot_start = False        # Indica el inicio del robot

        self.bridge = CvBridge()

        # Recibir imagenes desde el nodo de la cámara
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publicar velocidad de acuerdo al color detectado
        self.publisher_ = self.create_publisher(Float32, 'max_vel', 10)


    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen ROS2 a OpenCV: {e}')
            return


        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        red_detected    = blob_exist(clean_mask(col_mask(hsv_image, "red")))
        yellow_detected = blob_exist(clean_mask(col_mask(hsv_image, "yellow")))
        green_detected  = blob_exist(clean_mask(col_mask(hsv_image, "green")))
        raw_flag_detected = self.checked_flag(image)

        if raw_flag_detected:
            self.flag_counter += 1
        else:
            self.flag_counter = max(0, self.flag_counter - 1)

        flag_confirm = self.flag_counter >= self.frames_flag

        # Máquina de estados de bandera
        if flag_confirm and not self.pres_flag:
            # Se detecto una bandera
            self.flag_active_count += 1
            # Se bloquea el contador de la bandera hasta que se deje de ver para que no se sumen mas banderas
            self.pres_flag = True
            self.get_logger().info(f'¡FLAG {self.flag_active_count} DETECTED!')

            # Cambia el estado de acuerdo a que bandera se muestre (inicio o fin)
            if self.flag_active_count == 1:
                self.robot_start = True
                self.get_logger().info('START!')
            
            elif self.flag_active_count == 2:
                self.robot_start = False
                self.get_logger().info(f'FINISH!')

        elif not flag_confirm and self.pres_flag:
            # Si el robot ya dejo de ver la bandera, se termina su bloqueo
            self.pres_flag = False


        msg_vel = Float32()

        if not self.robot_start:
            msg_vel.data = self.vel_red
            
            if self.flag_active_count >= 2:
                self.get_logger().info(f'FINISHED - STOP', throttle_duration_sec = 2.0) 
                # throttle_duration_sec = 2.0 nos ayuda a limitar la frecuencia con la que se imprime un mensaje en terminal
            else:
                self.get_logger().info(f'WAITING FOR START FLAG', throttle_duration_sec = 2.0)

        else:
            if red_detected:
                msg_vel.data = self.vel_red
                self.get_logger().info('RED - STOP')
            elif yellow_detected:
                msg_vel.data = self.vel_yellow
                self.get_logger().info('YELLOW - SLOW')
            elif green_detected:
                msg_vel.data = self.vel_green
                self.get_logger().info('GREEN - GO')
            else:
                msg_vel.data = self.vel_no_light
                self.get_logger().info('NO LIGHT')

        self.publisher_.publish(msg_vel)

        # --- Debug visual (comentado por default, útil para tuning de máscaras) ---
        if self.debug:
            # Debug para bandera de arranque o final 
            height, width, _ = image.shape
            # Dibujamos visualmente la zona exacta donde el código busca la bandera
            roi_y_inicio = int(height * 0.1)
            roi_y_fin = int(height * 0.6)
            roi_x_inicio = int(width * 0.1)
            roi_x_fin = int(width * 0.9)
            
            # Pintamos el rectángulo de búsqueda de la bandera directamente en la pantalla principal
            cv.rectangle(image, (roi_x_inicio, roi_y_inicio), (roi_x_fin, roi_y_fin), (255, 0, 0), 2)

            # Debug para semaforo
            red_mask    = clean_mask(col_mask(hsv_image, "red"))
            yellow_mask = clean_mask(col_mask(hsv_image, "yellow"))
            green_mask  = clean_mask(col_mask(hsv_image, "green"))
            cv.imshow('image', image)
            cv.imshow('red mask',    cv.bitwise_and(image, image, mask=red_mask))
            cv.imshow('yellow mask', cv.bitwise_and(image, image, mask=yellow_mask))
            cv.imshow('green mask',  cv.bitwise_and(image, image, mask=green_mask))
            cv.waitKey(1)

    def checked_flag(self, image):
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)

        height, width = gray.shape
        # Cortamos la zona superior/media donde suelen aparecer las banderas
        roi_y_inicio = int(height * 0.1)
        roi_y_fin = int(height * 0.6)
        roi_x_inicio = int(width * 0.1)
        roi_x_fin = int(width * 0.9)

        gray_roi = blur[roi_y_inicio:roi_y_fin, roi_x_inicio:roi_x_fin]

        _, binary_roi = cv.threshold(
            gray_roi, 0, 255,
            cv.THRESH_BINARY + cv.THRESH_OTSU
        )

        # Si el modo debug está activo, forzamos la apertura de la ROI binarizada
        if self.debug:
            cv.imshow('ROI Bandera Binarizada', binary_roi)

        # Contamos transiciones en la matriz de la ROI
        h_change = np.sum(binary_roi[:, 1:] != binary_roi[:, :-1])
        v_change = np.sum(binary_roi[1:, :] != binary_roi[:-1, :])

        t_change = h_change + v_change
        norm_change = t_change / (binary_roi.shape[0] * binary_roi.shape[1])

        # Bajamos el umbral a un valor sumamente sensible (0.03) para asegurarnos de que reaccione rápido
        if norm_change > 0.03:
            return True
            
        return False

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
