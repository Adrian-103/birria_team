import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np

# --------------------------------------
# Claves semáforo - Tópico
# --------------------------------------
CLAVES = {
    "green": "verde",
    "yellow": "amarillo",
    "red": "rojo",
    "sin sem": "ss",
}

# --------------------------------------
# HSV Color Range Setup for Color Masks
# --------------------------------------
color_rang = {
    "red": [
        (np.array([0, 140, 200]), np.array([8, 255, 255])),
        (np.array([170, 130, 200]), np.array([179, 255, 255]))
    ],
    "yellow": [
        (np.array([15, 120, 190]), np.array([35, 255, 255]))
    ],
    "green": [
        (np.array([38, 90, 100]), np.array([88, 255, 255]))
    ]
}

def col_mask(hsv_image, color):
    mask = None
    for lower, upper in color_rang[color]:
        cur_mask = cv.inRange(hsv_image, lower, upper)
        mask = cur_mask if mask is None else cv.bitwise_or(mask, cur_mask)
    return mask

def clean_mask(mask):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=2)
    return mask
    
def blob_exist(mask, min_area=300, max_area=6000):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv.contourArea(contour)
        if min_area < area < max_area:
            return True
    return False


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__('traffic_light_node')

        # Parámetros configurables ---------------------------
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        #Control para bandera ---------------------------
        self.flag_counter = 0
        self.frames_flag = 8
        self.flag_active_count = 0      # Cuenta las banderas que se detectan
        self.pres_flag = False          # Evita que la bandera se sume varias veces
        self.robot_start = False        # Indica el inicio del robot

        #Para no repetir el color en topico ---------------------------
        self.last_c = None


        #self.debug = self.get_parameter('debug').value (Quitar)
        
        self.bridge = CvBridge()

        #Subscripcion a camara ---------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publicar clave del semaforo ---------------------------
        self.color_pub = self.create_publisher(String, '/semaforo', 10)

        self.get_logger().info('Nodo semaforo listo')


    def image_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error al convertir imagen ROS2 a OpenCV: {e}')
            return


        hsv_image = cv.cvtColor(image, cv.COLOR_BGR2HSV)

        red_mask    = clean_mask(col_mask(hsv_image, "red"))
        yellow_mask = clean_mask(col_mask(hsv_image, "yellow"))
        green_mask  = clean_mask(col_mask(hsv_image, "green"))

        red_detected    = blob_exist(red_mask)
        yellow_detected = blob_exist(yellow_mask)
        green_detected  = blob_exist(green_mask)

        raw_flag_detected = self.checked_flag(image)

        # Bandera Control ---------------------------
        if raw_flag_detected:
            self.flag_counter += 1
        else:
            self.flag_counter = max(0, self.flag_counter - 1)

        flag_confirm = self.flag_counter >= self.frames_flag

        # Máquina de estados de bandera ---------------------------
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

        if not self.robot_start:       
            if self.flag_active_count >= 2:
                self.get_logger().info(f'FINISHED - STOP', throttle_duration_sec = 2.0) 
                # throttle_duration_sec = 2.0 nos ayuda a limitar la frecuencia con la que se imprime un mensaje en terminal
            else:
                self.get_logger().info(f'WAITING FOR START FLAG', throttle_duration_sec = 2.0)
                self._pub_color("sin sem")

        else:
            if red_detected:
                self.get_logger().info('RED')
                self._pub_color("red")

            elif yellow_detected:
                self.get_logger().info('YELLOW')
                self._pub_color("yellow")

            elif green_detected:
                self.get_logger().info('GREEN')
                self._pub_color("green")

            else:
                self.get_logger().info('NO LIGHT', throttle_duration_sec = 2.0)
                self._pub_color("sin sem")


        # Debug visual ---------------------------
        if self.debug:
            # Debug para bandera de arranque o final 
            height, width, _ = image.shape
            # Dibujamos visualmente la zona exacta donde el código busca la bandera
            roi_y_inicio = int(height * 0.05)
            roi_y_fin = int(height * 0.45)
            roi_x_inicio = int(width * 0.25)
            roi_x_fin = int(width * 0.75)
            
            # Pintamos el rectángulo de búsqueda de la bandera directamente en la pantalla principal
            cv.rectangle(image, (roi_x_inicio, roi_y_inicio), (roi_x_fin, roi_y_fin), (255, 0, 0), 2)

            # Debug para semaforo
            cv.imshow('image', image)
            cv.imshow('red mask',    cv.bitwise_and(image, image, mask=red_mask))
            cv.imshow('yellow mask', cv.bitwise_and(image, image, mask=yellow_mask))
            cv.imshow('green mask',  cv.bitwise_and(image, image, mask=green_mask))
            cv.waitKey(1)

    def _pub_color(self, color_key):
        # Publica la clave ya que se detecto el color
        if color_key != self.last_c:
            clave = CLAVES.get(color_key, color_key)
            msg = String()
            msg.data = clave
            self.color_pub.publish(msg)
            self.last_c = color_key
            self.get_logger().info(f'Semaforo, topico: "{clave}"')


    def checked_flag(self, image):
        # Pasamos la imagen a escala de grises y aplicamos GausianBlur para eliminar ruido digital
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)

        # Con esto se crea como una "caja flotante" para que solo analice esa parte donde se detecte la bandera
        # en lugar de analizar toda la imagen con threshold que era lo primero que teniamos
        height, width = gray.shape
        # Cortamos la zona superior/media donde suelen aparecer las banderas
        roi_y_inicio = int(height * 0.05)
        roi_y_fin = int(height * 0.45)
        roi_x_inicio = int(width * 0.25)
        roi_x_fin = int(width * 0.75)

        gray_roi = blur[roi_y_inicio:roi_y_fin, roi_x_inicio:roi_x_fin]

        _, binary_roi = cv.threshold(
            gray_roi, 0, 255,
            cv.THRESH_BINARY + cv.THRESH_OTSU
        )

        white_ratio = np.sum(binary_roi == 255) / binary_roi.size
        black_ratio = np.sum(binary_roi == 0) / binary_roi.size

        if white_ratio < 0.30 or black_ratio < 0.30:
            return False

        h_change = np.sum(binary_roi[:, 1:] != binary_roi[:, :-1])
        v_change = np.sum(binary_roi[1:, :] != binary_roi[:-1, :])

        t_change = h_change + v_change
        norm_change = t_change / binary_roi.size

        if self.debug:
            self.get_logger().info(
                f'flag debug',
                throttle_duration_sec=1.0
            )
            cv.imshow('ROI Bandera Binarizada', binary_roi)

        if norm_change < 0.07:
            return False

        return True

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
