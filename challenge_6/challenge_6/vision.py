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

        self.debug = self.get_parameter('debug').value
        self.vel_red = self.get_parameter('vel_red').value
        self.vel_yellow = self.get_parameter('vel_yellow').value
        self.vel_green = self.get_parameter('vel_green').value
        self.vel_no_light = self.get_parameter('vel_no_light').value

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

        msg = Float32()

        if red_detected:
            msg.data = self.vel_red
            self.get_logger().info('RED - STOP')
        elif yellow_detected:
            msg.data = self.vel_yellow
            self.get_logger().info('YELLOW - SLOW')
        elif green_detected:
            msg.data = self.vel_green
            self.get_logger().info('GREEN - GO')
        else:
            msg.data = self.vel_no_light
            self.get_logger().info('NO LIGHT')

        self.publisher_.publish(msg)

        # --- Debug visual (comentado por default, útil para tuning de máscaras) ---
        if self.debug:
            red_mask    = clean_mask(col_mask(hsv_image, "red"))
            yellow_mask = clean_mask(col_mask(hsv_image, "yellow"))
            green_mask  = clean_mask(col_mask(hsv_image, "green"))
            cv.imshow('image', image)
            cv.imshow('red mask',    cv.bitwise_and(image, image, mask=red_mask))
            cv.imshow('yellow mask', cv.bitwise_and(image, image, mask=yellow_mask))
            cv.imshow('green mask',  cv.bitwise_and(image, image, mask=green_mask))
            cv.waitKey(1)

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
