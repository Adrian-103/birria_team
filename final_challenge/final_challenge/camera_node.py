import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge


class camera_node(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('publish_width', 320)
        self.declare_parameter('publish_height', 240)

        self.publish_width  = self.get_parameter('publish_width').value
        self.publish_height = self.get_parameter('publish_height').value

        self.frame_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        self.cap = None
        for i in range(5):
            candidate = cv.VideoCapture(i)
            if candidate.isOpened():
                self.cap = candidate
                self.get_logger().info(f'Cámara encontrada en índice {i}')
                break
        if self.cap is None:
            self.get_logger().error('No se encontró ninguna cámara')
            raise RuntimeError('No camera found')

        self.bridge = CvBridge()
        self.create_timer(1 / 24, self.timer_callback)
        self.get_logger().info(
            f'Publicando a {self.publish_width}x{self.publish_height}')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('No se pudo capturar un frame')
            return

        if (frame.shape[1] != self.publish_width or
                frame.shape[0] != self.publish_height):
            frame = cv.resize(
                frame, (self.publish_width, self.publish_height),
                interpolation=cv.INTER_LINEAR)

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.frame_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = camera_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
