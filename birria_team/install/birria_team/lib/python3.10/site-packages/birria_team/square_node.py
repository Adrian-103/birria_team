import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class SquareOpenLoop(Node):
    def __init__(self):
        super().__init__('square_open_loop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.v = 0.1
        self.w = 0.15

        self.t_avance = 15.0
        self.t_giro = 10.5

        self.rate = self.create_rate(10)

        self.state = "avanzar"
        self.start_time = time.time()
        self.lado = 0

        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

    def loop(self):
        vel = Twist()
        elapsed = time.time() - self.start_time

        if self.lado >= 4:
            self.pub.publish(Twist())
            self.get_logger().info("Cuadrado terminado")
            self.timer.cancel()
            return

        if self.state == "avanzar":
            vel.linear.x = self.v

            if elapsed >= self.t_avance:
                self.state = "girar"
                self.start_time = time.time()

        elif self.state == "girar":
            vel.angular.z = self.w

            if elapsed >= self.t_giro:
                self.state = "avanzar"
                self.start_time = time.time()
                self.lado += 1

        self.pub.publish(vel)


def main():
    rclpy.init()
    node = SquareOpenLoop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()