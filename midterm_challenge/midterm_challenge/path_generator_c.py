import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class PathGenerator(Node):
    def __init__(self):
        super().__init__('path_generator_c')

        # Publicador
        self.goal_pub = self.create_publisher(Point, '/goal', 10)

        # Posición actual
        self.pose_sub = self.create_subscription(
            Point,
            '/puzzlebot_pose',
            self.pose_callback,
            10
        )

        # Puntos
        self.goals = [
            (2.0, 0.0),
            (2.0, 2.0),
            (0.0, 2.0),
            (0.0, 0.0)
        ]

        self.current_goal_index = 0
        self.tolerance = 0.1

        # Estado actual
        self.current_x = 0.0
        self.current_y = 0.0

        # Detectar si no llega
        self.prev_distance = float('inf')
        self.stuck_counter = 0
        self.max_stuck = 20

        self.start_time = self.get_clock().now()
        self.max_time = 15.0  # segundos por punto

        self.publish_goal()

    def publish_goal(self):
        point = Point()
        x, y = self.goals[self.current_goal_index]

        point.x = x
        point.y = y
        point.z = 0.0

        self.goal_pub.publish(point)

        # Reset métricas
        self.prev_distance = float('inf')
        self.stuck_counter = 0
        self.start_time = self.get_clock().now()

        self.get_logger().info(f"Nuevo objetivo: ({x}, {y})")

    def pose_callback(self, msg):
        if self.current_goal_index >= len(self.goals):
            return

        self.current_x = msg.x
        self.current_y = msg.y

        goal_x, goal_y = self.goals[self.current_goal_index]

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y

        distance = math.sqrt(dx**2 + dy**2)

        # Llegó
        if distance < self.tolerance:
            self.get_logger().info("Llegó al objetivo")
            self.next_goal()
            return

        # Atascado
        if distance >= self.prev_distance:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        if self.stuck_counter > self.max_stuck:
            self.get_logger().warn("No llegó al objetivo")
            self.next_goal()
            return

        self.prev_distance = distance

        # Timeout
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if elapsed > self.max_time:
            self.get_logger().warn("No llegó al objetivo (timeout)")
            self.next_goal()
            return

    def next_goal(self):
        self.current_goal_index += 1

        if self.current_goal_index >= len(self.goals):
            self.get_logger().info("Terminó los 3 puntos")
            return

        self.publish_goal()

def main(args=None):
    rclpy.init(args=args)
    node = PathGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
