#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist


class MasterControl(Node):

    def __init__(self):

        super().__init__('master_control')

        # Entradas
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        self.current_sign = "sin_señal"

        # Intersección

        self.intersection_active = False
        self.intersection_end_time = 0.0

        # Parámetros

        self.straight_time = 1.0
        self.turn_time = 1.5

        # Suscriptores

        self.create_subscription(Float32,'/lin_vel',self.lin_vel_callback,10)

        self.create_subscription(Float32,'/ang_vel',self.ang_vel_callback,10)

        self.create_subscription(String,'/detected_sign',self.sign_callback,10)

        self.create_subscription(Bool,'/interseccion_detectada',self.inter_callback,10)

        # Publicador

        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)

        self.timer = self.create_timer(0.05,self.control_loop)

        self.get_logger().info("Master Control iniciado")

    # Callbacks

    def lin_vel_callback(self, msg):

        if not self.intersection_active:
            self.lin_vel = msg.data

    def ang_vel_callback(self, msg):

        if not self.intersection_active:
            self.ang_vel = msg.data

    def sign_callback(self, msg):

        self.current_sign = msg.data

    def inter_callback(self, msg):

        if not msg.data:
            return

        if self.intersection_active:
            return

        # ----------------------------------

        self.intersection_active = True

        if self.current_sign == "turnL":

            self.intersection_end_time = (time.time() + self.turn_time)

            self.get_logger().info("Interseccion -> Giro izquierda")

        elif self.current_sign == "turnR":

            self.intersection_end_time = (time.time() + self.turn_time)

            self.get_logger().info("Interseccion -> Giro derecha")

        elif self.current_sign == "straight":

            self.intersection_end_time = (time.time() + self.straight_time)

            self.get_logger().info("Interseccion -> Seguir derecho")

    # Loop

    def control_loop(self):

        cmd = Twist()

        # Intersección

        if self.intersection_active:

            if time.time() >= self.intersection_end_time:

                self.intersection_active = False

                self.get_logger().info("Fin maniobra")

                return

            # ------------------------------

            if self.current_sign == "turnL":

                cmd.linear.x = 0.03
                cmd.angular.z = 0.35

            elif self.current_sign == "turnR":

                cmd.linear.x = 0.03
                cmd.angular.z = -0.35

            elif self.current_sign == "straight":

                cmd.linear.x = 0.05
                cmd.angular.z = 0.0

            self.cmd_pub.publish(cmd)

            return

        # Normal

        cmd.linear.x = self.lin_vel
        cmd.angular.z = self.ang_vel

        self.cmd_pub.publish(cmd)


def main(args=None):

    rclpy.init(args=args)

    node = MasterControl()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        stop = Twist()

        node.cmd_pub.publish(stop)

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()