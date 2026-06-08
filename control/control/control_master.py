#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from std_msgs.msg import String
from std_msgs.msg import Bool

from geometry_msgs.msg import Twist

from rcl_interfaces.msg import SetParametersResult


class MasterControl(Node):

    def __init__(self):

        super().__init__('master_control')

        # Parámetros
        self.declare_parameter('segundos_stop_intersection', 1.0)
        self.declare_parameter('segundos_lineal_straight', 5.0)
        self.declare_parameter('segundos_angular', 3.0)
        self.declare_parameter('segundos_lineal_turn', 3.0)
        self.declare_parameter('vel_lineal_intersection', 0.1)
        self.declare_parameter('vel_angular_turn', 0.5)

        self.segundos_stop_intersection = self.get_parameter('segundos_stop_intersection').value
        self.segundos_lineal_straight = self.get_parameter('segundos_lineal_straight').value
        self.segundos_angular = self.get_parameter('segundos_angular').value
        self.segundos_lineal_turn = self.get_parameter('segundos_lineal_turn').value
        self.vel_lineal_intersection = self.get_parameter('vel_lineal_intersection').value
        self.vel_angular_turn = self.get_parameter('vel_angular_turn').value

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Entradas
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.current_sign = "sin_señal"
        self.last_valid_sign = None   # última señal accionable recibida

        # Maniobras
        self.executing_maneuver = False
        self.maneuver_type = None
        self.maneuver_start_time = 0.0

        # Suscripciones
        self.create_subscription(Float32, '/lin_vel', self.lin_vel_callback, 10)
        self.create_subscription(Float32, '/ang_vel', self.ang_vel_callback, 10)
        self.create_subscription(String, '/detected_sign', self.sign_callback, 10)
        self.create_subscription(Bool, '/interseccion_detectada', self.inter_callback, 10)

        # Publicador
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Master Control iniciado")

    # Parámetros dinámicos

    def parameter_callback(self, params):

        for param in params:

            if param.name == 'segundos_stop_intersection':
                self.segundos_stop_intersection = param.value

            elif param.name == 'segundos_lineal_straight':
                self.segundos_lineal_straight = param.value

            elif param.name == 'segundos_angular':
                self.segundos_angular = param.value

            elif param.name == 'segundos_lineal_turn':
                self.segundos_lineal_turn = param.value

            elif param.name == 'vel_lineal_intersection':
                self.vel_lineal_intersection = param.value

            elif param.name == 'vel_angular_turn':
                self.vel_angular_turn = param.value

            self.get_logger().info(f'{param.name} -> {param.value}')

        return SetParametersResult(successful=True)

    # Callbacks

    def lin_vel_callback(self, msg):
        self.lin_vel = msg.data

    def ang_vel_callback(self, msg):
        self.ang_vel = msg.data

    def sign_callback(self, msg):
        self.current_sign = msg.data
        # Guardar la última señal accionable para usarla en la intersección
        # aunque el tópico ya haya regresado a "sin_señal"
        if msg.data in ["straight", "turnL", "turnR"]:
            self.last_valid_sign = msg.data

    def inter_callback(self, msg):

        if not msg.data:
            return

        if self.executing_maneuver:
            return

        if self.last_valid_sign is None:
            return

        self.executing_maneuver = True
        self.maneuver_start_time = time.time()
        self.maneuver_type = self.last_valid_sign

        self.get_logger().info(f"Interseccion -> {self.maneuver_type}")

    # Loop

    def control_loop(self):

        cmd = Twist()

        # Maniobra en intersección

        if self.executing_maneuver:

            elapsed = time.time() - self.maneuver_start_time

            # Straight

            if self.maneuver_type == "straight":

                t0 = self.segundos_stop_intersection

                if elapsed < t0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                elif elapsed < (t0 + self.segundos_lineal_straight):
                    cmd.linear.x = self.vel_lineal_intersection
                    cmd.angular.z = 0.0

                else:
                    self.executing_maneuver = False
                    self.last_valid_sign = None
                    self.get_logger().info("Fin maniobra")
                    return

            # Turn Left

            elif self.maneuver_type == "turnL":

                t0 = self.segundos_stop_intersection
                t1 = t0 + self.segundos_lineal_turn
                t2 = t1 + self.segundos_angular
                t3 = t2 + self.segundos_lineal_turn

                if elapsed < t0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                elif elapsed < t1:
                    cmd.linear.x = self.vel_lineal_intersection
                    cmd.angular.z = 0.0

                elif elapsed < t2:
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.vel_angular_turn

                elif elapsed < t3:
                    cmd.linear.x = self.vel_lineal_intersection
                    cmd.angular.z = 0.0

                else:
                    self.executing_maneuver = False
                    self.last_valid_sign = None
                    self.get_logger().info("Fin maniobra")
                    return

            # Turn Right

            elif self.maneuver_type == "turnR":

                t0 = self.segundos_stop_intersection
                t1 = t0 + self.segundos_lineal_turn
                t2 = t1 + self.segundos_angular
                t3 = t2 + self.segundos_lineal_turn

                if elapsed < t0:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0

                elif elapsed < t1:
                    cmd.linear.x = self.vel_lineal_intersection
                    cmd.angular.z = 0.0

                elif elapsed < t2:
                    cmd.linear.x = 0.0
                    cmd.angular.z = -self.vel_angular_turn

                elif elapsed < t3:
                    cmd.linear.x = self.vel_lineal_intersection
                    cmd.angular.z = 0.0

                else:
                    self.executing_maneuver = False
                    self.last_valid_sign = None
                    self.get_logger().info("Fin maniobra")
                    return

            self.cmd_pub.publish(cmd)
            return

        # Normal

        cmd.linear.x = self.lin_vel
        cmd.angular.z = self.ang_vel

        self.cmd_pub.publish(cmd)


# Main

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
