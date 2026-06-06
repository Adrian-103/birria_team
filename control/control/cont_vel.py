#!/usr/bin/env python3

import time
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Bool

# Funciones de membresía
def triangular(x, a, b, c):
    x = np.asarray(x)

    if b == a:
        left = np.where(x <= b, 1.0, 0.0)
    else:
        left = (x - a) / (b - a)

    if c == b:
        right = np.where(x >= b, 1.0, 0.0)
    else:
        right = (c - x) / (c - b)

    return np.maximum(0, np.minimum(left, right))


def trapezoidal(x, a, b, c, d):

    x = np.asarray(x)

    if b == a:
        left = np.where(x < a, 0.0, 1.0)
    else:
        left = np.maximum(0, np.minimum((x - a) / (b - a), 1))

    if d == c:
        right = np.where(x > d, 0.0, 1.0)
    else:
        right = np.maximum(0, np.minimum((d - x) / (d - c), 1))

    return np.minimum(left, right)

# Fuzzy and
def fuzzy_and(a, b):
    return a * b

# ROS2
class FuzzyVelocityNode(Node):

    def __init__(self):

        super().__init__('fuzzy_velocity_node')

        # Estados actuales
        self.semaforo = "ss"
        self.senal = "sin_señal"
        self.obstaculo = "libre"

        # Estados especiales
        self.finished = False

        self.stop_active = False
        self.stop_end_time = 0.0

        self.stop_seen = False

        self.give_way_active = False

        # Subscriptores
        self.create_subscription(String,'/semaforo',self.semaforo_callback,10)

        self.create_subscription(String,'/detected_sign',self.senal_callback,10)

        self.create_subscription(String,'/obstaculo',self.obstaculo_callback,10)

        self.create_subscription(Bool,'/interseccion_detectada',self.inter_callback,10)

        # Publicador cmd_vel
        self.vel_pub = self.create_publisher(Float32,'/lin_vel',10)

        # Timer
        self.timer = self.create_timer(0.1,self.control_loop)

        self.get_logger().info("Nodo de velocidad difusa iniciado")

    # Callbacks
    def semaforo_callback(self, msg):

        self.semaforo = msg.data

    def obstaculo_callback(self, msg):

        self.obstaculo = msg.data

    def inter_callback(self, msg):
        if msg.data:

            if self.give_way_active:

                self.give_way_active = False

                self.get_logger().info("Interseccion detectada -> fin Give Way")

    def senal_callback(self, msg):

        nueva = msg.data

        self.senal = nueva

        # Meta final
        if nueva == "yb":

            self.finished = True

            self.get_logger().info("Meta detectada")

        # Stop
        if nueva == "stop":

            if not self.stop_seen:

                self.stop_seen = True

                self.stop_active = True
                self.stop_end_time = time.time() + 3.0

                self.get_logger().info("STOP detectado")

        else:
            self.stop_seen = False

        # Give Way
        if nueva == "gW":

            if not self.give_way_active:

                self.give_way_active = True

                self.get_logger().info("Give Way activado")

    # Lógica difusa
    def calcular_velocidad_fuzzy(self):

        sem_map = {
            "rojo": 0,
            "amarillo": 50,
            "verde": 100,
            "ss": 100
        }

        sig_map = {
            "stop": 0,
            "caja": 0,
            "yb": 0,
            "gW": 50,
            "rW": 50,
            "straight": 100,
            "sin_señal": 100
        }

        sem = sem_map.get(self.semaforo, 100)
        sig = sig_map.get(self.senal, 100)

        # Membresías semáforo
        mu_rojo = trapezoidal(sem, 0, 0, 20, 50)
        mu_amarillo = triangular(sem, 25, 50, 75)
        mu_verde = trapezoidal(sem, 50, 80, 100, 100)

        # Membresías señal
        mu_stop = trapezoidal(sig, 0, 0, 20, 50)
        mu_prec = triangular(sig, 25, 50, 75)
        mu_normal = trapezoidal(sig, 50, 80, 100, 100)

        rules = []

        # ROJO
        rules.append((fuzzy_and(mu_rojo, mu_stop), 0.0))
        rules.append((fuzzy_and(mu_rojo, mu_prec), 0.0))
        rules.append((fuzzy_and(mu_rojo, mu_normal), 0.0))

        # AMARILLO
        rules.append((fuzzy_and(mu_amarillo, mu_stop), 0.0))
        rules.append((fuzzy_and(mu_amarillo, mu_prec), 0.05))
        rules.append((fuzzy_and(mu_amarillo, mu_normal), 0.05))

        # VERDE
        rules.append((fuzzy_and(mu_verde, mu_stop), 0.0))
        rules.append((fuzzy_and(mu_verde, mu_prec), 0.05))
        rules.append((fuzzy_and(mu_verde, mu_normal), 0.10))

        num = 0.0
        den = 0.0

        for w, z in rules:

            num += w * z
            den += w

        if den == 0:
            return 0.0

        return float(num / den)

    # Loop
    def control_loop(self):
        
        velocidad = Float32()

        # Prioridad 0
        if self.finished:

            velocidad.data = 0.0
            self.vel_pub.publish(velocidad)
            return

        # Prioridad 1
        if self.obstaculo == "caja":

            velocidad.data = 0.0
            self.vel_pub.publish(velocidad)
            return

        # Prioridad 2
        if self.stop_active:

            if time.time() < self.stop_end_time:

                velocidad.data = 0.0
                self.vel_pub.publish(velocidad)
                return

            else:

                self.stop_active = False

                self.get_logger().info("Fin STOP")

        # Prioridad 3
        if self.give_way_active:

            velocidad.data = 0.05
            self.vel_pub.publish(velocidad)
            return

        # Lógica difusa

        velocidad.data = self.calcular_velocidad_fuzzy()

        self.vel_pub.publish(velocidad)

# Main
def main(args=None):

    rclpy.init(args=args)

    node = FuzzyVelocityNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()