import math
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from rcl_interfaces.msg import SetParametersResult

def clamp(value, min_value, max_value):

    # Limita una variable entre un valor mínimo y máximo.
    return max(min(value, max_value), min_value)


# Clase PID
@dataclass
class PID:

    kp: float
    ki: float
    kd: float

    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self):

        # Reinicia términos acumulados del PID.
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):

        # dt = tiempo transcurrido desde la última iteración
        if dt <= 0.0:
            return 0.0

        # Término integral
        # Acumula error en el tiempo.
        self.integral += error * dt

        # Término derivativo
        # Mide qué tan rápido cambia el error.
        # Ayuda a amortiguar oscilaciones.
        derivative = (error - self.prev_error) / dt
        self.prev_error = error

        # Ecuación PID
        return (self.kp * error + self.ki * self.integral + self.kd * derivative)


# Controlador
class PointPIDController(Node):

    def __init__(self):

        super().__init__('point_PID_controller')

        # Parámetros

        # Frecuencia del controlador.
        # La cámara trabaja a 60 FPS,entonces hacemos el control también a 60 Hz.
        self.declare_parameter('sample_freq', 60.0)

        # PID angular

        # El error viene en pixeles.
        # Valores pequeños de kp son suficientes?

        # kp:Hace que el robot gire proporcionalmente al error de línea.
        self.declare_parameter('kp_yaw', 0.008)

        # ki:
        # Lo dejamos en cero porque:
        # - el error visual cambia constantemente
        # - el integral acumula ruido
        self.declare_parameter('ki_yaw', 0.0)

        # kd:
        # Ayuda a amortiguar oscilaciones.
        # Hay ruido,jitter,y pequeñas vibraciones en el error.
        self.declare_parameter('kd_yaw', 0.001)

        # Velocidades

        # Velocidad lineal máxima física del robot.
        self.declare_parameter('max_linear_vel', 0.2)

        # Velocidad angular máxima física del robot.
        self.declare_parameter('max_angular_vel', 0.2)

        # Pérdida de linea

        # Si en este tiempo no llega ningún line_error -> asumimos que el robot perdió la línea.

        self.declare_parameter('line_lost_timeout', 0.5)

        # Filtro exponencial

        # Suaviza el error visual para evitar vibraciones.
        # Fórmula:
        # ef = alpha*e + (1-alpha)*ef
        #
        # alpha grande:
        # -> más rápido
        # -> más ruido
        #
        # alpha pequeño:
        # -> más suave
        # -> más lento

        self.declare_parameter('filter_alpha', 0.4)

        # Parámetros

        self.sample_freq = self.get_parameter('sample_freq').value

        kp_yaw = self.get_parameter('kp_yaw').value
        ki_yaw = self.get_parameter('ki_yaw').value
        kd_yaw = self.get_parameter('kd_yaw').value

        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        self.line_lost_timeout = self.get_parameter('line_lost_timeout').value

        self.alpha = self.get_parameter('filter_alpha').value

        # PID
        self.pid_heading = PID(kp_yaw, ki_yaw, kd_yaw)
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Variables

        # Error recibido desde visión
        self.line_error = 0.0

        # Error filtrado
        self.filtered_error = 0.0

        # Velocidad máxima permitida por semáforo.
        # Verde   -> 0.2
        # Amarillo-> 0.1
        # Rojo    -> 0.0

        self.traffic_max_vel = 0.2

        # Variables de odometría
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Para inicializar filtro
        self.first_sample = True

        # Tiempo de última detección de línea
        self.last_line_time = self.get_clock().now()

        # Tiempo del último ciclo
        self.last_time = self.get_clock().now()

        # Suscriptores

        # Error de línea desde line_node
        self.line_sub = self.create_subscription(Float32,'/line_error',self.line_callback,10)

        # Velocidad máxima desde semáforo
        self.vel_sub = self.create_subscription(Float32,'/max_vel',self.vel_callback,10)

        # Odometría
        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback,10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist,'/cmd_vel',10)

        # Timer
        timer_period = 1.0 / self.sample_freq

        self.timer = self.create_timer(timer_period,self.control_loop)

    # Callback error de linea

    def line_callback(self, msg: Float32):
        raw_error = float(msg.data)
        # Filtro exponencial
        # El error visual tiene ruido.
        if self.first_sample:
            self.filtered_error = raw_error
            self.first_sample = False
        else:
            self.filtered_error = (self.alpha * raw_error + (1 - self.alpha) * self.filtered_error)
        self.line_error = self.filtered_error
        self.last_line_time = self.get_clock().now()

    # Callback semaforo

    def vel_callback(self, msg: Float32):
        # Actualiza velocidad máxima permitida.
        self.traffic_max_vel = float(msg.data)

    # Callback odometria

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Quaternion -> yaw
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w

        siny_cosp = 2.0 * (qw * qz + qx * qy)

        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        self.yaw = math.atan2(siny_cosp,cosy_cosp)

    # Loop de control

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9

        self.last_time = now

        if dt <= 0.0:
            return

        cmd = Twist()

        # Se perdió la linea?

        time_since_line = (now - self.last_line_time).nanoseconds * 1e-9

        if time_since_line > self.line_lost_timeout:

            self.get_logger().warn(
                'LINEA PERDIDA'
            )

            # Reiniciamos PID
            self.pid_heading.reset()

            # Detenemos robot
            self.cmd_pub.publish(cmd)

            return

        # PID angular

        # Error positivo:
        # -> línea a la izquierda
        # -> robot gira izquierda
        # Error negativo:
        # -> línea a la derecha
        # -> robot gira derecha

        angular_cmd = self.pid_heading.update(self.line_error,dt)

        # Saturación angular
        angular_cmd = clamp(angular_cmd,-self.max_angular_vel,self.max_angular_vel)

        # Velocidad lineal
        # La velocidad base viene del semáforo.
        linear_cmd = self.traffic_max_vel

        # Reducción en curvas
        # Entre mayor sea el error: más cerrada es la curva, reducimos velocidad
        # Esto evita: derrapes, salirse de pista y sobrepasar la línea.

        reduction = max(0.3, 1.0 - abs(self.line_error) / 250.0)

        linear_cmd *= reduction

        # Saturación lineal
        linear_cmd = clamp(linear_cmd, 0.0, self.max_linear_vel)

        # Publicar
        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd
        self.cmd_pub.publish(cmd)

    # Callback parametros

    def parameter_callback(self, params):
        for param in params:

            if param.name == 'kp_yaw':
                self.pid_heading.kp = param.value

            elif param.name == 'ki_yaw':
                self.pid_heading.ki = param.value
                self.pid_heading.integral = 0.0

            elif param.name == 'kd_yaw':
                self.pid_heading.kd = param.value

            elif param.name == 'max_linear_vel':
                self.max_linear_vel = param.value

            elif param.name == 'max_angular_vel':
                self.max_angular_vel = param.value

            elif param.name == 'line_lost_timeout':
                self.line_lost_timeout = param.value

            elif param.name == 'filter_alpha':
                self.alpha = param.value

            self.get_logger().info(f'{param.name} updated to {param.value}')

        return SetParametersResult(successful=True)

# Main

def main(args=None):
    rclpy.init(args=args)
    node = PointPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()