from dataclasses import dataclass

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


@dataclass
class PID:
    kp: float
    ki: float
    kd: float
    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error, dt):
        if dt <= 0.0:
            return 0.0
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class LineFollowerPID(Node):

    def __init__(self):
        super().__init__('line_follower_pid')

        # Frecuencia del controlador
        self.declare_parameter('sample_freq', 60.0)

        # PID angular
        self.declare_parameter('kp_yaw', 0.008)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.001)

        # Velocidad lineal fija para pruebas
        self.declare_parameter('linear_vel', 0.1)
        self.declare_parameter('max_angular_vel', 0.2)

        # Si no llega error en este tiempo -> línea perdida
        self.declare_parameter('line_lost_timeout', 0.5)

        # Filtro exponencial para suavizar el error visual
        self.declare_parameter('filter_alpha', 0.4)

        # Tolerancia del error (deadband) para reducir oscilaciones
        # Si |error| < tolerance -> salida angular = 0
        # En las mismas unidades que /line_error (píxeles, o [-1,1] si normalize_error=true)
        # 0.0 = deshabilitado
        self.declare_parameter('error_tolerance', 0.0) 

        # Leer parámetros
        self.sample_freq      = self.get_parameter('sample_freq').value
        self.linear_vel       = self.get_parameter('linear_vel').value
        self.max_angular_vel  = self.get_parameter('max_angular_vel').value
        self.line_lost_timeout = self.get_parameter('line_lost_timeout').value
        self.alpha            = self.get_parameter('filter_alpha').value
        self.error_tolerance = self.get_parameter('error_tolerance').value 

        self.pid = PID(
            kp=self.get_parameter('kp_yaw').value,
            ki=self.get_parameter('ki_yaw').value,
            kd=self.get_parameter('kd_yaw').value,
        )

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Estado
        self.line_error     = 0.0
        self.filtered_error = 0.0
        self.first_sample   = True
        self.last_line_time = self.get_clock().now()
        self.last_time      = self.get_clock().now()

        # Suscriptor / publicador
        self.create_subscription(Float32, '/line_error', self.line_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(1.0 / self.sample_freq, self.control_loop)
        self.get_logger().info('line_follower_pid listo.')

    # ------------------------------------------------------------------ #

    def line_callback(self, msg: Float32):
        raw = float(msg.data)
        # Filtro exponencial
        if self.first_sample:
            self.filtered_error = raw
            self.first_sample = False
        else:
            self.filtered_error = self.alpha * raw + (1.0 - self.alpha) * self.filtered_error
        self.line_error = self.filtered_error
        self.last_line_time = self.get_clock().now()

    # ------------------------------------------------------------------ #

    def control_loop(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        cmd = Twist()

        # Línea perdida -> detener
        if (now - self.last_line_time).nanoseconds * 1e-9 > self.line_lost_timeout:
            self.get_logger().warn('LINEA PERDIDA', throttle_duration_sec=1.0)
            self.pid.reset()
            self.cmd_pub.publish(cmd)
            return

        # Deadband suave: dentro de la tolerancia el error efectivo es 0,
        # justo afuera arranca desde 0 y crece suavemente (sin saltos).
        if abs(self.line_error) < self.error_tolerance:
            effective_error = 0.0
        else:
            sign = 1.0 if self.line_error > 0 else -1.0
            effective_error = self.line_error - sign * self.error_tolerance

        # PID angular
        angular_cmd = self.pid.update(effective_error, dt)
        angular_cmd = clamp(angular_cmd, -self.max_angular_vel, self.max_angular_vel)

        # Velocidad lineal: reducir en curvas pronunciadas
        # Entre mayor error -> curva más cerrada -> menos velocidad
        reduction  = max(0.3, 1.0 - abs(self.line_error) / 250.0)
        linear_cmd = clamp(self.linear_vel * reduction, 0.0, self.linear_vel)

        cmd.linear.x  = linear_cmd
        cmd.angular.z = -angular_cmd
        self.cmd_pub.publish(cmd)

    # ------------------------------------------------------------------ #

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'kp_yaw':
                self.pid.kp = param.value
            elif param.name == 'ki_yaw':
                self.pid.ki = param.value
                self.pid.integral = 0.0
            elif param.name == 'kd_yaw':
                self.pid.kd = param.value
            elif param.name == 'linear_vel':
                self.linear_vel = param.value
            elif param.name == 'max_angular_vel':
                self.max_angular_vel = param.value
            elif param.name == 'line_lost_timeout':
                self.line_lost_timeout = param.value
            elif param.name == 'filter_alpha':
                self.alpha = param.value
            elif param.name == 'error_tolerance':
                self.error_tolerance = param.value 
            self.get_logger().info(f'{param.name} -> {param.value}')
        return SetParametersResult(successful=True)


# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())   # parar motores al salir
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
