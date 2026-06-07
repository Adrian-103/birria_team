from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
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


class LinePIDController(Node):
    """
    PID de seguimiento de línea (solo velocidad angular).

    Entrada:  /line_error  (std_msgs/Float32)  -- error de la línea respecto al centro
    Salida:   /ang_vel     (std_msgs/Float32)  -- velocidad angular deseada en rad/s

    Este nodo NO genera velocidad lineal y NO maneja pérdida de línea.
    El nodo de control central combina /ang_vel con la velocidad lineal y
    decide el /cmd_vel final usando /line_status.
    """

    def __init__(self):
        super().__init__('line_pid_controller')

        # Frecuencia del controlador
        self.declare_parameter('sample_freq', 60.0)

        # PID angular
        self.declare_parameter('kp_yaw', 0.008)
        self.declare_parameter('ki_yaw', 0.0)
        self.declare_parameter('kd_yaw', 0.001)

        # Saturación de velocidad angular (rad/s)
        self.declare_parameter('max_angular_vel', 0.2)

        # Filtro exponencial para suavizar el error visual
        self.declare_parameter('filter_alpha', 0.4)

        # Tolerancia del error (deadband) para reducir oscilaciones
        # Si |error| < tolerance -> salida angular = 0
        # En las mismas unidades que /line_error (píxeles, o [-1,1] si normalize_error=true)
        # 0.0 = deshabilitado
        self.declare_parameter('error_tolerance', 0.0)

        # Leer parámetros
        self.sample_freq     = self.get_parameter('sample_freq').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.alpha           = self.get_parameter('filter_alpha').value
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
        self.last_time      = self.get_clock().now()

        # Suscriptor / publicador
        self.create_subscription(Float32, '/line_error', self.line_callback, 10)
        self.ang_pub = self.create_publisher(Float32, '/ang_vel', 10)

        self.create_timer(1.0 / self.sample_freq, self.control_loop)
        self.get_logger().info('line_pid_controller listo. Publicando en /ang_vel.')

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

    # ------------------------------------------------------------------ #

    def control_loop(self):
        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # Deadband suave: dentro de la tolerancia el error efectivo es 0,
        # justo afuera arranca desde 0 y crece suavemente (sin saltos).
        if abs(self.line_error) < self.error_tolerance:
            effective_error = 0.0
        else:
            effective_error = self.line_error - math.copysign(self.error_tolerance, self.line_error)

        # PID angular
        angular_cmd = self.pid.update(effective_error, dt)
        angular_cmd = clamp(angular_cmd, -self.max_angular_vel, self.max_angular_vel)

        self.ang_pub.publish(Float32(data=float(-angular_cmd)))

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
            elif param.name == 'max_angular_vel':
                self.max_angular_vel = param.value
            elif param.name == 'filter_alpha':
                self.alpha = param.value
            elif param.name == 'error_tolerance':
                self.error_tolerance = param.value
            self.get_logger().info(f'{param.name} -> {param.value}')
        return SetParametersResult(successful=True)


# ------------------------------------------------------------------ #

def main(args=None):
    rclpy.init(args=args)
    node = LinePIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Publicar 0 angular al salir para no dejar un comando viejo activo
        node.ang_pub.publish(Float32(data=0.0))
        rclpy.spin_once(node, timeout_sec=0.2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
