import math
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path, Odometry

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

# Usamos tu misma clase PID intacta
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

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower_node')
        
        # PARAMETROS (Puedes ajustarlos después)
        self.max_linear_vel = 0.2
        self.max_angular_vel = 1.5
        self.sample_freq = 50
        self.point_tolerance = 0.08  # 8 cm para cambiar al siguiente punto rápido
        self.angle_tolerance = 0.15  # Radianes

        # PIDs con los valores que tenías
        self.pid_distance = PID(kp=1.2, ki=0.0, kd=0.08)
        self.pid_heading = PID(kp=3.0, ki=0.0, kd=0.15)

        # Estado del Robot (Se actualizará con /odom)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.path_queue = []
        self.goal_received = False

        self.last_time = self.get_clock().now() 

        # Suscriptores y Publicadores (La nueva arquitectura)
        self.path_sub = self.create_subscription(Path, '/ruta_planeada', self.path_callback, 10)		
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer del ciclo de control
        timer_period = 1.0 / self.sample_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

    def path_callback(self, msg: Path):
        # Si recibimos una ruta nueva, limpiamos la cola actual y cargamos la nueva
        if len(msg.poses) > 0 and not self.goal_received:
            self.path_queue = []
            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                self.path_queue.append((x, y))
            
            self.get_logger().info(f"Ruta recibida con {len(self.path_queue)} puntos. ¡Iniciando persecución!")
            self.goal_received = True
            self.pid_distance.reset()
            self.pid_heading.reset()

    def odom_callback(self, msg: Odometry):
        # Actualizamos nuestra posición leyendo la odometría oficial
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extraer Yaw del cuaternión
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        cmd = Twist()

        # Si no hay ruta o ya terminamos, nos detenemos
        if not self.goal_received or len(self.path_queue) == 0:
            self.cmd_pub.publish(cmd)
            return

        # Vemos a dónde tenemos que ir (el punto actual)
        goal_x, goal_y = self.path_queue[0]

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance_error = math.sqrt(dx * dx + dy * dy)

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)

        # ¿Ya llegamos lo suficientemente cerca del punto actual?
        if distance_error < self.point_tolerance:
            self.get_logger().info(f"Punto alcanzado. Faltan {len(self.path_queue) - 1} puntos.")
            self.path_queue.pop(0) # Quitamos el punto alcanzado
            
            if len(self.path_queue) == 0:
                self.goal_received = False
                self.get_logger().info("¡LLEGAMOS A LA META DEL LABERINTO!")
                self.cmd_pub.publish(cmd) # Freno total
                return
            else:
                # Reseteamos el PID para el siguiente punto
                self.pid_distance.reset()
                self.pid_heading.reset()
                return            

        # --- Lógica de Control ---
        linear_cmd = self.pid_distance.update(distance_error, dt)
        angular_cmd = self.pid_heading.update(heading_error, dt)

        # Si estamos muy desalineados, rotamos sobre nuestro propio eje antes de avanzar
        if abs(heading_error) > self.angle_tolerance:
            linear_cmd = 0.0
        else:
            linear_cmd *= max(0.0, math.cos(heading_error)) # Suaviza el avance en curvas

        # Limitamos velocidades para no quemar motores
        linear_cmd = clamp(linear_cmd, 0.0, self.max_linear_vel) # Solo avanzar hacia adelante
        angular_cmd = clamp(angular_cmd, -self.max_angular_vel, self.max_angular_vel)

        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        # ¡Publicamos la velocidad a las llantas!
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
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