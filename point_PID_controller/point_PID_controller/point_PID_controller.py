import math
from dataclasses import dataclass
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def normalize_angle(angle):
    #Hace que el angulo se mantenga entre -pi y pi
    #Con while loop por si se pasa mas de una vuelta

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle

#Se crea una clase para el PID pq van a ser PIDs, uno para la vel lineal y otro para la angular
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
        derivative = (error - self.prev_error)/dt

        self.prev_error = error
        return self.kp*error + self.ki*self.integral + self.kd*derivative

class PointPIDController(Node):
    def __init__(self):
        super().__init__('point_PID_controller')
        
        #PARAMETROS

        #Mismos valores que en la hackerboard
        self.declare_parameter('wheel_radius', 0.0505)
        self.declare_parameter('width', 0.183)

        #Velocidades maximas
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 1.5)

        #Frecuencia de sampleo del PID en Hz
        self.declare_parameter('sample_freq', 50)

        #Tolerancia de error en el punto
        self.declare_parameter('point_tolerance', 0.05) #en metros
        self.declare_parameter('angle_tolerance', 0.10) #radianes

        #PID de distancia
        self.declare_parameter('kp_dist', 1.2) 
        self.declare_parameter('ki_dist', 0.0) 
        self.declare_parameter('kd_dist', 0.08) 

        #PID de direccion
        self.declare_parameter('kp_yaw', 3.0) 
        self.declare_parameter('ki_yaw', 0.0) 
        self.declare_parameter('kd_yaw', 0.15) 

        #Filtro exponencial
        self.declare_parameter('filter_alpha', 0.5)
        self.first_sample = True

        #OBTENER PARAMETROS
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.width = self.get_parameter('width').value
        self.sample_freq = self.get_parameter('sample_freq').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.point_tolerance = self.get_parameter('point_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.alpha = self.get_parameter('filter_alpha').value

        kp_dist = self.get_parameter('kp_dist').value
        ki_dist = self.get_parameter('ki_dist').value
        kd_dist = self.get_parameter('kd_dist').value

        kp_yaw = self.get_parameter('kp_yaw').value
        ki_yaw = self.get_parameter('ki_yaw').value
        kd_yaw = self.get_parameter('kd_yaw').value

        #Controladores PID
        self.pid_distance = PID(kp_dist, ki_dist, kd_dist)
        self.pid_heading = PID(kp_yaw, ki_yaw, kd_yaw)

        self.add_on_set_parameters_callback(self.parameter_callback)

        #Estado del Puzzlebot
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.current_goal = None
        self.goal_received = False
        self.goal_queue = []

        #Velocidades del encoder en rad/s
        self.w_l = 0.0
        self.w_r = 0.0

        self.last_time = self.get_clock().now() 

        #Subscriptores y publishers 
        self.goal_sub = self.create_subscription(Point, '/goal', self.goal_callback, 10)		
        self.enc_l_sub = self.create_subscription(Float32, '/VelocityEncL', self.encL_callback, qos_profile)
        self.enc_r_sub = self.create_subscription(Float32, '/VelocityEncR', self.encR_callback, qos_profile)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pose_pub = self.create_publisher(Point, '/puzzlebot_pose', 10)

        timer_period = 1.0 / self.sample_freq
        self.timer = self.create_timer(timer_period, self.control_loop)

    def goal_callback(self, msg: Point):

        if len(self.goal_queue) < 10:
            x = float(msg.x)
            y = float(msg.y)
            self.goal_queue.append((x, y))
            self.get_logger().info(
            f'New goal received: x={x:.3f}, y={y:.3f}'
        	)

            if self.goal_received == False:
                self.current_goal = self.goal_queue.pop(0)
                self.goal_received = True
                self.pid_distance.reset()
                self.pid_heading.reset()

    def encL_callback(self, msg: Float32):
        raw = float(msg.data)

        if self.first_sample:
            self.w_l = raw
        else:
            self.w_l = self.alpha * raw + (1 - self.alpha) * self.w_l


    def encR_callback(self, msg: Float32):
        raw = float(msg.data)

        if self.first_sample:
            self.w_r = raw
            self.first_sample = False
        else:
            self.w_r = self.alpha * raw + (1 - self.alpha) * self.w_r

    def update_odometry(self, dt):
        pose = Point()

        #Localizacion con dead reckoning
        v_l = self.wheel_radius * self.w_l
        v_r = self.wheel_radius * self.w_r

        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / self.width

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + w * dt)

        pose.x = self.x
        pose.y = self.y
        pose.z = self.yaw * 180 / math.pi

        self.pose_pub.publish(pose)

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        #Se actualiza constantemente la localizacion del robot
        self.update_odometry(dt)

        cmd = Twist()

        #Si todavia no recibe nada, no hace nada
        if not self.goal_received:
            self.cmd_pub.publish(cmd)
            return

        goal_x, goal_y = self.current_goal

        dx = goal_x - self.x
        dy = goal_y - self.y
        distance_error = math.sqrt(dx * dx + dy * dy)

        target_heading = math.atan2(dy, dx)
        heading_error = normalize_angle(target_heading - self.yaw)

        #Si el error es menor a la tolerancia, llegamos a la meta
        if distance_error < self.point_tolerance:
            if abs(heading_error) < self.angle_tolerance:
                #Detiene el robot
                self.cmd_pub.publish(cmd)

                self.get_logger().info(
                    f'Goal reached: x={self.x:.3f}, y={self.y:.3f}'
                )
                
                #Cambiamos a la nueva meta
                if len(self.goal_queue) > 0:
                    self.current_goal = self.goal_queue.pop(0)
                else:
                    self.goal_received = False

                self.pid_distance.reset()
                self.pid_heading.reset()
                return            

        #Salida PID
        linear_cmd = self.pid_distance.update(distance_error, dt)
        angular_cmd = self.pid_heading.update(heading_error, dt)

        #Si existe error angular mayor a la tolerancia, solo gira
        if abs(heading_error) > self.angle_tolerance:
            linear_cmd = 0.0
        else:
            linear_cmd *= max(0.0, math.cos(heading_error))

        #Se limitan las velocidades
        linear_cmd = clamp(linear_cmd, -self.max_linear_vel, self.max_linear_vel)
        angular_cmd = clamp(angular_cmd, -self.max_angular_vel, self.max_angular_vel)

        #Se limita la velocidad lineal a positiva
        linear_cmd = max(0.0, linear_cmd)

        cmd.linear.x = linear_cmd
        cmd.angular.z = angular_cmd

        #Se publica en /cmd_vel
        self.cmd_pub.publish(cmd)

    def parameter_callback(self, params):
        for param in params:
            #PID distancia
            if param.name == 'kp_dist':
                self.pid_distance.kp = param.value
            elif param.name == 'ki_dist':
                self.pid_distance.ki = param.value
                self.pid_distance.integral = 0.0 #Se reinicia la integral si se cambia el valor
            elif param.name == 'kd_dist':
                self.pid_distance.kd = param.value

            #PID direccion
            elif param.name == 'kp_yaw':
                self.pid_heading.kp = param.value
            elif param.name == 'ki_yaw':
                self.pid_heading.ki = param.value
            elif param.name == 'kd_yaw':
                self.pid_heading.kd = param.value

            #tolerancias
            elif param.name == 'point_tolerance':
                self.point_tolerance = param.value
            elif param.name == 'angle_tolerance':
                self.angle_tolerance = param.value

            #frecuencia de sampleo
            elif param.name == 'sample_freq':
                self.sample_freq = param.value

            #Velocidades maximas
            elif param.name == 'max_linear_vel':
                self.max_linear_vel = param.value
            elif param.name == 'max_angular_vel':
                self.max_angular_vel = param.value

            #Medidas del puzzlebot
            elif param.name == 'wheel_radius':
                self.wheel_radius = param.value
            elif param.name == 'width':
                self.width = param.value

            #Para el filtro exponencial
            elif param.name == 'filter_alpha':
                self.alpha = param.value


        self.get_logger().info(f'{param.name} updated to {param.value}')
        return SetParametersResult(successful=True)

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
