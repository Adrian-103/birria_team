import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
  )

def normalize_angle(angle):
    #Hace que el angulo se mantenga entre -pi y pi
    #Con while loop por si se pasa mas de una vuelta

    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


class odometry_node(Node):
    def __init__(self):
        super().__init__('odometry_node')

        #Parameters
        self.declare_parameter('wheel_radius', 0.0582)
        self.declare_parameter('width', 0.183)
        self.declare_parameter('filter_alpha', 0.3)
        self.declare_parameter('sample_freq', 50)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.width = self.get_parameter('width').value
        self.alpha = self.get_parameter('filter_alpha').value
        self.sample_freq = self.get_parameter('sample_freq').value

        #Coordenadas para la odometria
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        #Velocidades del encoder en rad/s
        self.w_l = 0.0
        self.w_r = 0.0

        self.last_time = self.get_clock().now()

        self.first_sample_r = True
        self.first_sample_l = True

        #Subscriptores y publishers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.enc_l_sub = self.create_subscription(Float32, '/VelocityEncL', self.encL_callback, qos_profile)
        self.enc_r_sub = self.create_subscription(Float32, '/VelocityEncR', self.encR_callback, qos_profile)

        timer_period = 1.0 / self.sample_freq
        self.timer = self.create_timer(timer_period, self.odom_loop)

    def encL_callback(self, msg: Float32):
        raw = float(msg.data)

        if self.first_sample_l:
            self.w_l = raw
            self.first_sample_l = False
        else:
            self.w_l = self.alpha * raw + (1 - self.alpha)     * self.w_l


    def encR_callback(self, msg: Float32):
        raw = float(msg.data)

        if self.first_sample_r:
            self.w_r = raw
            self.first_sample_r = False
        else:
            self.w_r = self.alpha * raw + (1 - self.alpha)     * self.w_r

    def odom_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        #Modelo cinematico de robot diferencial 
        v_l = self.wheel_radius * self.w_l
        v_r = self.wheel_radius * self.w_r
 
        v = 0.5 * (v_r + v_l)
        w = (v_r - v_l) / self.width
 
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw = normalize_angle(self.yaw + w * dt)

        odom = Odometry()

        # 3. Set the Header
        odom.header.stamp = now.to_msg() 
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link" # The frame of your robot

        # 4. Set the Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # 5. Set the Orientation (Converting yaw in radians to a Quaternion)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # 6. Set the Velocity (Twist)
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = w

        # 7. Publish the Odometry message
        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = odometry_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
