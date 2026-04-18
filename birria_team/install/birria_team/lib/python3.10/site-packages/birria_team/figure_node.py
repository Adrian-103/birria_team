import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from rcl_interfaces.msg import SetParametersResult

class PolygonController(Node):
    def __init__(self):
        super().__init__('polygon_controller')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Parámetro dinámico
        self.declare_parameter('n_lados', 4)

        # Velocidad lineal
        self.v = 0.1
        # Velocidad angular
        self.w = 0.15
        # Tiempo de avance
        self.t_avance = 5.5

        # Tiempos de giro por cada lado
        self.tiempos_giro = {
            3: 13.0,   
            4: 9.5,   
            5: 7.75,
            6: 6.5,
            7: 5.5,
            8: 4.75
        }

        self.n_lados = self.get_parameter('n_lados').value
        self.t_giro = self.tiempos_giro.get(self.n_lados, 15.0)

        # Estado
        self.state = "avanzar"
        self.start_time = time.time()
        self.lado_actual = 0

        self.add_on_set_parameters_callback(self.param_callback)
        self.timer = self.create_timer(0.1, self.loop)

    def param_callback(self, params):
        for param in params:
            if param.name == 'n_lados':
                if param.value in self.tiempos_giro:
                    self.n_lados = param.value
                    self.t_giro = self.tiempos_giro[self.n_lados]

                    self.get_logger().info(
                        f"Figura: {self.n_lados} lados"
                    )

                    # Reiniciar figura
                    self.lado_actual = 0
                    self.state = "avanzar"
                    self.start_time = time.time()

        return SetParametersResult(successful=True)

    def loop(self):
        vel = Twist()
        elapsed = time.time() - self.start_time

        if self.lado_actual >= self.n_lados:
            self.pub.publish(Twist())
            return

        if self.state == "avanzar":
            vel.linear.x = self.v

            if elapsed >= self.t_avance:
                self.state = "girar"
                self.start_time = time.time()

        elif self.state == "girar":
            vel.angular.z = self.w

            if elapsed >= self.t_giro:
                self.state = "avanzar"
                self.start_time = time.time()
                self.lado_actual += 1

        self.pub.publish(vel)


def main():
    rclpy.init()
    node = PolygonController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()