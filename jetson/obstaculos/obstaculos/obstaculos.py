import rclpy
import time
import Jetson.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import String

class obstaculos(Node):

    def __init__(self):
        super().__init__('obstaculos')

        #Publisher:
        self.status_pub = self.create_publisher(String, '/obstaculo', 10)

        #Timer:
        timer_period = 1 #1 segundo
        self.timer = self.create_timer(timer_period, self.timer_callback)

        #Pines:
        self.TRIG = 35 #GPIO 76
        self.ECHO = 37 #GPIO 12

        GPIO.setmode(GPIO.BOARD)

        #Configurar dirección de los pines
        GPIO.setup(self.TRIG, GPIO.OUT)
        GPIO.setup(self.ECHO, GPIO.IN)

        GPIO.output(self.TRIG, False)

    def timer_callback(self):
        
        #Pulso del trigger:
        GPIO.output(self.TRIG, True)
        time.sleep(0.00001)  # 10µs
        GPIO.output(self.TRIG, False)

        #Esperar ECHO HIGH
        pulse_start = time.time()
        timeout_start = time.time()
        while GPIO.input(self.ECHO) == 0:
            pulse_start = time.time()
            if time.time() - timeout_start > 0.1:  # timeout 100ms
                self.get_logger().warn('Timeout esperando ECHO HIGH')
                msg = String()
                msg.data = "no_caja"
                self.status_pub.publish(msg)
                return
            
         # Esperar ECHO LOW
        pulse_end = time.time()
        timeout_start = time.time()
        while GPIO.input(self.ECHO) == 1:
            pulse_end = time.time()
            if time.time() - timeout_start > 0.1:  # timeout 100ms
                self.get_logger().warn('Timeout esperando ECHO HIGH')
                msg = String()
                msg.data = "no_caja"
                self.status_pub.publish(msg)
                return

        # Calcular distancia en cm
        distancia = (pulse_end - pulse_start) * 17150

        msg = String()
        self.get_logger().info(f'Distancia calculada: {distancia:.2f} cm')
        if distancia <= 15.0:
            msg.data = "caja"
        else:
            msg.data = "no_caja"

        self.status_pub.publish(msg)
        self.get_logger().info(f'Distancia: {distancia:.1f} cm -> {msg.data}')




def main(args=None):
    rclpy.init(args=args)
    node = obstaculos()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()