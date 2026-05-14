import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class line_node(Node):

    def __init__(self):
        super().__init__('line_node')

        #Publisher
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        imagen = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #Escala de grises
        imagen_blur = cv.GaussianBlur(imagen, (5,5), 0) #Blur Gaussiano
        imagen_canny = cv.Canny(imagen_blur, 50, 150) #Canny

        height, width = imagen_canny.shape
        roi = imagen_canny[height//2:height, 0:width]

        #La región de interés se calcula de la siguiente manera: roi = imagen_canny[y_inicio:y_fin, x_inicio:x_fin]
        #Así como lo escribí aquí, cortamos la imagen a la zona de abajo nada más para concentrarnos en la pista

        num_labels, labels, stats, centroids = cv.connectedComponentsWithStats(roi)

        #La función anterior nos devuelve lo siguiente:
        # -> num_labels: número de componentes encontrados incluyendo el fondo
        # -> labels: imagen donde cada píxel tiene el número de sus componentes
        # -> stats: información de cada componente (área, posición, tamaño)
        # -> centroids: coordenadas del centroide de cada componente

        #Hay que recordar la siguiente información:
        #El centroide 0 siempre es el fondo. entonces los componentes reales inician desde el índice [1].

        #Lo que queremos es encontrar el componente más grande (que debería de ser la línea) e ignorar los demás.
        #Para eso usamos stats que tiene el área de cada componente en la columna en cv.CC_STAT_AREA

        if num_labels <= 1:
            self.get_logger().warn('No se detectó la línea')
            return
        
        largest = np.argmax(stats[1:, cv.CC_STAT_AREA]) + 1

        cx = centroids[largest][0] #Coordenada X del centroide

        #Solo necesitamos el eje X porque el error es horizontal - qué tan lejos está la línea del centro del frame.

        error = (width/2) - cx
        #Error positivo -> La línea está a la izquierda
        #Error negativo -> La línea está a la derecha

        msg = Float32()
        msg.data = float(error)
        self.error_pub.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = line_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()