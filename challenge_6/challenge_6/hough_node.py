import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class hough_node(Node):

    def __init__(self):
        super().__init__('hough_node')

        #Publisher
        self.error_pub = self.create_publisher(Float32, '/line_error', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)

        self.bridge = CvBridge()

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').value

        self.first_sample = True
        self.filtered_cx = 0.0
        self.declare_parameter('alpha', 0.4)
        self.alpha = self.get_parameter('alpha').value
        
    
    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        imagen = cv.cvtColor(frame, cv.COLOR_BGR2GRAY) #Escala de grises
        imagen_blur = cv.GaussianBlur(imagen, (5,5), 0) #Blur Gaussiano
        imagen_canny = cv.Canny(imagen_blur, 50, 150) #Canny

        height, width = imagen_canny.shape
        roi = imagen_canny[int(height*0.65):height, int(width*0.15):int(width*0.85)]

        #La región de interés se calcula de la siguiente manera: roi = imagen_canny[y_inicio:y_fin, x_inicio:x_fin]
        #Así como lo escribí aquí, cortamos la imagen a la zona de abajo nada más para concentrarnos en la pista

        lines = cv.HoughLinesP(roi, 1, np.pi/180, threshold=50, minLineLength=30, maxLineGap=10)

        #La transformada de Hough es un algoritmo que nos permite detectar figuras geométricas, incluso líneas,
        #de modo que es posible de alguna forma "saber" por dónde va la línea constantemente.

        #La idea general del nodo es esta:
        # Canny nos da los píxeles de borde sin ninguna estructura - solo sabe que ahí hay un cambio brusco de intensidad.
        # Hough toma esos píxeles y busca cuáles de ellos están alineados para formar una línea matemática.
        # Es como si votara: cada píxel de borde "vota" por todas las líneas posibles que podrían pasar por él.
        # Las líneas que reciben más votos son las que realmente existen en la imagen.

        # En el contexto de seguimiento de línea, esto representa una gran ventaja, dado que en lugar de buscar el centroide
        # en un blob, podemos obtener directamente la ecuación de la línea (ángulo y posición). Con eso podemos calcular
        # el error de forma más precisa, especialmente en curvas


        #El flujo de este nodo es básicamente:
        # 1. Canny nos da los bordes
        # 2. HoughLinesP agrupa esos bordes en segmentos
        # 3. De todos los segmentos detectados, calculas el punto medio promedio en X
        # 4. Error = centro del frame − ese punto medio

        #Los parámetros de HoughLinesP tuneables son:
        # - rho: resolución en píxeles, típicamente 1
        # - theta: resolución angular en radianes, típicamente np.pi/180
        # - threshold: mínimo de votos para considerar una línea, entre 50 y 100 para empezar
        # - minLineLength: longitud mínima de un segmento para aceptarlo
        # - maxLineGap: máximo hueco permitido entre dos segmentos para unirlos

        #La función de arriba nos da un array de segmentos o None si es que no detecta nada.

        if lines is None:
            self.get_logger().warn("No se detectó la línea")
            return
        #Este es simplemente el caso de que no detecte nada y no crasheé si algo pasa.

        
        cx = np.mean([(x1 + x2) / 2 for x1, y1, x2, y2 in lines[:, 0]])
        cx_frame = cx + int(width * 0.15)  # convertir cx del ROI a coordenadas del frame

        #Cada segmento de lines queda de una forma: [x1, y1, x2, y2]
        #El punto medio en X de cada segmento es (x1 + x2) / 2
        #Queremos el promedio de todos esos puntos medios

        if self.first_sample:
            self.filtered_cx = cx_frame
            self.first_sample = False
        else:
            self.filtered_cx = self.alpha * cx_frame + (1 - self.alpha) * self.filtered_cx

        if self.debug:
            for x1, y1, x2, y2 in lines[:, 0]:
                cv.line(frame, (x1 + int(width*0.15), y1 + int(height*0.65)), 
                       (x2 + int(width*0.15), y2 + int(height*0.65)), (0, 255, 0), 2)
            cv.circle(frame, (int(self.filtered_cx), int(height*0.65) + (int(height*0.35)//2)), 5, (0, 0, 255), -1)
            cv.rectangle(frame, 
             (int(width*0.15), int(height*0.65)), 
             (int(width*0.85), height), 
             (255, 0, 0), 2)  # azul
            debug_frame = cv.resize(frame, (640, 360))
            cv.imshow('Hough debug', debug_frame)
            cv.waitKey(1)


        error = (width / 2) - self.filtered_cx
        #Error positivo -> La línea está a la izquierda
        #Error negativo -> La línea está a la derecha

        msg = Float32()
        msg.data = float(error)
        self.error_pub.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = hough_node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()