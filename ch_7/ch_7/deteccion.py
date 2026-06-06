# -------------------------
#  Antes de correr:
#    1. Copia best.pt desde Colab a la Rubik Pi
#    2. Instalar: pip install ultralytics opencv-python --break-system-packages
#
#  Copiar best.pt al mismo directorio que este archivo
# -------------------------

# -------------------------
#  Librerias
# -------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2 as cv
import numpy as np
from ultralytics import YOLO

# ---------------------------   
#  Paleta colores YOLO
# ---------------------------
PALETA = [
    (52,  152, 219),   # azul
    (46,  204, 113),   # verde
    (231, 76,  60 ),   # rojo
    (241, 196, 15 ),   # amarillo
    (155, 89,  182),   # morado
    (26,  188, 156),   # turquesa
]

# ---------------------------   
#  Nodo Deteccion
# ---------------------------
class TrafficSignNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_node')

        # ---------------------------   
        #  Parámetros configurables
        # ---------------------------
        self.declare_parameter('debug', False)   
        self.declare_parameter('confianza',  0.50)   # umbral mínimo de confianza (50%)
        self.declare_parameter('ancho', 160)         # resolución de procesamiento
        self.declare_parameter('alto', 140)          
        self.declare_parameter('fps', 3)             # frecuencia de inferencia

        self.debug = self.get_parameter('debug').value
        self.confianza = self.get_parameter('confianza').value
        self.ancho = self.get_parameter('ancho').value
        self.alto = self.get_parameter('alto').value
        self.fps = self.get_parameter('fps').value

        # ---------------------------
        #  Modelo YOLOv11
        # ---------------------------
        try:
            self.model = YOLO('/home/jess/ros2_ws/src/birria_team/ch_7/ch_7/best.pt')
            self.nombres = self.model.names  # Diccionario de las señales que tenemos
            self.get_logger().info(
                f'Modelo cargado, clases: {list(self.nombres.values())}'
            )
        except Exception as e:
            self.get_logger().error(f'Error cargando modelo: {e}')
            self.model = None

        # Variables internas ---------------------------
        self.bridge = CvBridge()
        self.last_sign = None               # para no repetir el mismo log
        self.last_frame = None              # guarda el último frame

        if self.debug:
            cv.namedWindow('Deteccion Señales', cv.WINDOW_NORMAL)
            cv.resizeWindow('Deteccion Señales', 640, 480)


        # ---------------------------
        #  Subscripción cámara
        # ---------------------------
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw', 
            self.camera_callback,
            10
            )
        
        # ---------------------------
        #  Publicadores
        # ---------------------------
        # Para señal detectada
        self.sign_pub = self.create_publisher(String, 'detected_sign', 10)

        # Para la imagen con bounding boxes
        self.img_pub = self.create_publisher(Image, '/inference_result', 10)

        # ---------------------------
        #  Timer para inferencia (FPS)
        # ---------------------------
        # Para no saturar el procesador de la Rubik Pi
        T = 1.0 / self.fps              # 1/3 segundos entre inferencias
        self.timer = self.create_timer(T, self.timer_callback)

        self.get_logger().info(f'Nodo listo {self.ancho}x{self.alto} @ {self.fps} FPS')


    def camera_callback(self, msg):
        try:
            # Mensaje ROS2 a imagen OpenCV (BGR)
            self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')


    def timer_callback(self):
        if self.last_frame is None:
            self.get_logger().info('Esperando frame', throttle_duration_sec=3.0)
            return

        if self.model is None:
            return
        
        # ---------------------------
        #  Resolución
        # ---------------------------
        frame_s = cv.resize(
            self.last_frame,
            (self.ancho, self.alto),
            interpolation=cv.INTER_LINEAR          # Método de reducción de calidad
        )

        # ---------------------------
        #  Inferencia YOLOv11
        # ---------------------------
        res = self.model.predict(
            source = frame_s,
            imgsz = 160,
            conf = self.confianza,
            verbose = False,
        ) [0]

        # Extraer os resultados de las inferencias
        boxes = res.boxes.xyxy.cpu().numpy()        # Coordenadas
        clases = res.boxes.cls.cpu().numpy()        # Indice de clases
        confianzas = res.boxes.conf.cpu().numpy()   # Porcentaje

        # Nombre de señal detectada ---------------------------
        if len(clases) > 0:
            for cls, conf in zip(clases, confianzas):
                nombre = self.nombres[int(cls)]
                msg_sign = String()
                msg_sign.data = nombre
                self.sign_pub.publish(msg_sign)

                 # Log solo cuando cambia la señal detectada
                if nombre != self.last_sign:
                    self.get_logger().info(f'Señal: {nombre} ({conf:.0%})')
                    self.last_sign = nombre
        else:
            # No hubo detecciones
            if self.last_sign is not None:
                self.get_logger().info('Sin señal')
                self.last_sign = None

        # Bounding boxes ---------------------------
        frame_a = self._dibujar(
            frame_s.copy(), boxes, clases, confianzas
        )

        img_msg = self.bridge.cv2_to_imgmsg(frame_a, encoding='bgr8')
        self.img_pub.publish(img_msg)

        # Debug ---------------------------
        if self.debug:
            frame_d = cv.resize(frame_a, (640, 480), interpolation=cv.INTER_LINEAR)
            cv.imshow('Deteccion Señales', frame_d)
            cv.waitKey(1)


    def _dibujar(self, frame, boxes, clases, confianzas):
        """Dibuja bounding boxes y etiquetas sobre el frame."""
        for box, cls, conf in zip(boxes, clases, confianzas):
            x1, y1, x2, y2 = map(int, box)
            color = PALETA[int(cls) % len(PALETA)]
            label = f"{self.nombres[int(cls)]} {conf:.0%}"

            # Rectángulo de detección
            cv.rectangle(frame, (x1, y1), (x2, y2), color, 1)

            # Fondo de etiqueta
            (tw, th), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            cv.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)

            # Texto
            cv.putText(frame, label, (x1 + 2, y1 - 3),
                       cv.FONT_HERSHEY_SIMPLEX, 0.4,
                       (255, 255, 255), 1, cv.LINE_AA)
        return frame


    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
