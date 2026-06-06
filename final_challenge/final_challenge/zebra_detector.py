import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

class ZebraDetector(Node):
    def __init__(self):
        super().__init__('zebra_detector_node')
        self.bridge = CvBridge()
        
        # --- DECLARACIÓN DE PARÁMETROS ---
        # Umbral para binarizar (0-255). Lo que sea menor a esto se vuelve blanco (activo)
        self.declare_parameter('bin_threshold', 80) 
        
        # Dimensiones del Kernel de Dilatación
        self.declare_parameter('kernel_width', 30)
        self.declare_parameter('kernel_height', 5)
        
        # Umbral para decidir que sí hay cebra (% de píxeles activos)
        self.declare_parameter('detection_thresh', 40.0) 
        
        # Geometría del Trapecio (Valores normalizados de 0.0 a 1.0 respecto al tamaño de la imagen)
        self.declare_parameter('roi_top_width', 0.4)    # Ancho del trapecio arriba (40% de la imagen)
        self.declare_parameter('roi_bottom_width', 0.9) # Ancho del trapecio abajo (90% de la imagen)
        self.declare_parameter('roi_height_offset', 0.6)# Dónde empieza el trapecio (60% hacia abajo)

        # Cargar valores iniciales
        self.update_params()
        
        # Callback para actualizar parámetros en vivo desde RQT
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Suscriptores y Publicadores
        # OJO: Cambia '/camera/image_raw' por el tópico real de tu cámara
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.pub_detect = self.create_publisher(Bool, '/interseccion_detectada', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/vision/zebra_debug/compressed', 10)
        
        self.get_logger().info("¡Detector de Cebra Trapezoidal Iniciado!")

    def update_params(self):
        self.thresh_val = self.get_parameter('bin_threshold').value
        self.k_w = self.get_parameter('kernel_width').value
        self.k_h = self.get_parameter('kernel_height').value
        self.det_th = self.get_parameter('detection_thresh').value
        self.rtw = self.get_parameter('roi_top_width').value
        self.rbw = self.get_parameter('roi_bottom_width').value
        self.rho = self.get_parameter('roi_height_offset').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parámetro actualizado: {param.name} = {param.value}')
        self.update_params()
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # 1. Convertir de ROS a OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f"Error convirtiendo imagen: {e}")
            return

        alto, ancho = frame.shape[:2]

        # 2. Definir los 4 puntos del trapecio según los parámetros
        # Arriba-Izq, Arriba-Der, Abajo-Der, Abajo-Izq
        tl = [int(ancho * (0.5 - self.rtw/2)), int(alto * self.rho)]
        tr = [int(ancho * (0.5 + self.rtw/2)), int(alto * self.rho)]
        br = [int(ancho * (0.5 + self.rbw/2)), alto]
        bl = [int(ancho * (0.5 - self.rbw/2)), alto]
        
        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))

        # 3. Binarización INVERTIDA (Piso blanco -> 0, Líneas Negras -> 255)
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binarizada = cv2.threshold(gris, self.thresh_val, 255, cv2.THRESH_BINARY_INV)

        # 4. Dilatación en toda la imagen (Unimos la cebra)
        kernel = np.ones((self.k_h, self.k_w), np.uint8)
        dilatada = cv2.dilate(binarizada, kernel, iterations=2)

        # 5. Crear máscara trapezoidal y extraer solo lo que nos importa
        mask = np.zeros_like(dilatada)
        cv2.fillPoly(mask, [pts], 255)
        roi_final = cv2.bitwise_and(dilatada, mask)

        # 6. Calcular el porcentaje de píxeles activos DENTRO del trapecio
        area_trapecio = cv2.countNonZero(mask) # Cuántos píxeles mide nuestro trapecio
        if area_trapecio > 0:
            blancos = cv2.countNonZero(roi_final)
            porcentaje = (blancos / area_trapecio) * 100.0
        else:
            porcentaje = 0.0

        # 7. Publicar decisión booleana
        msg_bool = Bool()
        msg_bool.data = bool(porcentaje > self.det_th)
        self.pub_detect.publish(msg_bool)

        # 8. Generar imagen de Debug (Para visualizar en rqt_image_view)
        debug_img = cv2.cvtColor(roi_final, cv2.COLOR_GRAY2BGR)
        # Dibujamos el contorno del trapecio en verde
        cv2.polylines(debug_img, [pts], True, (0, 255, 0), 2)
        
        # Ponemos texto informativo
        color_texto = (0, 0, 255) if msg_bool.data else (255, 255, 255)
        texto = f"Area Negra: {porcentaje:.1f}%"
        cv2.putText(debug_img, texto, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color_texto, 2)
        
        mensaje_comprimido = self.bridge.cv2_to_compressed_imgmsg(debug_img, dst_format='jpg')
	self.pub_debug.publish(mensaje_comprimido)


def main(args=None):
    rclpy.init(args=args)
    node = ZebraDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
