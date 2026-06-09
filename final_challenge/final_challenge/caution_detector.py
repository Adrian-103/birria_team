import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import numpy as np

class CautionDetector(Node):
    def __init__(self):
        super().__init__('caution_detector_node')
        self.bridge = CvBridge()
        
        # --- PARÁMETROS EXCLUSIVOS PARA EL AMARILLO ---
        self.declare_parameter('yellow_h_min', 15)
        self.declare_parameter('yellow_h_max', 35)
        self.declare_parameter('yellow_s_min', 100)
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 100)
        self.declare_parameter('yellow_v_max', 255)
        
        # Morfología
        self.declare_parameter('kernel_width', 25)
        self.declare_parameter('kernel_height', 5)
        
        # Umbral de activación para la cinta (suele ser menor porque el amarillo es solo la mitad de las rayas)
        self.declare_parameter('caution_thresh', 15.0) 
        
        # Geometría del Trapecio (Ajustable independientemente si quieres mirar más cerca/lejos)
        self.declare_parameter('roi_top_width', 0.4)    
        self.declare_parameter('roi_bottom_width', 0.9) 
        self.declare_parameter('roi_height_offset', 0.6)

        self.update_params()
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Suscriptores y Publicadores Independientes
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub_caution = self.create_publisher(Bool, '/linea_precaucion_detectada', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/vision/caution_debug/compressed', 10)
        
        self.get_logger().info("¡Detector de Línea de Precaución (Solo Amarillo) Iniciado!")

    def update_params(self):
        self.y_h_min = self.get_parameter('yellow_h_min').value
        self.y_h_max = self.get_parameter('yellow_h_max').value
        self.y_s_min = self.get_parameter('yellow_s_min').value
        self.y_s_max = self.get_parameter('yellow_s_max').value
        self.y_v_min = self.get_parameter('yellow_v_min').value
        self.y_v_max = self.get_parameter('yellow_v_max').value
        
        self.k_w = self.get_parameter('kernel_width').value
        self.k_h = self.get_parameter('kernel_height').value
        self.c_th = self.get_parameter('caution_thresh').value
        self.rtw = self.get_parameter('roi_top_width').value
        self.rbw = self.get_parameter('roi_bottom_width').value
        self.rho = self.get_parameter('roi_height_offset').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Parámetro precaución modificado: {param.name} = {param.value}')
        self.update_params()
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return

        alto, ancho = frame.shape[:2]

        # 1. Definir Trapecio ROI
        tl = [int(ancho * (0.5 - self.rtw/2)), int(alto * self.rho)]
        tr = [int(ancho * (0.5 + self.rtw/2)), int(alto * self.rho)]
        br = [int(ancho * (0.5 + self.rbw/2)), alto]
        bl = [int(ancho * (0.5 - self.rbw/2)), alto]
        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))

        # 2. Filtrado puramente por Color Amarillo (HSV)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.y_h_min, self.y_s_min, self.y_v_min])
        upper_yellow = np.array([self.y_h_max, self.y_s_max, self.y_v_max])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 3. Dilatación para conectar las rayas amarillas saltándose el negro
        kernel = np.ones((self.k_h, self.k_w), np.uint8)
        dilatada = cv2.dilate(mask_yellow, kernel, iterations=2)

        # 4. Aplicar máscara trapezoidal
        mask_roi = np.zeros_like(dilatada)
        cv2.fillPoly(mask_roi, [pts], 255)
        roi_final = cv2.bitwise_and(dilatada, mask_roi)

        # 5. Calcular porcentaje de área amarilla activa
        area_trapecio = cv2.countNonZero(mask_roi)
        if area_trapecio > 0:
            amarillos = cv2.countNonZero(roi_final)
            porcentaje = (amarillos / area_trapecio) * 100.0
        else:
            porcentaje = 0.0

        # 6. Publicar decisión booleana
        msg_bool = Bool()
        msg_bool.data = bool(porcentaje > self.c_th)
        self.pub_caution.publish(msg_bool)

        # 7. Imagen de Debug (Pinta de color amarillo chillón lo detectado)
        debug_img = np.zeros_like(frame)
        debug_img[roi_final > 0] = [0, 255, 255] # BGR para Amarillo
        
        cv2.polylines(debug_img, [pts], True, (0, 255, 0), 2)
        color_texto = (0, 255, 255) if msg_bool.data else (255, 255, 255)
        cv2.putText(debug_img, f"Area Amarilla: {porcentaje:.1f}%", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color_texto, 2)

        mensaje_comprimido = self.bridge.cv2_to_compressed_imgmsg(debug_img, dst_format='jpg')
        self.pub_debug.publish(mensaje_comprimido)

def main(args=None):
    rclpy.init(args=args)
    node = CautionDetector()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()