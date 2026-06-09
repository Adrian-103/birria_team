import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import cv2
import numpy as np

class ZebraDetector(Node):
    def __init__(self):
        super().__init__('zebra_detector_node')
        self.bridge = CvBridge()
        
        # --- PARÁMETROS AJUSTABLES EN VIVO ---
        
        # 1. Umbral para el NEGRO (0-255)
        self.declare_parameter('bin_threshold', 80) 
        
        # 2. Umbrales HSV para el AMARILLO
        self.declare_parameter('yellow_h_min', 20)
        self.declare_parameter('yellow_h_max', 35)
        self.declare_parameter('yellow_s_min', 100)
        self.declare_parameter('yellow_s_max', 255)
        self.declare_parameter('yellow_v_min', 100)
        self.declare_parameter('yellow_v_max', 255)
        
        # 3. Kernel de Dilatación
        self.declare_parameter('kernel_width', 30)
        self.declare_parameter('kernel_height', 5)
        
        # 4. Umbrales de Detección (% de área dentro del trapecio)
        self.declare_parameter('zebra_thresh', 40.0)   # Más alto porque la cebra llena más la cámara
        self.declare_parameter('caution_thresh', 10.0) # Más bajo porque el amarillo es solo la mitad de la cinta
        
        # 5. Geometría del Trapecio
        self.declare_parameter('roi_top_width', 0.4)    
        self.declare_parameter('roi_bottom_width', 0.9) 
        self.declare_parameter('roi_height_offset', 0.6)

        # Cargar valores iniciales
        self.update_params()
        
        # Activar el callback para cambios en vivo desde rqt
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Suscriptores y Publicadores
        self.sub_img = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        # Tópicos separados
        self.pub_zebra = self.create_publisher(Bool, '/interseccion_detectada', 10)
        self.pub_caution = self.create_publisher(Bool, '/linea_precaucion_detectada', 10)
        self.pub_debug = self.create_publisher(CompressedImage, '/vision/zebra_debug/compressed', 10)
        
        self.get_logger().info("¡Detector Dual (Cebra / Precaución) Iniciado!")

    def update_params(self):
        self.thresh_val = self.get_parameter('bin_threshold').value
        self.y_h_min = self.get_parameter('yellow_h_min').value
        self.y_h_max = self.get_parameter('yellow_h_max').value
        self.y_s_min = self.get_parameter('yellow_s_min').value
        self.y_s_max = self.get_parameter('yellow_s_max').value
        self.y_v_min = self.get_parameter('yellow_v_min').value
        self.y_v_max = self.get_parameter('yellow_v_max').value
        
        self.k_w = self.get_parameter('kernel_width').value
        self.k_h = self.get_parameter('kernel_height').value
        self.z_th = self.get_parameter('zebra_thresh').value
        self.c_th = self.get_parameter('caution_thresh').value
        self.rtw = self.get_parameter('roi_top_width').value
        self.rbw = self.get_parameter('roi_bottom_width').value
        self.rho = self.get_parameter('roi_height_offset').value

    def parameter_callback(self, params):
        for param in params:
            self.get_logger().info(f'Modificado: {param.name} = {param.value}')
        self.update_params()
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            return

        alto, ancho = frame.shape[:2]

        # 1. Definir los puntos del trapecio
        tl = [int(ancho * (0.5 - self.rtw/2)), int(alto * self.rho)]
        tr = [int(ancho * (0.5 + self.rtw/2)), int(alto * self.rho)]
        br = [int(ancho * (0.5 + self.rbw/2)), alto]
        bl = [int(ancho * (0.5 - self.rbw/2)), alto]
        pts = np.array([tl, tr, br, bl], np.int32).reshape((-1, 1, 2))

        # 2. Búsqueda del NEGRO
        gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask_black = cv2.threshold(gris, self.thresh_val, 255, cv2.THRESH_BINARY_INV)

        # 3. Búsqueda del AMARILLO
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([self.y_h_min, self.y_s_min, self.y_v_min])
        upper_yellow = np.array([self.y_h_max, self.y_s_max, self.y_v_max])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 4. Dilatación (para unir las franjas)
        kernel = np.ones((self.k_h, self.k_w), np.uint8)
        dilatada_black = cv2.dilate(mask_black, kernel, iterations=2)
        dilatada_yellow = cv2.dilate(mask_yellow, kernel, iterations=2)

        # 5. Aplicar máscara trapezoidal
        mask_roi = np.zeros_like(gris)
        cv2.fillPoly(mask_roi, [pts], 255)
        
        roi_black = cv2.bitwise_and(dilatada_black, mask_roi)
        roi_yellow = cv2.bitwise_and(dilatada_yellow, mask_roi)

        # 6. Calcular porcentajes independientes
        area_trapecio = cv2.countNonZero(mask_roi)
        pct_black = 0.0
        pct_yellow = 0.0
        
        if area_trapecio > 0:
            pct_black = (cv2.countNonZero(roi_black) / area_trapecio) * 100.0
            pct_yellow = (cv2.countNonZero(roi_yellow) / area_trapecio) * 100.0

        # 7. LÓGICA DE DECISIÓN ESTRICTA
        # Si hay suficiente amarillo, es la línea de precaución.
        is_caution = pct_yellow > self.c_th
        
        # Solo puede ser cebra si hay mucho negro Y NO hay amarillo.
        is_zebra = (pct_black > self.z_th) and not is_caution

        # 8. Publicar a los tópicos
        self.pub_zebra.publish(Bool(data=bool(is_zebra)))
        self.pub_caution.publish(Bool(data=bool(is_caution)))

        # 9. Imagen de Debug Mejorada
        # Creamos una imagen oscura y pintamos de blanco lo que detectó negro, y amarillo lo que detectó amarillo
        debug_img = np.zeros_like(frame)
        debug_img[roi_black > 0] = [255, 255, 255] # Blanco para el negro detectado
        debug_img[roi_yellow > 0] = [0, 255, 255]  # Amarillo para el amarillo detectado
        
        # Dibujar trapecio
        cv2.polylines(debug_img, [pts], True, (0, 255, 0), 2)
        
        # Textos informativos
        color_z = (0, 255, 0) if is_zebra else (100, 100, 100)
        color_c = (0, 255, 255) if is_caution else (100, 100, 100)
        
        cv2.putText(debug_img, f"Negro: {pct_black:.1f}%", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_z, 2)
        cv2.putText(debug_img, f"Amarillo: {pct_yellow:.1f}%", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color_c, 2)
        
        estado = "NINGUNA"
        if is_caution: estado = "PRECAUCION"
        elif is_zebra: estado = "CEBRA"
        cv2.putText(debug_img, f"-> {estado}", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
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
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()