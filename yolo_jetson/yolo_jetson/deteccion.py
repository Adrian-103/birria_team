# ------------------------------------------------------------------------------
#  Antes de correr:
#   1. Revisar que el best sea el correcto y el path donde este
# ------------------------------------------------------------------------------
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from collections import Counter

import cv2 as cv
import numpy as np
from ultralytics import YOLO

# ----------------------------------------
#  Claves Señales y Semáforo - Tópico
# ----------------------------------------
CLAVES_SENALES = {
    "Give_way": "gW",
    "Go_ahead": "straight",
    "Left_turn": "turnL",
    "Right_turn": "turnR",
    "Road_work_ahead": "rW",
    "Roundabout": "rA",
    "Stop": "stop",
}

CLAVES_SEMAFORO = {
    "green": "verde",
    "yellow": "amarillo",
    "red": "rojo"
}

# ---------------------------
#  Paleta colores YOLO
# ---------------------------
PALETA = {
    "Give_way":        (52,  152, 219),
    "Go_ahead":        (46,  204, 113),
    "Left_turn":       (231, 76,  60 ),
    "Right_turn":      (241, 196, 15 ),
    "Road_work_ahead": (155, 89,  182),
    "Roundabout":      (26,  188, 156),
    "Stop":            (230, 126, 34 ),
}
COLOR_DEFECTO = (200, 200, 200)

# ----------------------------------------
# HSV Color Range Setup for Color Masks
# ----------------------------------------
COLOR_RANG = {
    "red": [
        (np.array([0, 140, 200]), np.array([8, 255, 255])),
        (np.array([170, 130, 200]), np.array([179, 255, 255]))
    ],
    "yellow": [
        (np.array([15, 120, 190]), np.array([35, 255, 255]))
    ],
    "green": [
        (np.array([38, 90, 100]), np.array([88, 255, 255]))
    ]
}

def col_mask(hsv_image, color):
    mask = None
    for lower, upper in COLOR_RANG[color]:
        cur_mask = cv.inRange(hsv_image, lower, upper)
        mask = cur_mask if mask is None else cv.bitwise_or(mask, cur_mask)
    return mask

def clean_mask(mask):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv.erode(mask, kernel, iterations=1)
    mask = cv.dilate(mask, kernel, iterations=2)
    return mask
    
def blob_exist(mask, min_area=6, max_area=1500):
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv.contourArea(contour)
        if min_area < area < max_area:
            return True
    return False

# ---------------------------
#  Inicia Nodo
# ---------------------------
class TrafficNode(Node):
    def __init__(self):
        super().__init__('traffic_node_unified')

        # ---------------------------
        #  Parámetros configurables
        # ---------------------------
        self.declare_parameter('confianza', 0.50)
        self.declare_parameter('ancho', 160)
        self.declare_parameter('alto', 140)
        self.declare_parameter('fps', 5)
        self.declare_parameter('frames_c', 6)

        # ---- Debug (igual que line_follower_cv) ----
        # debug:                 publica la imagen anotada en el tópico (on/off)
        # debug_window:          ademas muestra una ventana local (solo con pantalla)
        # debug_compressed:      JPEG por la red (recomendado por el hotspot)
        # debug_resize_width:    ancho de la imagen publicada (0 = tamaño de inferencia)
        # debug_publish_every_n: publica 1 de cada N frames (throttle)
        self.declare_parameter('debug', True)
        self.declare_parameter('debug_window', False)
        self.declare_parameter('debug_compressed', False)
        self.declare_parameter('debug_resize_width', 240)
        self.declare_parameter('debug_publish_every_n', 1)

        self.confianza = self.get_parameter('confianza').value
        self.ancho     = self.get_parameter('ancho').value
        self.alto      = self.get_parameter('alto').value
        self.fps       = self.get_parameter('fps').value
        self.frames_c  = self.get_parameter('frames_c').value

        # ---------------------------
        #  Parámetros ROI (fracciones de ancho/alto de la imagen de inferencia)
        # ---------------------------
        self.declare_parameter('roi_xmin', 0.50)
        self.declare_parameter('roi_xmax', 1.00)
        self.declare_parameter('roi_ymin', 0.00)
        self.declare_parameter('roi_ymax', 1.00)

        self.roi_xmin = self.get_parameter('roi_xmin').value
        self.roi_xmax = self.get_parameter('roi_xmax').value
        self.roi_ymin = self.get_parameter('roi_ymin').value
        self.roi_ymax = self.get_parameter('roi_ymax').value

        try:
            RUTA_MODELO = '/home/puzzlebot/birria_ws/src/birria_team/yolo_jetson/yolo_jetson/best_a.pt'
            self.model = YOLO(RUTA_MODELO)
            self.nombres = self.model.names
        except Exception as e:
            self.get_logger().error(f'Error cargando YOLO: {e}')
            self.model = None

        # ---------------------------
        #  Variables Internas
        # ---------------------------
        self.bridge       = CvBridge()
        self.last_frame   = None
        self.last_header  = None
        self.last_sign    = None   
        self.last_color_s = None   
        self.his          = []     # Historial para conteo de frames
        self._frame_count = 0      # contador para el throttle del debug

        # Ventana local solo si se pide explícitamente (la Jetson suele ir sin pantalla)
        if self.get_parameter('debug_window').value:
            cv.namedWindow('Deteccion General', cv.WINDOW_NORMAL)
            cv.resizeWindow('Deteccion General', 640, 480)

        # --------------------------------------
        #  Subscripciones y Publicadores
        # --------------------------------------
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.sign_pub  = self.create_publisher(String, '/detected_sign', 10)
        self.color_pub = self.create_publisher(String, '/semaforo', 10)

        # El tipo de publicador del debug se fija aquí (comprimido o crudo).
        # debug_compressed NO es reconfigurable en vivo porque cambia el tipo de tópico.
        self._debug_compressed = self.get_parameter('debug_compressed').value
        if self._debug_compressed:
            self.img_pub = self.create_publisher(CompressedImage, '/inference_result/compressed', 10)
        else:
            self.img_pub = self.create_publisher(Image, '/inference_result', 10)

        # ---------------------------
        #  Timer 
        # ---------------------------
        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    # ---------------------------
    #  Región de interés 
    # ---------------------------
    def _en_roi(self, box):
        x1, y1, x2, y2 = box
        cx = (x1 + x2) / 2                             
        cy = (y1 + y2) / 2
        x_min = self.ancho * self.roi_xmin             
        x_max = self.ancho * self.roi_xmax
        y_min = self.alto * self.roi_ymin   
        y_max = self.alto * self.roi_ymax
        return x_min < cx < x_max and y_min < cy < y_max

    # ---------------------------
    #  Cámara - Timer callback 
    # ---------------------------
    def camera_callback(self, msg):
        try:
            self.last_frame  = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_header = msg.header
        except Exception as e:
            self.get_logger().error(f'Error capturando cámara: {e}')

    def timer_callback(self):
        if self.last_frame is None or self.model is None:
            return

        # Refrescar ROI cada ciclo para permitir ajuste en vivo
        self.roi_xmin = self.get_parameter('roi_xmin').value
        self.roi_xmax = self.get_parameter('roi_xmax').value
        self.roi_ymin = self.get_parameter('roi_ymin').value
        self.roi_ymax = self.get_parameter('roi_ymax').value

        frame_s = cv.resize(self.last_frame, (self.ancho, self.alto), interpolation=cv.INTER_LINEAR)
        
        detecciones_validas = []
        todas_las_cajas_yolo = [] 

        #  YOLO revisa si hay señales
        raw_results = self.model.predict(source=frame_s, imgsz=320, conf=self.confianza, verbose=False)[0]

        for box, cls, conf in zip(raw_results.boxes.xyxy.cpu().numpy(),
                                  raw_results.boxes.cls.cpu().numpy(),
                                  raw_results.boxes.conf.cpu().numpy()):
            nombre_clase = self.nombres[int(cls)]
            
            if nombre_clase != "Semaforo":
                todas_las_cajas_yolo.append(box)
                if self._en_roi(box):
                    detecciones_validas.append((box, nombre_clase, float(conf)))

        # Procesamiento semáforo, HSV solo para ROI
        hsv_image = cv.cvtColor(frame_s, cv.COLOR_BGR2HSV)

        # Crear una máscara negra del tamaño de la imagen pequeña
        mascara_roi_permitido = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
        
        # Calcular coordenadas en píxeles - ROI
        x_min_p = int(self.ancho * self.roi_xmin)
        x_max_p = int(self.ancho * self.roi_xmax)
        y_min_p = int(self.alto * self.roi_ymin)
        y_max_p = int(self.alto * self.roi_ymax)
        
        # Dibujar un rectángulo blanco en la máscara: solo esta zona dejará pasar color
        cv.rectangle(mascara_roi_permitido, (x_min_p, y_min_p), (x_max_p, y_max_p), 255, -1)

        # Máscaras de color HSV 
        red_mask    = clean_mask(col_mask(hsv_image, "red"))
        yellow_mask = clean_mask(col_mask(hsv_image, "yellow"))
        green_mask  = clean_mask(col_mask(hsv_image, "green"))

        # Aplicar el filtro geométrico del ROI
        red_mask    = cv.bitwise_and(red_mask, mascara_roi_permitido)
        yellow_mask = cv.bitwise_and(yellow_mask, mascara_roi_permitido)
        green_mask  = cv.bitwise_and(green_mask, mascara_roi_permitido)

        # Borra las señales detectadas por YOLO para evitar interferencias
        for box in todas_las_cajas_yolo:
            bx1, by1, bx2, by2 = map(int, box)
            bx1, by1 = max(0, bx1 - 2), max(0, by1 - 2)
            bx2, by2 = min(self.ancho - 1, bx2 + 2), min(self.alto - 1, by2 + 2)
            
            cv.rectangle(red_mask, (bx1, by1), (bx2, by2), 0, -1)
            cv.rectangle(yellow_mask, (bx1, by1), (bx2, by2), 0, -1)
            cv.rectangle(green_mask, (bx1, by1), (bx2, by2), 0, -1)

        # Revisa si está encendido el semáforo solo en el ROI
        red_detected    = blob_exist(red_mask)
        yellow_detected = blob_exist(yellow_mask)
        green_detected  = blob_exist(green_mask)

        if red_detected:
            self._pub_color_semaforo("red")
        elif yellow_detected:
            self._pub_color_semaforo("yellow")
        elif green_detected:
            self._pub_color_semaforo("green")
        else:
            msg_sem = String()
            msg_sem.data = "sin_sem"
            self.color_pub.publish(msg_sem)
            self.last_color_s = None

    # --------------------------------------
    #  Historial y filtrado de señales
    # --------------------------------------
        if len(detecciones_validas) > 0:
            mejor_senal = max(detecciones_validas, key=lambda x: x[2])
            self.his.append(mejor_senal[1])
        else:
            self.his.append(None)
        
        if len(self.his) > self.frames_c:
            self.his.pop(0)

        votos_validos = [h for h in self.his if h is not None]

        if len(votos_validos) >= 5:
            conteo = Counter(votos_validos)
            ganador, num_votos = conteo.most_common(1)[0]

            if num_votos >= 4:
                clave_sign = CLAVES_SENALES.get(ganador, ganador)
                
                # Publicar solo si es una señal válida dentro del ROI
                msg_sign = String()
                msg_sign.data = clave_sign
                self.sign_pub.publish(msg_sign)

                if ganador != self.last_sign:
                    self.get_logger().info(f'SEÑAL - {ganador} - Tópico: "{clave_sign}" ({num_votos} frames)')
                    self.last_sign = ganador
            else:
                self._publicar_sin_senal()
        else:
            self._publicar_sin_senal()

        # ------------- 
        #  Debug (opcional, para ahorrar recursos)
        # ------------- 
        self._publicar_debug(
            frame_s, detecciones_validas,
            (x_min_p, y_min_p, x_max_p, y_max_p),
            red_detected, yellow_detected, green_detected,
        )

    # --------------------------------------
    #  Construcción y publicación del debug
    # --------------------------------------
    def _publicar_debug(self, frame_s, detecciones, roi_px, red, yellow, green):
        publicar = self.get_parameter('debug').value
        ventana  = self.get_parameter('debug_window').value

        # Si nadie quiere ver nada, no gastamos CPU ni ancho de banda
        if not publicar and not ventana:
            return

        # Throttle: publicar 1 de cada N frames
        every = max(1, int(self.get_parameter('debug_publish_every_n').value))
        self._frame_count += 1
        if self._frame_count % every != 0:
            return

        # Dibujar cajas y overlays sobre una copia
        frame_dib = self._dibujar(frame_s.copy(), detecciones)

        x_min_p, y_min_p, x_max_p, y_max_p = roi_px
        cv.rectangle(frame_dib, (x_min_p, y_min_p), (x_max_p, y_max_p), (255, 0, 255), 1, cv.LINE_AA)

        # Indicador del semáforo (arriba a la derecha)
        st_y, st_x = 2, self.ancho - 22
        color_ind = (100, 100, 100)   # gris = sin semáforo en ROI
        if red:
            color_ind = (0, 0, 255)
        elif yellow:
            color_ind = (0, 242, 255)
        elif green:
            color_ind = (0, 255, 0)
        cv.circle(frame_dib, (st_x + 10, st_y + 10), 6, color_ind, -1)
        cv.circle(frame_dib, (st_x + 10, st_y + 10), 6, (255, 255, 255), 1)

        # Redimensionar (para visualización / ancho de banda)
        width = int(self.get_parameter('debug_resize_width').value)
        if width > 0 and frame_dib.shape[1] != width:
            scale = width / frame_dib.shape[1]
            frame_dib = cv.resize(frame_dib, (width, int(frame_dib.shape[0] * scale)),
                                  interpolation=cv.INTER_LINEAR)

        # Publicar por tópico
        if publicar:
            if self._debug_compressed:
                ok, buf = cv.imencode('.jpg', frame_dib, [cv.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    out = CompressedImage()
                    if self.last_header is not None:
                        out.header = self.last_header
                    out.format = 'jpeg'
                    out.data = buf.tobytes()
                    self.img_pub.publish(out)
            else:
                out = self.bridge.cv2_to_imgmsg(frame_dib, encoding='bgr8')
                if self.last_header is not None:
                    out.header = self.last_header
                self.img_pub.publish(out)

        # Ventana local (solo si hay pantalla)
        if ventana:
            cv.imshow('Deteccion General', frame_dib)
            cv.waitKey(1)

    def _pub_color_semaforo(self, color_key):
        # Publica el estado del semáforo SOLO si cambió de color y está en el ROI
        if color_key != self.last_color_s:
            clave = CLAVES_SEMAFORO.get(color_key, color_key)
            msg = String()
            msg.data = clave
            self.color_pub.publish(msg)
            self.last_color_s = color_key
            self.get_logger().info(f'SEMÁFORO - Estado actual: {color_key.upper()}')

    def _publicar_sin_senal(self):
        # Envía explícitamente "sin_señal" cuando no hay nada válido en el ROI
        msg_sign = String()
        msg_sign.data = "sin_señal"
        self.sign_pub.publish(msg_sign)
        self.last_sign = None

    def _dibujar(self, frame, detecciones):
        for box, nombre, conf in detecciones:
            x1, y1, x2, y2 = map(int, box)
            color = PALETA.get(nombre, COLOR_DEFECTO)
            label = f"{nombre} {conf:.0%}"

            cv.rectangle(frame, (x1, y1), (x2, y2), color, 1)
            (tw, th), _ = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.4, 1)
            cv.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv.putText(frame, label, (x1 + 2, y1 - 3), cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv.LINE_AA)
        return frame

    def destroy_node(self):
        cv.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrafficNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
