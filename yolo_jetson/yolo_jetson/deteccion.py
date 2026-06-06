# -------------------------
#  Antes de correr:
#    1. Copiar best_G.pt y best_V.pt al directorio del paquete
#    2. pip install ultralytics opencv-python --break-system-packages
# -------------------------

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from collections import Counter

import cv2 as cv
import numpy as np
from ultralytics import YOLO

# ---------------------------
#  Clases del modelo de vueltas
# ---------------------------
Clases_v = {'Left_turn', 'Right_turn'}

# ---------------------------
#  Claves - Tópico
# ---------------------------
CLAVES = {
    "Give_way": "gW",
    "Go_ahead": "straight",
    "Left_turn": "turnL",
    "Right_turn": "turnR",
    "Road_work_ahead": "rW",
    "Roundabout": "rA",
    "Stop": "stop",
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
color_d = (200, 200, 200)

# ---------------------------
#  Inicia Nodo
# ---------------------------
class TrafficSignNode(Node):
    def __init__(self):
        super().__init__('traffic_sign_node')

        # ---------------------------
        #  Parámetros configurables
        # ---------------------------
        self.declare_parameter('debug', False)
        self.declare_parameter('confianza', 0.50)
        self.declare_parameter('confianza_v', 0.70)  # confianza solamente para left & right
        self.declare_parameter('ancho', 160)
        self.declare_parameter('alto', 140)
        self.declare_parameter('fps', 3)
        self.declare_parameter('frames_c', 6)

        self.debug       = self.get_parameter('debug').value
        self.confianza   = self.get_parameter('confianza').value
        self.confianza_v = self.get_parameter('confianza_v').value
        self.ancho       = self.get_parameter('ancho').value
        self.alto        = self.get_parameter('alto').value
        self.fps         = self.get_parameter('fps').value
        self.frames_c = self.get_parameter('frames_c').value

        # ---------------------------
        #  Modelo general
        # ---------------------------
        try:
            self.model_g   = YOLO('/home/jess/ros2_ws/src/yolo_cf/yolo_cf/best_G.pt')
            self.nombres_g = self.model_g.names
            self.get_logger().info(
                f'Modelo general cargado, clases: {list(self.nombres_g.values())}'
            )
        except Exception as e:
            self.get_logger().error(f'Error cargando modelo general: {e}')
            self.model_g = None

        # ---------------------------
        #  Modelo vueltas
        # ---------------------------
        try:
            self.model_v = YOLO('/home/jess/ros2_ws/src/yolo_cf/yolo_cf/best_V.pt')
            nombres_raw  = self.model_v.names
            if nombres_raw.get(0) not in ['Left_turn', 'Right_turn']:
                self.nombres_v = {0: 'Left_turn', 1: 'Right_turn'}
            else:
                self.nombres_v = nombres_raw
            self.get_logger().info(
                f'Modelo vueltas cargado, clases: {list(self.nombres_v.values())}'
            )
        except Exception as e:
            self.get_logger().error(f'Error cargando modelo vueltas: {e}')
            self.model_v = None

        #  Variables internas ---------------------------
        self.bridge     = CvBridge()
        self.last_sign  = None
        self.last_frame = None

        # Historial para conteo de frames (left & right)
        self.his = []

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
        self.sign_pub = self.create_publisher(String, '/detected_sign',    10)
        self.img_pub  = self.create_publisher(Image,  '/inference_result', 10)

        # ---------------------------
        #  Timer 
        # ---------------------------
        T = 1.0 / self.fps
        self.timer = self.create_timer(T, self.timer_callback)

        self.get_logger().info(
            f'Nodo listo {self.ancho}x{self.alto} @ {self.fps} FPS, '
            f'conte de {self.frames_c} frames'
        )


    def camera_callback(self, msg):
        try:
            self.last_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Error convirtiendo imagen: {e}')


    def _hay_overlap(self, box1, box2, umbral=0.3):
    # Por si un bounding box se solapa con otro
        x1 = max(box1[0], box2[0])
        y1 = max(box1[1], box2[1])
        x2 = min(box1[2], box2[2])
        y2 = min(box1[3], box2[3])

        if x2 < x1 or y2 < y1:
            return False

        # Área de intersección 
        interseccion = (x2 - x1) * (y2 - y1)

        # Área de cada box
        area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
        area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])

        # Overlap = intersección / área más pequeña
        overlap = interseccion / min(area1, area2)
        return overlap > umbral

    def _detec_sent_flecha(self, frame, box, nombre_o):
    # Verifica la direccion de la flecha usando el centroide del contorno
        x1, y1, x2, y2 = map(int, box)

        #Evitamos cortes fuera de imagen
        h_img, w_img = frame.shape[:2]
        x1 = max(0, min(x1, w_img - 1))
        x2 = max(0, min(x2, w_img - 1))
        y1 = max(0, min(y1, h_img - 1))
        y2 = max(0, min(y2, h_img - 1))

        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return nombre_o
        
        #Escala para un mejor analisis
        roi = cv.resize(roi, (120, 120), interpolation=cv.INTER_LINEAR)

        #Convertir a gris y suaviza
        gray = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)
        blur = cv.GaussianBlur(gray, (5, 5), 0)

        # Threshold automatico
        _, th = cv.threshold(
            blur, 0, 255, 
            cv.THRESH_BINARY_INV + cv.THRESH_OTSU
        )

        #Limpia ruido
        kernel = np.ones((3, 3), np.uint8)
        th = cv.morphologyEx(th, cv.MORPH_OPEN, kernel)
        th = cv.morphologyEx(th, cv.MORPH_CLOSE, kernel)

        #Busca contornos
        conts, _ = cv.findContours(
            th, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE
        )

        if not conts:
            return nombre_o

        #Toma el contorno mas grande (flecha)
        c = max(conts, key = cv.contourArea)

        h_roi = th.shape[0]

        # Para mayor precision usaremos la parte superior de las flechas
        puntos_s = c[c[:, 0, 1] < h_roi * 0.5]
        
        if len(puntos_s) > 5:
            M = cv.moments(puntos_s)
        else:
            M = cv.moments(c)

        # Calcula centroide
        if M["m00"] == 0:
            return nombre_o
        
        cx = int(M["m10"] / M["m00"])
        w = th.shape[1]

        if cx < w * 0.48:
            resu = "Left_turn"
        elif cx > w * 0.55:
            resu = "Right_turn"
        else:
            resu = nombre_o

        if resu != nombre_o:
            self.get_logger().info(
                f'Flecha corregida: {nombre_o} a {resu}'
                f'Centroide x = {cx}/{w}'
            )
        return resu

    def timer_callback(self):
        if self.last_frame is None:
            self.get_logger().info(
                'Esperando frame', throttle_duration_sec=3.0
            )
            return

        if self.model_g is None and self.model_v is None:
            return

        # ---------------------------
        #  Reducir resolución
        # ---------------------------
        frame_s = cv.resize(
            self.last_frame,
            (self.ancho, self.alto),
            interpolation=cv.INTER_LINEAR
        )

        detec_g = []   
        detec_v = []   

        # ---------------------------
        #  Inferencia modelo general
        # ---------------------------
        if self.model_g is not None:
            res_g = self.model_g.predict(
                source  = frame_s,
                imgsz   = 160,
                conf    = self.confianza,
                verbose = False,
            )[0]

            for box, cls, conf in zip(
                res_g.boxes.xyxy.cpu().numpy(),
                res_g.boxes.cls.cpu().numpy(),
                res_g.boxes.conf.cpu().numpy()
            ):
                nombre = self.nombres_g[int(cls)]
                if nombre not in Clases_v:
                    detec_g.append((box, nombre, float(conf)))

        # ---------------------------
        #  Inferencia modelo vueltas
        # ---------------------------
        if self.model_v is not None:
            res_v = self.model_v.predict(
                source  = frame_s,
                imgsz   = 160,
                conf    = self.confianza_v,
                verbose = False,
            )[0]

            for box, cls, conf in zip(
                res_v.boxes.xyxy.cpu().numpy(),
                res_v.boxes.cls.cpu().numpy(),
                res_v.boxes.conf.cpu().numpy()
            ):
                nombre = self.nombres_v[int(cls)]
                
                if nombre in Clases_v:
                    nombre = self._detec_sent_flecha(frame_s, box, nombre)
                    detec_v.append((box, nombre, float(conf)))

        # ---------------------------
        #  Filtro de overlap
        # ---------------------------
        detec_v_final = []
        for box_v, nombre_v, conf_v in detec_v:
            overlap = False
            for box_g, nombre_g, conf_g in detec_g:
                # Si hay solapamiento Y el modelo general está más seguro
                if self._hay_overlap(box_v, box_g) and conf_g > conf_v:
                    overlap = True
                    self.get_logger().info(
                        f'Ignorando {nombre_v} ({conf_v:.0%}) — '
                        f'overlap con {nombre_g} ({conf_g:.0%})'
                    )
                    break
            if not overlap:
                detec_v_final.append((box_v, nombre_v, conf_v))

        # Combinar detecciones finales
        detec = detec_g + detec_v_final

        # ---------------------------
        #  Verificar por Conteo de 
        #  Frames
        # ---------------------------
        if len(detec) > 0:
            # Detectar la mejor deteccion al historial
            m_d = max(detec, key=lambda x: x[2])
            self.his.append(m_d[1])

        else:
            # Si no detecta con el suficiente conteo, no se agrega al historial 
            self.his.append(None)
        
        # Mantener solo los ultimos frames (cantidad que nosotros asignamos)
        if len(self.his) > self.frames_c:
            self.his.pop(0)

        # Conteo de votos menos None
        vot = [h for h in self.his if h is not None]

        # ---------------------------
        #  Publicar señal detectada
        # ---------------------------

        if len(vot) >= 5:
            cont = Counter(vot)
            win = cont.most_common(1)[0]
            name_w = win[0]
            vot_w = win[1]

        # Se publica el valor (señal) que haya sido derectado por mas de 4 veces
            if vot_w >= 4:
                clave = CLAVES.get(name_w, name_w)

                msg_sign = String()
                msg_sign.data = clave
                self.sign_pub.publish(msg_sign)

                if name_w != self.last_sign:
                    org = "VUELTA" if name_w in Clases_v else "GENERAL"
                    self.get_logger().info(
                        f'[{org}] Sign: {name_w} '
                        f'tópico: {clave}'
                        f'({vot_w}/{self.frames_c} frames)'
                    )
                    self.last_sign = name_w
                    
            else:
                if self.last_sign is None:
                    self.get_logger().info('Esperando mas frames')
                    self.last_sign = None
                msg_sign = String()
                msg_sign.data = "sin_señal"
                self.sign_pub.publish(msg_sign)

        else:
            if self.last_sign is not None:
                self.get_logger().info('Sin señal')
                self.last_sign = None
            msg_sign = String()
            msg_sign.data = "sin_señal"
            self.sign_pub.publish(msg_sign)

        # ---------------------------
        #  Bounding boxes
        # ---------------------------
        frame_a = self._dibujar(frame_s.copy(), detec)

        img_msg = self.bridge.cv2_to_imgmsg(frame_a, encoding='bgr8')
        self.img_pub.publish(img_msg)

        # ---------------------------
        #  Debug
        # ---------------------------
        if self.debug:
            frame_d = cv.resize(frame_a, (640, 480), interpolation=cv.INTER_LINEAR)
            cv.imshow('Deteccion Señales', frame_d)
            cv.waitKey(1)


    def _dibujar(self, frame, detec):
        for box, nombre, conf in detec:
            x1, y1, x2, y2 = map(int, box)
            color = PALETA.get(nombre, color_d)
            label = f"{nombre} {conf:.0%}"

            cv.rectangle(frame, (x1, y1), (x2, y2), color, 1)

            (tw, th), _ = cv.getTextSize(
                label, cv.FONT_HERSHEY_SIMPLEX, 0.4, 1
            )
            cv.rectangle(
                frame,
                (x1, y1 - th - 6), (x1 + tw + 4, y1),
                color, -1
            )
            cv.putText(
                frame, label, (x1 + 2, y1 - 3),
                cv.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255), 1, cv.LINE_AA
            )

            tag = "V" if nombre in Clases_v else "G"
            cv.putText(
                frame, tag, (x2 - 12, y1 + 12),
                cv.FONT_HERSHEY_SIMPLEX, 0.35,
                (255, 255, 255), 1, cv.LINE_AA
            )
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
