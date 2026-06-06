import rclpy
from rclpy.node import Node
import numpy as np

# Importamos los mensajes para crear una Ruta oficial de ROS 2
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class Nodo:
    def __init__(self, posicion, origen = None):
        self.posicion = posicion
        self.origen = origen

        self.g = 0
        self.h = 0
        self.f = 0

    def calc_heuristica(self, meta):
        self.h = abs(self.posicion[1] - meta[1]) + abs(self.posicion[0] - meta[0])

    def calc_f(self):
        self.f = self.g + self.h

# 0 = Libre, 1 = Pared
mapa_pista = np.array([
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,1,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,1],
    [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1],
])

def a_estrella(inicio, meta, mapa):
    lista_visitados = []
    lista_ruta = []
    lista_exploracion = []

    nodo_actual = Nodo(inicio)
    lista_exploracion.append(nodo_actual)

    while nodo_actual.posicion != meta:

        if len(lista_exploracion) == 0:
            return None

        arriba = (nodo_actual.posicion[0] - 1,  nodo_actual.posicion[1])
        izquierda = (nodo_actual.posicion[0],  nodo_actual.posicion[1] - 1)
        abajo = (nodo_actual.posicion[0] + 1,  nodo_actual.posicion[1])
        derecha = (nodo_actual.posicion[0], nodo_actual.posicion[1] + 1)
        #Definimos los nodos a visitar

        direcciones = [arriba, izquierda, abajo, derecha]

        for direccion in direcciones:
            if direccion[0] < 0 or direccion[0] > 23 or direccion[1] < 0 or direccion[1] > 15:
                continue
                #Verificamos que no se salga de los bordes
            
            if mapa [direccion[0]][direccion[1]] == 1:
                continue
                #Verificamos que no sea una pared

            nodo_prospecto = Nodo(direccion, nodo_actual)

            nodo_prospecto.g = nodo_prospecto.origen.g + 1
            nodo_prospecto.calc_heuristica(meta)
            nodo_prospecto.calc_f()

            ## Búsqueda del nodo prospecto en la lista de visitados:
            ya_visitado = any(nodo.posicion == nodo_prospecto.posicion for nodo in lista_visitados)
            ya_explorado = any(nodo.posicion == nodo_prospecto.posicion for nodo in lista_exploracion)

            if not ya_visitado:
                if ya_explorado:
                    # Buscar el nodo existente en lista_exploracion
                    for nodo_existente in lista_exploracion:
                        if nodo_existente.posicion == nodo_prospecto.posicion:
                            if nodo_prospecto.g < nodo_existente.g:
                                nodo_existente.g = nodo_prospecto.g
                                nodo_existente.origen = nodo_prospecto.origen
                                nodo_existente.calc_f()
                            break
                else:
                    lista_exploracion.append(nodo_prospecto)

        nodo_menor = lista_exploracion[0]
        for index in range(len(lista_exploracion)):
            if lista_exploracion[index].f < nodo_menor.f:
                nodo_menor = lista_exploracion[index]
        
        nodo_actual = nodo_menor
        lista_exploracion.remove(nodo_menor)
        lista_visitados.append(nodo_actual)

    while nodo_actual is not None:
        lista_ruta.append(nodo_actual.posicion)
        nodo_actual = nodo_actual.origen

    lista_ruta.reverse()
    return lista_ruta, lista_visitados


class GlobalPlanner(Node):
    def __init__(self):
        super().__init__('global_planner_node')
        
        self.get_logger().info("Calculando ruta óptima con A*...")
        
        inicio = (22, 13)
        meta = (2, 2)
        
        # Ejecutamos el algoritmo
        resultado = a_estrella(inicio, meta, mapa_pista)
        
        if resultado is None or resultado[0] is None:
            self.get_logger().error("¡No se encontró un camino posible!")
            self.ruta_metros = []
        else:
            ruta_casillas, _ = resultado
            self.ruta_metros = []
            
            # Traducimos a metros (El truco de la traslación)
            for (fila, col) in ruta_casillas:
                x_m = (22 - fila) * 0.1
                y_m = (13 - col) * 0.1
                self.ruta_metros.append((x_m, y_m))
                
            self.get_logger().info(f"Ruta calculada con éxito: {len(self.ruta_metros)} puntos.")

        # Creamos el publicador del Path
        self.path_pub = self.create_publisher(Path, '/ruta_planeada', 10)
        
        # Un timer que publique la ruta cada 1 segundo para que RViz o tu controlador la escuchen
        self.timer = self.create_timer(1.0, self.publicar_ruta)

    def publicar_ruta(self):
        if not self.ruta_metros:
            return

        # Construimos el mensaje Path de ROS 2
        msg_ruta = Path()
        msg_ruta.header.frame_id = 'odom' # El marco de referencia
        msg_ruta.header.stamp = self.get_clock().now().to_msg()

        for (x, y) in self.ruta_metros:
            pose = PoseStamped()
            pose.header = msg_ruta.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0 # El robot no vuela
            
            # Agregamos el punto a la lista del mensaje
            msg_ruta.poses.append(pose)

        # ¡Publicamos!
        self.path_pub.publish(msg_ruta)


def main(args=None):
    rclpy.init(args=args)
    nodo = GlobalPlanner()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()