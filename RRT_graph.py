import cv2 as cv
import numpy as np
import random
import math
import RRT_map
from RRT_map import DIM, NUM_OBSTACLES, OBSTACLES_DIM

# Constate que controla la distancia maxima de union entre un nodo
# nuevo y uno perteneciente al arbol
MAX_DISTANCE = 15


# Clase que controla todo el algoritmo RRT
class Graph:
    # Constructor del grado
    def __init__(self, _map):
        
        # Pasamos como argumento el mapa donde estan definidos 
        # los obstaculos y el punto start y goal
        self._map = _map

        # Vectores que almacenan las coordenadas de los nodos
        # Añadimos como primer nodo el punto start
        self.x = []
        self.y = []
        self.x.append(self._map.start[0])
        self.y.append(self._map.start[1])

        # Vectores que almacenan el padre de cada nodo
        # (nodo mas cercano)
        # Añadimos el padre de start que es el propio start
        self.parent = []
        self.parent.append(0)

        # Variables que almacenan el punto del grafo mas cercano al goal y 
        # si este ha encontrado el goal
        self.goal_nearest = None
        self.goal_reached = False
        # Vector que almacena las posiciones del camino star-goal
        self.path_to_goal = []

    # Cada nodo tiene su identificador n entero y sus coordeandas x e y.
    def insert_node(self, node, x, y): 
        self.x.insert(node, x)
        self.y.insert(node, y)
    
    # Eliminamos el nodo n del vector
    def delete_node(self, node):
        self.x.pop(node)
        self.y.pop(node)
    
    # Funcion para almacenar el padre de cada nodo
    def insert_link(self, parent, child):
        self.parent.insert(parent, child)
    
    # Eliminamos un padre-hijo del vector
    def delete_link(self, node):
        self.parent.pop(node)

    # Calculamos cuantos nodos tenemos el en grafo
    def graph_lenth(self):
        length = len(self.x)
        return length

    # Calculo de la distancia euclidea entre dos nodos
    def distance_nodes(self, node1, node2):
        x1, y1 = float(self.x[node1]), float(self.y[node1])
        x2, y2 = float(self.x[node2]), float(self.y[node2])

        distance = ((x2-x1)**2+(y2-y1)**2)**0.5
        return distance
    
    # Calculamos el nodo mas cercano a otro calculando la distancia
    # de ese nodo con el resto ya introducidos
    def nearest_node(self, node):
        reference_node = 0
        
        dist_min = self.distance_nodes(reference_node, node)
        nearest = 0

        for i in range(0, node):
            if self.distance_nodes(i, node) < dist_min:
                dist_min = self.distance_nodes(i, node)
                nearest = i

        return nearest
    
    # Creamos de forma uniforme (para que las zonas mas inexploradas tengan preferencia) 
    # un nuevo nodo
    def newRandom_node(self):
        x, y = int(random.uniform(0, self._map.mapW)), int(random.uniform(0, self._map.mapH))
        return x, y
    
    # Metodo para comprobar si un nodo esta dentro de un obstáculo. Para ello tomamos la posicion
    # del nodo y el centro de todos los obtaculos y comprobamos si se encuentra a una distancia superior 
    # a la dimension del obstaculo
    def not_colision_point2(self):
        last_node = self.graph_lenth()-1
        x, y = float(self.x[last_node]), float(self.y[last_node])
        for center in self._map.obstacles:
            dist = ((x-center[0])**2+(y-center[1])**2)**0.5
            if dist <= OBSTACLES_DIM:
                self.delete_node(last_node)
                return False
        
        return True

    # Metodo alternativo para comprobar si un nodo esta dentro de un obstáculo. Para ello tomamos la posicion
    # del nodo y comprobamos el color del pixel. Si es negro estará dentro de un obstáculo
    def not_colision_point3(self):
        last_node = self.graph_lenth()-1
        x, y = self.x[last_node], self.y[last_node]
        color = 0
        for i in self._map.mapImage[x][y]:
            color += i
        if color == 0:
            self.delete_node(last_node)
            return False
        return True


    # Comprobamos que el link entre dos nodos no se cruza con algun obstaculo.                
    def not_colision_path(self, node1, node2):

        # Dos puntos. Una vez tengo su padre, calculo la ecuacion de la recta que los une.
        # divido esa recta en aproximadamente 100 puntos y estudio si alguno de esos puntos toca con algun obstáculo. Si no toca con ninguno,
        # creo el link y dibujo la recta
        x1, y1 = float(self.x[node1]), float(self.y[node1])
        x2, y2 = float(self.x[node2]), float(self.y[node2])

        # Ecuacion de interpolacion. Divide el segmento entre ambos puntos en N puntos equidistantes
        for i in range(0,101):
                p=i/100
                x=x1*p + x2*(1-p)
                y=y1*p + y2*(1-p)
                for center in self._map.obstacles:
                    dist = ((x-center[0])**2+(y-center[1])**2)**0.5
                    if dist <= OBSTACLES_DIM:
                        return False
        return True
    

    # Si el camino entre dos nodos no esta obstaculizado  creo el link entre nodos 
    # y lo almaceno
    def link_nodes(self, node1, node2):
        if self.not_colision_path(node2, node1):
            self.insert_link(node2, node1)

            return True
        else:
            self.delete_node(node2)
            return False

    # Funcion mas importante que recoge todo el algoritmo RRT. Para emepzar con el algoritmo necesito crear
    # nodo aleatorio valido, saber cual es su nodo padre, y tener en cuenta la distancia maxima permitida
    def rrt_algorithm(self, near_node, random_node, max_distance = MAX_DISTANCE):
        #Calculo la distancia entre el nodo nuevo y su padre
        modulus = self.distance_nodes(near_node, random_node)

        # En caso de que sea una distancia mayor a la permitida, obtengo otro punto sobre su linea de union
        # que se encuentre a la distancia maxima permitida
        if modulus > max_distance:
            x1, y1 = self.x[near_node], self.y[near_node]
            x2, y2 = self.x[random_node], self.y[random_node]

            a = y2 - y1
            b = x2 - x1
            
            # Tengo que usar atan2 para que me de correctamente el signo del angulo. Con atan no vale
            angle = math.atan2(a, b)

            x2_new = int(max_distance * math.cos(angle) + x1)
            y2_new = int(max_distance * math.sin(angle) + y1)

            # Un vez calculado las coordenadas del nodo valido, eliminamos las coordenadas del anterior
            # y le almacenamos las nuevas
            self.delete_node(random_node)
            self.insert_node(random_node, x2_new, y2_new)

            # Comprobamos si el nuevo nodo creado se encuentra a una distancia determinada, en este caso 15,
            # del goal
            if abs(x2_new - self._map.goal[0]) < 15 and abs(y2_new - self._map.goal[1]) < 15:
                
                # Cambio el valor del booleano y almaceno ese nodo como el mas cercano al nodo
                self.goal_reached = True
                self.goal_nearest = random_node


    # Calculo el camino  goal- start. Almaceno en un vector el punto que ha alcanzado al goal y obtengo su padre.
    # Despues calculo el padre del padre y asi sucesivamente hasta que llego al start   
    def goal_path(self):      
            if self.goal_reached:
                self.path_to_goal.append(self.goal_nearest)
                parent = self.parent[self.goal_nearest]
            while parent != 0:
                self.path_to_goal.append(parent)
                parent = self.parent[parent]
            self.path_to_goal.append(0)
        
    # Extraigo del vector de nodos del camino goal-start las coordenadas de cada nodo y las almaceno en otro vector
    #  que devuelvo para poder dibujarlo despues
    def path_coordinates(self):
        path_coords = []
        for node in self.path_to_goal:
            x, y = self.x[node], self.y[node]
            path_coords.append([x, y])
        return path_coords
    
    # Funcion que realiza el RRT expandiendo el arbol hacia el goal
    def tree_to_goal(self):
        last_node = self.graph_lenth()
        # Creamos un nodo que va en direccion a las coordenadas del nodo
        # y creamos su union a la distancia maxima si la union es posible
        self.insert_node(last_node, self._map.goal[0], self._map.goal[1])
        near = self.nearest_node(last_node)
        self.rrt_algorithm(near, last_node)
        self.link_nodes(near, last_node)
        # Devolvemos las coordenadas y el padre porque seran necesarias para dibujar sobre la imagen
        return self.x, self.y, self.parent

    # Funcion que expande el arbol en direcciones aleatorias
    def explore_map(self):
        # Creamos el nuevo nodo
        new_node = self.graph_lenth()
        x, y = self.newRandom_node()
        self.insert_node(new_node, x, y)
        # Si el nodo es valido continuamos
        if self.not_colision_point3(): 
            # Calculamos el nodo padre
            near = self.nearest_node(new_node)
            # Realizamos el algoritmo
            self.rrt_algorithm(near, new_node)
            x, y = self.x[new_node], self.y[new_node]
            # Si el link es valido lo creamos
            self.link_nodes(near, new_node)
        # Devolvemos las variables necesarias para dibujar
        return self.x, self.y, self.parent




            