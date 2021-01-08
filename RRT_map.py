import cv2 as cv
import numpy as np
import random
import math



# Vector de colores para trabajar con la salida por pantalla
# en opencv
WHITE=(255,255,255)
BLUE=(0,0,255)
BLACK=(0,0,0)
RED=(255,0,0)
GREEN=(0,255,0) 

# Constantes para controlar la dimension del mapa, el
# numero y dimesion de los obtaculos y la distancia maxima
# entre nodos
DIM = 500
NUM_OBSTACLES = 70
OBSTACLES_DIM = 15



# Para introducir manualmente los obstáculos que quieras
OBSTACLES = {"Circle": [(300, 3), 30, BLACK, -1]} 


# La clase mapa sera la encargada de crear el objeto mapa. Este objeto sera necesario para dibujar en pantalla 
# y para tener informacion de los obstaculos y de los nodos start y goal
class Map:
    def __init__(self, start, goal):

        # Definimos los puntos de salida y objetivo de robot, así como las dimensiones del terreno.
        # Suponemos que disponemos de estos datos así como conocimiento del mapa completo
        self.start = start
        self.goal = goal
        self.mapDimensions = DIM
        self.mapH, self.mapW = DIM, DIM

        # Estos valores son solo elementos para dibujar. Controlamos el radio de los nodos start y goal para que
        # sean visibles. El valor -1 es para que los objetos dibujados esten rellenos.
        self.nodeRadius = 6
        self.nodeThickness = -1

        # Vector que almacena la posicion del centro de cada obstaculo. Como van a ser circulos, el radio se pasa 
        # como una constante
        self.obstacles = []

        # Aqui creamos lo que sera nuestra imagen que dara la salida por pantalla. Al comienzo queremos que 
        # todo sea una pantalla blanca. Del color de cada pixel tambien podemos extraer informacion del mapa
        self.mapImage = np.ones([self.mapH, self.mapW, 3], np.uint8)*255  #Blanco al inicio
        self.mapWindow = "Path planning algorithms"

    # Esta funcion inicializa el mapa e introduce los obstaculos y los puntos start y goal
    def drawMap(self):
        self.drawNode([self.start[0], self.start[1]], "S")
        self.drawNode([self.goal[0], self.goal[1]],"G")
        #self.drawObstacles()
        self.drawRandomObstacles()

        # Muestra el mapa por pantalla
        cv.imshow(self.mapWindow, self.mapImage)
        # Necesario para el efecto de animacion
        cv.waitKey(1)
    
    # Funcion que nos ayuda a ver por pantalla el efecto de animacion
    def refreshImage(self):
        cv.imshow(self.mapWindow, self.mapImage)
        cv.waitKey(1)


    # Funcion que dibuja cada nodo dentro del mapa en funcion de su tipo
    def drawNode(self, coordinate, nodeType):
        
        # Tenemos tres tinode de puntos:  nodos "N", punto de partida "S", objetivo "G".
        # print(nodeType)

        # Todas son funciones que vienen implementadas en la libreria cv2
        if nodeType == "N" :
            cv.circle(self.mapImage, (coordinate[0], coordinate[1]), 3, RED, self.nodeThickness)
        if nodeType == "S" : 
            cv.circle(self.mapImage, (self.start[0], self.start[1]), self.nodeRadius, BLUE, self.nodeThickness)
        if nodeType == "G" : 
            cv.circle(self.mapImage, (self.goal[0], self.goal[1]), self.nodeRadius, GREEN, self.nodeThickness)
        if nodeType == "Path":
            cv.circle(self.mapImage, (coordinate[0], coordinate[1]), 3, GREEN, self.nodeThickness)


    #Funcion para dibujar los obstaculos que uno desee. Por ejemplo si quisiese generar un mapa en concreto

    #def drawObstacles(self):
     #    for key, value in self.obstacles.items():
     #       if key == "Circle":
     #           cv.circle(self.mapImage, value[0], value[1], value[2] , value[3])
     #       if key == "Rectangle":
     #           cv.rectangle(self.mapImage, value[0], value[1], value[2], value[3])


    # Funcion que genera circulos aletorios de un tamaño dado por constante y que almacena los centros 
    # en el vector de obstaculos para poder usarlos en el grafo.
    def drawRandomObstacles(self):
        # Generamos las coordenadas del centro del circulo.
        for i in range(0, NUM_OBSTACLES-1):
            radius_coords = int(random.uniform(0+OBSTACLES_DIM, DIM-OBSTACLES_DIM)), int(random.uniform(0+OBSTACLES_DIM, DIM-OBSTACLES_DIM))
            cv.circle(self.mapImage, (radius_coords[0], radius_coords[1]), OBSTACLES_DIM, BLACK, -1)
            self.obstacles.append(radius_coords) 
        #print(self.obstacles)
    
    # Dibujamos la linea entre dos nodos
    def drawEdge(self, node1, node2):
        cv.line(self.mapImage, (node1[0],node1[1]), (node2[0],node2[1]),  BLACK, 1)
    
    # Dado un vector con nodos dibujamos cada punto. Esta hecha para dibujar el camino al goal
    def drawGoalPath(self, path):
        for node in path:
            self.drawNode(node, "Path")
            cv.imshow(self.mapWindow, self.mapImage)
            cv.waitKey(1)