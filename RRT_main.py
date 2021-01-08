import cv2 as cv
import random
from RRT_graph import Graph
from RRT_map import Map


def main():

    # Definimos las corodenadas del punto inicial y final
    start = [2, 2]
    goal = [500, 500]

    # Generamos el mapa y el grafo inicial            
    mp = Map(start, goal)

    # Al pasar el objeto mapa ya tenemos almacenado en la info
    # tanto los obstáculos como el punto start y goal
    gph = Graph(mp)   

    # Generamos los obstáculos y colocamos punto start y goal
    mp.drawMap()

    # No tenemos en cuenta la longitud del grafo
    # solo si encontramos el goal o no
    while not gph.goal_reached: 
    
        # Generamos un aleatorio para controlar la probabilidad de expandir o 
        # alcanzar el goal
        probability = random.uniform(0, 100)

        # Expandimos hacia el goal
        if probability > 95:
            coordx, coordy, Parent = gph.tree_to_goal()
            # Dibujamos siempre el ultimo nodo y link añadido al vector que los almacena
            mp.drawNode([coordx[-1],coordy[-1]], "N") 
            mp.drawEdge( (coordx[-1],coordy[-1]) , (coordx[Parent[-1]],coordy[Parent[-1]])  )
            mp.refreshImage()
        # Expandimos hacia las zonas más inexploradas
        else: 
            coordx,coordy,Parent=gph.explore_map()
            mp.drawNode([coordx[-1], coordy[-1]], "N")
            mp.drawEdge((coordx[-1], coordy[-1]), (coordx[Parent[-1]], coordy[Parent[-1]]))
            mp.refreshImage()

    # Buscamos el camino mas corto goal-start dentro del grafo        
    gph.goal_path()

    # Dibujamos los nodos obtenidos para el camino
    mp.drawGoalPath(gph.path_coordinates())

    # Bloqueamos la salida en pantalla para que no se cierre cuando finalicen
    # los procesos
    cv.waitKey(0) 


if __name__ == '__main__':
    main()