from random import randint
from mesa import Agent, Model
from mesa.space import MultiGrid
from mesa.time import RandomActivation
from mesa.visualization.modules import CanvasGrid, TextElement
from mesa.visualization.ModularVisualization import ModularServer
from pathfinding.core.grid import Grid as Path_grid
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.finder.a_star import AStarFinder

import numpy

#Definición de la clase para el agente robot
class Robot(Agent):
    def __init__(self, model, pos, matrix):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.carry = False
        self.matrix = matrix
        self.path = []
    
    def step(self):
        agentsList = self.model.grid.get_cell_list_contents(self.pos)
        boxes = self.model.count_box(agentsList)
        
        #El robot encuentra una caja sola y la toma
        if(boxes == 1 and self.path == []):
            for agent in agentsList:
                if(type(agent) == Box and agent.active):
                    if(self.carry == False):
                        self.carry = True
                        #Se desactiva la caja
                        agent.active = False
                        #Se asigna un path al robot hacia un stack de cajas
                        self.find_path()
   
        #El robot ya tiene un path predefinido y deposita la caja en el stack
        elif(self.path != []):
            num_box = self.model.count_box(self.model.grid.get_cell_list_contents(self.path[0]))
            self.model.grid.move_agent(self, self.path[0])
            self.path.pop(0)
            if(self.path == [] and num_box < 5):
                self.carry = False
                box = Box(self.model, self.pos)
                self.model.grid.place_agent(box, box.pos)

        #El robot no tiene un path predefinido, pero ya tiene una caja
        elif(self.path == [] and self.carry):
            self.find_path()

        #El robot no tiene una caja y busca una en random walk
        else:
            next_moves = self.model.grid.get_neighborhood(self.pos, moore=False)
            next_move = self.random.choice(next_moves)
            if(self.model.matrix[next_move[1]][next_move[0]] != 3):
                self.model.grid.move_agent(self, next_move)
    
    #Función para encontrar el camino hacia un stack de cajas
    def find_path(self):
        matrix_path = numpy.ones((len(self.matrix), len(self.matrix[0])), dtype=int)
        grid = Path_grid(matrix = matrix_path)
        start = grid.node(self.pos[0], self.pos[1])
        around = self.model.grid.get_neighborhood(self.pos, moore=False, radius=8)
        
        for i in around:
            check = self.model.count_box(self.model.grid.get_cell_list_contents(i))
            if (check > 0 and check < 5):
                end = grid.node(i[0], i[1])
                finder = AStarFinder(diagonal_movement = DiagonalMovement.never)
                self.path, _ = finder.find_path(start, end, grid)
                self.path.pop(0)
                break
            
#Definición de la clase para el agente caja
class Box(Agent):
    def __init__(self, model, pos):
        super().__init__(model.next_id(), model)
        self.pos = pos
        self.active = True

#Definición de la clase para las paredes
class Wall(Agent):
  def __init__(self, model, pos):
    super().__init__(model.next_id(), model)
    self.pos = pos
    
#Función para imprimir texto en pantalla
class display_text(TextElement):
    def __init__(self):
        pass
    def render(self, model):
        return "\nPasos en total: " + str(model.schedule.steps*model.numRobots) + "\n"

#Definición del modelo
class Maze(Model):
    def __init__(self, M = 10, N = 10, numRobots = 5, numBoxes = 30, maxSteps = 300):
        super().__init__()
        self.schedule = RandomActivation(self)
        self.grid = MultiGrid(M, N, torus=False)
        self.maxSteps = maxSteps 
        self.numRobots = numRobots
        self.numBoxes = numBoxes
        self.list_stacks = []

        self.matrix = numpy.zeros((M,N))

        #Posición de los bloques de pared en el grid
        for i in range(M):
            for j in range(N):
                if j == 0 or i == 0 or j == N-1 or i == M-1:
                    self.matrix[i][j] = 3

        #Posición inicial aleatoria de las cajas en el grid
        while(numBoxes > 0):
            row = randint(0, N-1)
            column = randint(0, M-1)
            if(self.matrix[column][row] == 0):
                self.matrix[column][row] = 1
                numBoxes -= 1

        #Posición inicial aleatoria de los robots en el grid
        while(numRobots > 0):
            row = randint(0, N-1)
            column = randint(0, M-1)
            if(self.matrix[column][row] == 0):
                self.matrix[column][row] = 2
                numRobots -= 1

        #Se colocan los agentes en el grid
        for _,x,y in self.grid.coord_iter():
            if self.matrix[y][x] == 1:
                box = Box(self, (x, y))
                self.grid.place_agent(box, box.pos)
                self.schedule.add(box)
            elif self.matrix[y][x] == 2:
                robot = Robot(self, (x, y), self.matrix)
                self.grid.place_agent(robot, robot.pos)
                self.schedule.add(robot)
            elif self.matrix[y][x] == 3:
                wall = Wall(self, (x, y))
                self.grid.place_agent(wall, wall.pos)
                self.schedule.add(wall)

    #Función para contar los agentes de tipo caja en una lista
    @staticmethod
    def count_box(list):
        count = 0
        for agent in list:
            if type(agent) == Box and agent.active:
                count += 1
        return count

    def step(self):
        count = 1
        stacked_boxes = 0
        self.list_stacks = []

        #En cada iteración se revisa el grid y se cuentan las cajas que han sido 'stackeadas'
        for _,x,y in self.grid.coord_iter():
            if(self.matrix[y][x] == 1):
                stack = self.count_box(self.grid.get_cell_list_contents((x, y)))
                self.list_stacks.append({"x": x, "y": y, "numBoxes": stack})
                if(stack == 1):
                    break
                elif(count < self.numBoxes):
                    stacked_boxes += stack
                    count += 1
                #Si todas las cajas se encuentran en un stack, se detiene la simulación
                elif(self.numBoxes == stacked_boxes):
                    self.running = False

        #Si se llega al tiempo límite, también se detiene la simulación  
        if(self.schedule.steps >= self.maxSteps): 
            self.running = False
        else:
            self.schedule.step()

#Función para cargar la imagen correspondiente para los agentes
def agent_portrayal(agent):
    if(type(agent) == Robot):
        if(agent.carry):
            return {"Shape": "images/robot_with_box.png", "Layer": 1}
        else:
            return {"Shape": "images/robot.png", "Layer": 1}
    elif(type(agent) == Wall):    
        return {"Shape": "rect", "w": 1, "h": 1, "Filled": "true", "Color": "Gray", "Layer": 0}
    else:
        if(agent.active):
            numB = agent.model.count_box(agent.model.grid.get_cell_list_contents(agent.pos))
            if(numB == 2):
                return {"Shape": "images/stack2.png", "Layer": 0}
            elif(numB == 3):
                return {"Shape": "images/stack3.png", "Layer": 0}
            elif(numB == 4):
                return {"Shape": "images/stack4.png", "Layer": 0}
            elif(numB == 5):
                return {"Shape": "images/stack5.png", "Layer": 0}
            else:
                return {"Shape": "images/box.png", "Layer": 0}

grid = CanvasGrid(agent_portrayal, 10, 10, 450, 450)

text = display_text()
server = ModularServer(Maze, [grid, text], "Robots", {})
server.port = 8522
server.launch()