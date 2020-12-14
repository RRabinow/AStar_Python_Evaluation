import math
import random
import numpy as np
from matplotlib import pyplot
from matplotlib import colors


class Astar:

    def Manhatten_Distance(self, current, other, mat, start):
        return abs(current.x - other.x) + abs(current.y - other.y)
    def Euclidian_Distance(self, current, other, mat, start):
        return math.sqrt(math.pow(abs(current.x - other.x), 2) + math.pow(abs(current.y - other.y), 2))
    def Chebyshev_Distance(self, current, other, mat, start):
        D = 1
        D2 = 1
        return D * (abs(current.x - other.x) + abs(current.y - other.y)) + (D2 - 2 * D) * min(abs(current.x - other.x), abs(current.y - other.y))
    def Octile_Distance(self, current, other, mat, start):
        D = 1
        D2 = math.sqrt(2)
        return D * (abs(current.x - other.x) + abs(current.y - other.y)) + (D2 - 2 * D) * min(abs(current.x - other.x), abs(current.y - other.y))
    def Pi_Arc_Distance(self, current, other, mat, start):
        A = (math.radians(90 * (self.Euclidian_Distance(current, other, mat, start)/self.k))) + math.radians(90)
        
        b = abs(current.x - other.x)
        c = abs(current.y - other.y)
        return 1 * math.sqrt(math.pow(b, 2) + math.pow(c, 2) - (2 * b * c * math.cos(A)))#Calculate hypotenuse of non-right hand triangle using Law of Cosines
                                                                                        #Conjecture: Will be superior to Euclidian distance when obstacles are on course
                                                                                        # because euclid assumes right traingle with 90 degree corner
                                                                                        # whereas this bases the corner off number of obstacles.
                                                                                        # This is a proxy for an arc based off a euclidian line.
    def Euclidian_Obstacle_Distance(self, current, other, mat, start):
        multiplier = Euclidian_Distance(current, other, mat, start)/Euclidian_Distance(start, current, mat, start)
        extra_obstacle_distance = curent.g_cost - Euclidian_Distance(start, current, mat, start)
        return Euclidian_Distance(current, other, mat, start) + (extra_obstacle_distance * multiplier)
    
    def Dijkstra(self, current, other, mat, start):
        return 0

    def __init__(self, matrix, edgeMode=8, HMode="Manhatten_Distance", k=0):
        self.mat = self.prepare_matrix(matrix)
        self.edgeMode = edgeMode
        self.k = k
        
        if HMode == "Euclidian_Obstacle_Distance":
            self.heuristic = self.Euclidian_Obstacle_Distance
        if HMode == "Euclidian_Distance":
            self.heuristic = self.Euclidian_Distance
        elif HMode == "Manhatten_Distance":
            self.heuristic = self.Manhatten_Distance
        elif HMode == "Chebyshev_Distance":
            self.heuristic = self.Chebyshev_Distance
        elif HMode == "Octile_Distance":
            self.heuristic = self.Octile_Distance
        elif HMode == "Pi_Arc_Distance":
            self.heuristic = self.Pi_Arc_Distance
        elif HMode == "Dijkstra":
            self.heuristic = self.Dijkstra
        else:
            self.heuristic = self.Manhatten_Distance

    class Result:
        def __init__(self, path, cost, mat, open_list, closed_list, start, end):
            self.cost = cost
            self.path = path
            self.mat = mat
            self.open_list = open_list
            self.closed_list = closed_list
            self.total_memory = len(closed_list) + len(open_list)
            self.total_visits = len(closed_list)
            self.start = start
            self.end = end

        
            
            
        def visualize(self, HType):                
            dataout = [None] * len(self.mat)
            for i in range(0, len(self.mat)):
                dataout[i] = [0] * len(self.mat[0])
                
            
                
            
            for i in range(0, len(self.open_list)): #Neighbours
                dataout[self.open_list[i].y][self.open_list[i].x] = 2 #Blue

            for i in range(0, len(self.closed_list)): #Visited Nodes
                dataout[self.closed_list[i].y][self.closed_list[i].x] = 3 # Green

            for i in range(0, len(self.path)): #Best path
                dataout[self.path[i][1]][self.path[i][0]] = 1 # Yellow
            

            for i in range(0, len(self.mat)):# Obstacles
                for j in range(0, len(self.mat[0])):
                    if (self.mat[i][j].weight == None):
                               dataout[i][j] = 4 # Black

            
            pyplot.figure(figsize=(len(self.mat), len(self.mat[0])))
            #if(len(self.closed_list) != len(self.path)):
            #    colormap = colors.ListedColormap(["lightblue","yellow","blue","black", "lightgreen"])
            #else:
            #    colormap = colors.ListedColormap(["lightblue","yellow","blue","black"])
            colormap = colors.ListedColormap(["lightblue","yellow","blue","lightgreen","black" ][0:np.max(dataout)+1])
            pyplot.title(HType, fontsize=4*math.log(len(self.mat) * len(self.mat[0])))
            pyplot.text(self.path[0][0], self.path[0][1], "S", fontsize=4*math.log(12, len(self.mat) * len(self.mat[0])), verticalalignment='center', horizontalalignment='center')

            pyplot.text(self.path[-1][0], self.path[-1][1], "E", fontsize=4*math.log(12, len(self.mat) * len(self.mat[0])), verticalalignment='center', horizontalalignment='center')
            pyplot.imshow(dataout, cmap=colormap)
            pyplot.show()
            pyplot.clf()
            pyplot.cla()
            pyplot.close()

    class Node:
        def __init__(self, x, y, weight=1):
            self.x = x
            self.y = y
            self.weight = weight
            self.g_cost = 0
            self.f_cost = 0
            self.parent = None

        def __repr__(self):
            return str(self.weight)

    def print(self):
        for y in self.mat:
            print(y)

    def prepare_matrix(self, mat):
        matrix_for_astar = []
        for y, line in enumerate(mat):
            tmp_line = []
            for x, weight in enumerate(line):
                tmp_line.append(self.Node(x, y, weight=weight))
            matrix_for_astar.append(tmp_line)
        return matrix_for_astar

    def equal(self, current, end):
        return current.x == end.x and current.y == end.y

    

    def neighbours(self, matrix, current):
        neighbours_list = []
        if self.edgeMode == 8: #Diagonal moves allowed?
            if current.x - 1 >= 0 and current.y - 1 >= 0 and matrix[current.y - 1][current.x - 1].weight is not None:
                neighbours_list.append(matrix[current.y - 1][current.x - 1])
            if current.x - 1 >= 0 and matrix[current.y][current.x - 1].weight is not None:
                neighbours_list.append(matrix[current.y][current.x - 1])
            if current.x - 1 >= 0 and current.y + 1 < len(matrix) and matrix[current.y + 1][
                current.x - 1].weight is not None:
                neighbours_list.append(matrix[current.y + 1][current.x - 1])
            if current.y - 1 >= 0 and matrix[current.y - 1][current.x].weight is not None:
                neighbours_list.append(matrix[current.y - 1][current.x])
            if current.y + 1 < len(matrix) and matrix[current.y + 1][current.x].weight is not None:
                neighbours_list.append(matrix[current.y + 1][current.x])
            if current.x + 1 < len(matrix[0]) and current.y - 1 >= 0 and matrix[current.y - 1][
                current.x + 1].weight is not None:
                neighbours_list.append(matrix[current.y - 1][current.x + 1])
            if current.x + 1 < len(matrix[0]) and matrix[current.y][current.x + 1].weight is not None:
                neighbours_list.append(matrix[current.y][current.x + 1])
            if current.x + 1 < len(matrix[0]) and current.y + 1 < len(matrix) and matrix[current.y + 1][
                current.x + 1].weight is not None:
                neighbours_list.append(matrix[current.y + 1][current.x + 1])
                
        else: #Assume only 4 edges per node/vertex
            if current.x - 1 >= 0 and matrix[current.y][current.x - 1].weight is not None:
                neighbours_list.append(matrix[current.y][current.x - 1])
            if current.y - 1 >= 0 and matrix[current.y - 1][current.x].weight is not None:
                neighbours_list.append(matrix[current.y - 1][current.x])
            if current.y + 1 < len(matrix) and matrix[current.y + 1][current.x].weight is not None:
                neighbours_list.append(matrix[current.y + 1][current.x])
            if current.x + 1 < len(matrix[0]) and matrix[current.y][current.x + 1].weight is not None:
                neighbours_list.append(matrix[current.y][current.x + 1])
                
        return neighbours_list

    def build(self, end):
        node_tmp = end
        path = []
        while (node_tmp):
            path.append([node_tmp.x, node_tmp.y])
            node_tmp = node_tmp.parent
        return list(reversed(path))

    def run(self, point_start, point_end):
        matrix = self.mat
        start = self.Node(point_start[0], point_start[1])
        end = self.Node(point_end[0], point_end[1])
        closed_list = []
        open_list = [start]

        while open_list:
            current_node = open_list[0]

            for node in open_list:
                if node.f_cost < current_node.f_cost:
                    current_node = node
            
            if self.equal(current_node, end):
                return self.Result(self.build(current_node), current_node.weight + current_node.parent.g_cost, self.mat, open_list, closed_list, start, end)

            for node in open_list:
                if self.equal(current_node, node):
                    open_list.remove(node)
                    break

            closed_list.append(current_node)
            for neighbour in self.neighbours(matrix, current_node):
                
                if neighbour in closed_list:
                    continue
                
                if self.Euclidian_Distance(neighbour, current_node, self.mat, start) + current_node.g_cost + self.heuristic(neighbour, end, self.mat, start) < neighbour.f_cost or neighbour not in open_list: #Comparator was switched for testing, original does not align with A*
                    neighbour.g_cost = self.Euclidian_Distance(neighbour, current_node, self.mat, start) + current_node.g_cost
                    neighbour.f_cost = neighbour.g_cost + self.heuristic(neighbour, end, self.mat, start)
                    neighbour.parent = current_node
                    
                if neighbour not in open_list:
                    open_list.append(neighbour)

        return None

def isOptimal(mat, edge, start, end, cost):
    astar = Astar(mat, edgeMode=edge, HMode="Dijkstra")
    result = astar.run(start, end)
    #result.visualize()
    return result.cost - cost

def randMat(x, y, k):
    mat = [None] * y
    for i in range(0, y):
        mat[i] = [1] * x

    for i in range(0, k):
        mat[random.randrange(0,y)][random.randrange(0,x)] = None
        
    return mat

def runGauntlet(x, y, k, edge, n):
    HList = ["Dijkstra", "Octile_Distance","Chebyshev_Distance","Euclidian_Distance","Pi_Arc_Distance", "Euclidian_Obstacle_Distance"]
    #List = ["Octile_Distance"]
    for l in range(0, n):
        mat = randMat(x, y, k)
        start = [random.randrange(0,y),random.randrange(0,x)]
        end = [random.randrange(0,y),random.randrange(0,x)] #Maybe this should be static?
        memcost = [0] * len(HList)
        visits = [0] * len(HList)
        optimality = [0] * len(HList)
        failures = [0] * len(HList)
        for i in range(0, len(HList)):
            astar = Astar(mat, edgeMode=edge, HMode=HList[i], k=k)
            result = astar.run(start, end)
            if (result != None):
                result.visualize(HList[i])
                memcost[i] = memcost[i] + result.total_memory
                visits[i] = visits[i] + result.total_visits
                optimality[i] = optimality[i] + isOptimal(mat, edge, start, end, result.cost)
            else:
                failures[i] = failures[i] + 1
    for i in range(0, len(HList)):
        memcost[i] = memcost[i]/n
        visits[i] = visits[i]/n
        optimality[i] = optimality[i]/n
        
    print(HList)
    print("Memcost:" + str(memcost))
    print("Visits:" + str(visits))
    print("Optimality:" + str(optimality))
    print("Failures:" + str(failures))
    return

runGauntlet(100,100,50,8, 1)
