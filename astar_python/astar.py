import Math

class Astar:

    def Manhatten_Distance(self, current, other):
        return abs(current.x - other.x) + abs(current.y - other.y)
    def Euclidian_Distance(self, current, other):
        return Math.sqrt(Math.pow(abs(current.x - other.x), 2) + Math.pow(abs(current.y - other.y)))
    def Chebyshev_Distance(self, current, other):
        D = 1
        D2 = 1
        return D * (abs(current.x - other.x) + abs(current.y - other.y)) + (D2 - 2 * D) * min(abs(current.x - other.x), abs(current.y - other.y))
    def Octile_Distance(self, current, other):
        D = 1
        D2 = Math.sqrt(2)
        return D * (abs(current.x - other.x) + abs(current.y - other.y)) + (D2 - 2 * D) * min(abs(current.x - other.x), abs(current.y - other.y))
    def Euclidian_Distance(self, current, other):
        A = self.k * 45
        b = abs(current.x - other.x)
        c = abs(current.y - other.y)
        return Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2) - (2 * b * c * Math.cosine(A)))#Calculate hypotenuse of non-right hand triangle using Law of Cosines
                                                                                        #Conjecture: Will be superior to Euclidian distance when obstacles are on course
                                                                                        # because euclid assumes right traingle with 90 degree corner
                                                                                        # whereas this bases the corner off number of obstacles.
                                                                                        # This is a proxy for an arc based off a euclidian line.

    def __init__(self, matrix, edgeMode=8, HMode="Manhatten_Distance", k=0):
        self.mat = self.prepare_matrix(matrix)
        self.edgeMode = edgeMode
        self.k = k

        if HMode == "Manhatten_Distance":
            self.heuristic = self.Manhatten_Distance
        else:
            self.heuristic = sel.Manhatten_Distance

    class Node:
        def __init__(self, x, y, weight=0):
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
            current_node = open_list.pop()

            for node in open_list:
                if node.f_cost < current_node.f_cost:
                    current_node = node

            if self.equal(current_node, end):
                return self.build(current_node)

            for node in open_list:
                if self.equal(current_node, node):
                    open_list.remove(node)
                    break

            closed_list.append(current_node)

            for neighbour in self.neighbours(matrix, current_node):
                if neighbour in closed_list:
                    continue
                if neighbour.f_cost > current_node.f_cost or neighbour not in open_list: #Comparator was switched for testing, original does not align with A*
                    neighbour.g_cost = neighbour.weight + current_node.g_cost
                    f_cost = neighbour.g_cost + self.heuristic(neighbour, end)
                    neighbour.parent = current_node
                if neighbour not in open_list:
                    open_list.append(neighbour)

        return None

