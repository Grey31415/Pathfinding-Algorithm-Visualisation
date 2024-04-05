import math
from heapq import heappop, heappush
from termcolor import colored


class Node():
    """
    Node class for A*
    takes parent arg, positional arg, 
    g-, h-, f-values are assigned to a node

    eq: self, otherNode -> returns True if both positional arguments are the same

    dist2d: self, otherNode -> returns ecludian distance between both nodes
    """

    def __init__(self, parent = None, position = None):
        self.parent = parent
        self.position = position

        self.g = 0 #cost of path from start node to current node
        self.h = 0 #actual cost of path from current node to end node
        self.f = 0 #total cost of path from start to end using this node (f = g + h)

    def __eq__(self, otherNode):
        return self.position == otherNode.position
    
    def dist2d(self, otherNode):
        x1, y1 = self.position[0:2]
        x2, y2 = otherNode.position[0:2]

        dist = math.sqrt((x1-x2)**2 + (y1-y2)**2)

        return dist
    

def a_star(start_pos, end_pos, map, occupancy_cost, movement):
    """
    A Star pathfinding algorithm implemented for use on grid maps

    takes: start position (x, y), end position (x, y), grid map (0=clear, 1=barrier), occupancy cost factor, movement type (4N=4directions, 8N=8directions)

    returns: path as list of position-tuples from start position to end position
    """
    startNode = Node(None, start_pos)
    endNode = Node(None, end_pos)

    startNode_cost = 0
    startNode_estimated_cost_to_endNode = startNode.dist2d(endNode)
   
    front = [(startNode_estimated_cost_to_endNode, startNode_cost, start_pos, None)]

    came_from = {}

    s2 = math.sqrt(2)

    if movement == '8N':
        movements = [(1, 0, 1.0),
                    (0, 1, 1.0),
                    (-1, 0, 1.0),
                    (0, -1, 1.0),
                    (1, 1, s2),
                    (-1, 1, s2),
                    (-1, -1, s2),
                    (1, -1, s2)]

    elif movement == '4N':
        movements = [(1, 0, 1.0),
                    (0, 1, 1.0),
                    (-1, 0, 1.0),
                    (0, -1, 1.0)]
    
    else:
        raise NameError

    visited = []
    
    while front:
        element = heappop(front)

        total_cost, cost, pos, previous = element
        if pos in visited:
            continue

        visited.append(pos)

        came_from[pos] = previous

        if pos == end_pos:
            break

        for dx, dy, deltacost in movements:
            new_x = pos[0] + dx
            new_y = pos[1] + dy
            new_pos = (new_x, new_y)
            new_node = Node(previous, new_pos)

            if new_node.position[0] > (len(map) - 1) or new_node.position[0] < 0 or new_node.position[1] > (len(map[len(map)-1]) -1) or new_node.position[1] < 0:
                continue

            if map[new_node.position[0]][new_node.position[1]] != 0:
                continue

            else:
                potential_function_cost = map[new_x][new_y]*occupancy_cost
                new_cost = cost + deltacost + potential_function_cost
                new_total_cost_to_endNode = new_cost + Node.dist2d(new_node, endNode) + potential_function_cost
                newVisStat = False

                heappush(front, (new_total_cost_to_endNode, new_cost, new_pos, pos))

    path = []
    path_idx = []
    if pos == endNode.position:
        while pos:
            path_idx.append(pos)
            path.append((pos[0], pos[1]))
            pos = came_from[pos]

        path.reverse()
        path_idx.reverse()

    return path

def termMap(map, path):
    """
    maps out a given path in the given map

    takes: grid map (2D list, 0=clear, 1=barrier), path (list of tuples)

    returns: altered map with path marked
    """
    
    for pos in path:
        map[pos[0]][pos[1]] = 'o '

    start = path[0]
    end = path[-1]
    map[start[0]][start[1]] = 's '
    map[end[0]][end[1]] = 'e '

    print("\nMapped out path:", end = '')
    print('\n')
    print('+ ' + len(map[0])*'- ' + '+')

    for y in range(len(map)):
        print('| ', end = '')

        for x in range(len(map[len(map)-1])):
            val = map[y][x]

            if val == 0:
                print('• ', end = '')
            if val == 1:
                print("# ", end = '')
            if val == 'o ':
                print(colored(val, 'green'), end = '')
            if val == 's ' or val == 'e ':
                print(colored(val, 'red'), end = '')

        print('| ', end = '')
        print('\n', end = '')

    print('+ ' + len(map[0])*'- ' + '+')
    print('\n')
    print('Path length:',len(path))
    print('\n')

    return map


if __name__ == "__main__":
    startNode = (0,0)
    endNode = (1, 15)
    map = [     [0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0],
                [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], 
                [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0], 
                [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0], 
                [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0], 
                [0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0], 
                [0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0], 
                [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1], 
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1], 
                [0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0], 
                [0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0], 
                [0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0], 
                [0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1], 
                [0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0], 
                [0, 1, 0, 1, 0, 0, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0], 
                [0, 1, 0, 1, 0, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 0, 0], 
                [0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 1], 
                [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0], 
                [0, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0], 
                [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0] ]

    path = a_star(startNode, endNode, map, 3,'4N')
    print(path)
    termMap(map, path)
