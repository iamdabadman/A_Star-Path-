# -*- coding: utf-8 -*-
"""
# A* Shortest Path Algorithm
# http://en.wikipedia.org/wiki/A*
# Original Code found at
#    https://www.hackerearth.com/blog/algorithms/creating-first-2d-game-algorithm/
# Originally made by
#    Arpit Mishra
#
# Modificatins made by IAmDaBadMan (2019-03-03)
#    Added Random Obstacles.
#    Added real Random Starting and Finishing points.
#    Improved the Map Printing section of the original script
#       The cost of a print() command is quite high so it is better
#       to use as few print() commands as possible.  I stored the
#       individual characters into a single string variable and printed
#       that one variable instead of using
#           horizontal_map_size * vertical_map_size print commands.
"""
from __future__ import print_function
from heapq import heappush, heappop  # for priority queue
import math
import time
import random

# for compatibility with both py2 and py3
try:
    input = raw_input
except NameError:
    pass


class Node:
    """
    for handling the individual nodes or spaces in the given map
    """

    def __init__(self, coordinates,
                 distance, priority, possible_directions):
        if isinstance(coordinates, Shift):
            self.x_position = coordinates.change_in_x
            self.y_position = coordinates.change_in_y
        elif isinstance(coordinates, Node):
            self.x_position = coordinates.x_position
            self.y_position = coordinates.y_position
        else:
            self.x_position = coordinates[0]
            self.y_position = coordinates[1]
        self.distance = distance
        self.priority = priority
        self.possible_directions = possible_directions

    def __lt__(self, other):
        """
        comparison method for priority queue
        """
        return self.priority < other.priority

    def estimate(self, xDest, yDest):
        """
        Estimation function for the remaining distance to the goal.
        """
        dx = xDest - self.x_position
        dy = yDest - self.y_position
        # Euclidian Distance
        d = math.sqrt(dx ** 2 + dy ** 2)
        # Manhattan distance: d = abs(xd) + abs(yd)
        # Chebyshev distance: d = max(abs(xd), abs(yd))
        return d

    def updatePriority(self, xDest, yDest):
        """
        employs the a-star heuristic
        """
        self.priority = self.distance + self.estimate(xDest, yDest) * 10

    def nextMove(self, d):
        """
        give higher priority to going straight instead of diagonally
        d: direction to move
        """
        if self.possible_directions == 8 and d % 2 != 0:
            self.distance += 14
        else:
            self.distance += 10







def a_chosen_direction(x, possible_directions):
    return (x + possible_directions // 2) % possible_directions


def generate_path(possible_directions, dir_map, dx, dy, xA, yA, x, y):
    """
    generate the path from finish to start by following the possible_directions
    """
    path = ""
    while not (x == xA and y == yA):
        j = dir_map[y][x]
        c = str(a_chosen_direction(j, possible_directions))
        path = c + path
        x += dx[j]
        y += dy[j]
    return path


def outside_map(x_y_shift,
        horizontal_size_of_map, vertical_size_of_map):
    return any((x_y_shift.change_in_x < 0, x_y_shift.change_in_y < 0,
        x_y_shift.change_in_x > horizontal_size_of_map - 1,
        x_y_shift.change_in_y > vertical_size_of_map - 1))







class Shift:
    """
    x -> change in x
    y -> change in y
    """
    def __init__(self, x, y):
        self.change_in_x = x
        self.change_in_y = y






def collision_with_obstacle(x_y_shift, the_map, closed_nodes_map):
    return any((the_map[x_y_shift.change_in_y][x_y_shift.change_in_x] == 1,
        closed_nodes_map[x_y_shift.change_in_y][x_y_shift.change_in_x] == 1))


def generate_a_child_node(x_y_shift, node, direction, finish_coord):
    child_node = Node(x_y_shift,
                      node.distance, node.priority,
                      node.possible_directions)
    child_node.nextMove(direction)
    xB, yB = finish_coord
    child_node.updatePriority(xB, yB)
    return child_node



def pathFind(the_map, horizontal_size_of_map, vertical_size_of_map,
             possible_directions, dx, dy, xA, yA, xB, yB):
    """
    A-star algorithm. The path returned will be a string of digits of direction
    """
    finish_coord = (xB, yB)
    start_coord = (xA, yA)
    row = [0] * horizontal_size_of_map

    # map of closed (tried-out) nodes
    closed_nodes_map = [list(row) for _ in range(vertical_size_of_map)]

    # map of open (not-yet-tried) nodes
    open_nodes_map = [list(row) for _ in range(vertical_size_of_map)]

    # map of possible_directions
    dir_map = [list(row) for _ in range(vertical_size_of_map)]

    priority_queues = [[], []]  # priority queues of open (not-yet-tried) nodes
    priority_queue_indx = 0
    # create the start node and push into list of open nodes
    node = Node(start_coord, 0, 0, possible_directions=possible_directions)
    node.updatePriority(xB, yB)
    heappush(priority_queues[priority_queue_indx], node)
    open_nodes_map[yA][xA] = node.priority  # mark it on the open nodes map
    # A* search
    while len(priority_queues[priority_queue_indx]) > 0:
        # get the current node with the highest priority
        # from the list of open nodes
        top_node = priority_queues[priority_queue_indx][0]
        node = Node(top_node,
                    top_node.distance, top_node.priority,
                    possible_directions=possible_directions)
        x = node.x_position
        y = node.y_position
        heappop(priority_queues[priority_queue_indx])  # remove the node from the open list
        open_nodes_map[y][x] = 0
        closed_nodes_map[y][x] = 1  # mark it on the closed nodes map

        # quit searching when the goal is reached if node.estimate(xB, yB) == 0:
        if node.x_position == xB and node.y_position == yB:
            return generate_path(possible_directions, dir_map,
                                 dx, dy, xA, yA,
                                 node.x_position, node.y_position)

        # generate moves (child nodes) in all possible possible_directions
        for direction in range(possible_directions):
            change_in_x = node.x_position + dx[direction]
            change_in_y = node.y_position + dy[direction]
            x_y_shift = Shift(change_in_x, change_in_y)
            if not (outside_map(x_y_shift, horizontal_size_of_map, vertical_size_of_map) or
                    collision_with_obstacle(x_y_shift, the_map, closed_nodes_map)):
                child_node = generate_a_child_node(x_y_shift, node,
                                                   direction, finish_coord)

                # if it is not in the open list then add into that
                if open_nodes_map[x_y_shift.change_in_y][x_y_shift.change_in_x] == 0:
                    open_nodes_map[x_y_shift.change_in_y][x_y_shift.change_in_x] = child_node.priority
                    heappush(priority_queues[priority_queue_indx], child_node)
                    # mark its parent node direction
                    dir_map[x_y_shift.change_in_y][x_y_shift.change_in_x] = a_chosen_direction(direction, possible_directions=possible_directions)

                elif open_nodes_map[x_y_shift.change_in_y][x_y_shift.change_in_x] > child_node.priority:
                    # update the priority
                    open_nodes_map[x_y_shift.change_in_y][x_y_shift.change_in_x] = child_node.priority
                    # update the parent direction
                    dir_map[x_y_shift.change_in_y][x_y_shift.change_in_x] = a_chosen_direction(direction, possible_directions=possible_directions)

                    # replace the node by emptying one priority_queues to the other one
                    # except the node to be replaced will be ignored and
                    # the new node will be pushed in instead
                    while not (priority_queues[priority_queue_indx][0].x_position == x_y_shift.change_in_x and
                               priority_queues[priority_queue_indx][0].y_position == x_y_shift.change_in_y):
                        heappush(priority_queues[1 - priority_queue_indx], priority_queues[priority_queue_indx][0])
                        heappop(priority_queues[priority_queue_indx])
                    heappop(priority_queues[priority_queue_indx]) # remove the target node

                    # empty the larger size priority queue
                    # to the smaller one
                    if len(priority_queues[priority_queue_indx]) > len(priority_queues[1 - priority_queue_indx]):
                        priority_queue_indx = 1 - priority_queue_indx
                    while len(priority_queues[priority_queue_indx]) > 0:
                        heappush(priority_queues[1-priority_queue_indx], priority_queues[priority_queue_indx][0])
                        heappop(priority_queues[priority_queue_indx])
                    priority_queue_indx = 1 - priority_queue_indx
                    heappush(priority_queues[priority_queue_indx], child_node) # add the better node instead
    return '' # if no route found





if __name__ == "__main__":

    
    def distance( xA, yA, xB, yB):
        return math.sqrt( ( xB - xA) ** 2 + ( yB - yA) ** 2)
                      
    possible_directions = 8 # number of possible directions to move on the map
    if possible_directions == 4:
        dx = [1, 0, -1, 0]
        dy = [0, 1, 0, -1]
    elif possible_directions == 8:
        dx = [1, 1, 0, -1, -1, -1, 0, 1]
        dy = [0, 1, 1, 1, 0, -1, -1, -1]

    # DEFINE MAP SIZE HERE
    horizontal_size_of_map = 100
    vertical_size_of_map = 100
    the_map = []
    row = [0] * horizontal_size_of_map
    for i in range(vertical_size_of_map): # create empty map
        the_map.append(list(row))

    #########################################################################################################################
    #
    # Place random lines of random lengths and random directions throughout the graph.
    #
    
    NUMBER_OF_OBSTACLES = 35
    
    for lines in range( 1, NUMBER_OF_OBSTACLES):
        line_direction = random.randint(0, 1)
        line_length = random.randint(10, 30)
        if line_direction == 0: # Horizontal Line
            start_y = random.randint( 1, vertical_size_of_map - 1)
            start_x = random.randint( 1, horizontal_size_of_map - line_length - 1)
            for index in range( 1, line_length):
                the_map[ start_y][start_x + index] = 1
        else:
            start_y = random.randint( 1, vertical_size_of_map - line_length - 1)
            start_x = random.randint( 1, horizontal_size_of_map - 1)
            for index in range( 1, line_length):
                the_map[ start_y + index][start_x] = 1

    ##########################################################################################################################
    #
    # RANDOMLY SELECT START AND FINISH POINTS
    #
    #    The While loops ensure that the Start and Finish points are not placed on an obstacle.
    #
    
    while True:
        xA = random.randint( 1, horizontal_size_of_map - 1)
        yA = random.randint( 1, vertical_size_of_map - 1)
        if the_map[ yA][xA] == 0:
            break
    the_map[ yA][xA] = 2

    while True:
        xB = random.randint( 1, horizontal_size_of_map - 1)
        yB = random.randint( 1, vertical_size_of_map - 1)
        if the_map[ yB][ xB] == 0 and distance( xA, yA, xB, yB) > 40:
            break
    the_map[ yB][xB] = 4


    

    print('Map size (X,Y): ', horizontal_size_of_map, vertical_size_of_map)
    print('Start: ', xA, yA)
    print('Finish: ', xB, yB)
    t = time.time()
    route = pathFind(the_map, horizontal_size_of_map, vertical_size_of_map, possible_directions, dx, dy, xA, yA, xB, yB)
    print('Time to generate the route (seconds): ', time.time() - t)
    print('Route:')
    print(route)

    # mark the route on the map
    if len(route) > 0:
        x = xA
        y = yA
        the_map[y][x] = 2
        for i in range(len(route)):
            j = int(route[i])
            x += dx[j]
            y += dy[j]
            the_map[y][x] = 3
        the_map[y][x] = 4

    # display the map with the route added
    print('Map:')
    printMap = ""
    for y in range(vertical_size_of_map):
        for x in range(horizontal_size_of_map):
            xy = the_map[y][x]
            if xy == 0:
                printMap += '.'
            elif xy == 1:
                printMap += '█'
            elif xy == 2:
                printMap += 'S'
            elif xy == 3:
                printMap += 'R'
            elif xy == 4:
                printMap += 'F'
        printMap += '\n'
    print( printMap)
