import png
import numpy
import math
import Queue
import rospy

from Tkinter import *

import helpers

MIN_SIZE = 20


class QuadTree:

    def __init__(self, x, y, w, h, pixels, img_w, root):

        if helpers.mixed_space(x, y, w, h, pixels, img_w) and w >= MIN_SIZE:
            # print("occupied")
            self.root = self if root == None else root
            self.free = False
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.nw = QuadTree(x, y, w // 2, h // 2, pixels, img_w, self.root)
            self.ne = QuadTree(x + w // 2, y, w // 2, h //
                               2, pixels, img_w, self.root)
            self.sw = QuadTree(x, y + h // 2, w // 2, h //
                               2, pixels, img_w, self.root)
            self.se = QuadTree(x + w // 2, y + h // 2, w //
                               2, h // 2, pixels, img_w, self.root)
            self.neighbors = None

        elif helpers.free_space(x, y, w, h, pixels, img_w):
            # print("freeSpace found")
            self.root = self if root == None else root
            self.free = True
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.nw = None
            self.ne = None
            self.sw = None
            self.se = None
            self.neighbors = dict()

            # For dijkstra's
            self.dist = float('inf')  # math.inf in python3
            self.prev = None

        else:
            self.root = self if root == None else root
            self.free = False
            self.x = x
            self.y = y
            self.w = w
            self.h = h
            self.nw = None
            self.ne = None
            self.sw = None
            self.se = None
            self.neighbors = None

    def __lt__(self, other):
        return self.dist < other.dist

    def contains_point(self, x, y):
        return (x >= self.x) and (x < self.x + self.w) and (y >= self.y) and (y < self.y + self.h)

    def center_point(self):
        return (self.x + self.w / 2, self.y + self.h / 2)

    def distance_to(self, cell):
        p1 = self.center_point()
        p2 = cell.center_point()

        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def get_leaf_at(self, x, y):
        # print("(" + str(self.x) + '->' + str(self.x+self.w) +","+
        # str(self.y)+ '->' + str(self.y+self.h) + ")" + str(self.free))
        if self.free and self.contains_point(x, y):
            return self

        if self.nw != None and self.nw.contains_point(x, y):
            return self.nw.get_leaf_at(x, y)

        if self.nw != None and self.ne.contains_point(x, y):
            return self.ne.get_leaf_at(x, y)

        if self.nw != None and self.sw.contains_point(x, y):
            return self.sw.get_leaf_at(x, y)

        if self.nw != None and self.se.contains_point(x, y):
            return self.se.get_leaf_at(x, y)

        return None

    def get_neighbors(self):
        # Loop aroud perimitter of cell.
        neighbors = dict()

        for x in range(self.x, self.x + self.w):
            neighbor_north = self.root.get_leaf_at(x, self.y - MIN_SIZE / 2)
            # print("x:" + str(x)  + " self.y-mins/2:" + str(self.y - MIN_SIZE
            # / 2) + "-->" + str(neighbor_north))
            if neighbor_north != None:
                neighbors[neighbor_north] = self.distance_to(neighbor_north)

            neighbor_south = self.root.get_leaf_at(
                x, self.y + self.h + MIN_SIZE / 2)
            if neighbor_south != None:
                neighbors[neighbor_south] = self.distance_to(neighbor_south)

        for y in range(self.y, self.y + self.h):
            neighbor_west = self.root.get_leaf_at(self.x - MIN_SIZE / 2, y)
            if neighbor_west != None:
                neighbors[neighbor_west] = self.distance_to(neighbor_west)

            neighbor_east = self.root.get_leaf_at(
                self.x + self.w + MIN_SIZE / 2, y)
            if neighbor_east != None:
                neighbors[neighbor_east] = self.distance_to(neighbor_east)

        return neighbors

    def connectCells(self):
        if self.free:
            self.neighbors = self.get_neighbors()

        if self.ne != None:
            self.ne.connectCells()

        if self.nw != None:
            self.nw.connectCells()

        if self.sw != None:
            self.sw.connectCells()

        if self.se != None:
            self.se.connectCells()

    def draw(self, canvas):
        if self.nw == None:
            # Then this is a leaf
            if self.free:
                canvas.create_rectangle(
                    self.x, self.y, self.x + self.w, self.y + self.h)
            else:
                canvas.create_rectangle(
                    self.x, self.y, self.x + self.w, self.y + self.h, fill="#000")

        else:
            self.nw.draw(canvas)
            self.ne.draw(canvas)
            self.sw.draw(canvas)
            self.se.draw(canvas)

    def add_to_queue(self, queue):
        if self.free:
            queue.put(self)
        if self.nw != None:
            self.nw.add_to_queue(queue)
        if self.ne != None:
            self.ne.add_to_queue(queue)
        if self.sw != None:
            self.sw.add_to_queue(queue)
        if self.se != None:
            self.se.add_to_queue(queue)

        return


def dijkstras(start_x, start_y, goal_x, goal_y, quad_tree):
    # Connect neighboring cells in the tree, the tree is converted to a
    # graph
    quad_tree.connectCells()

    start_node = quad_tree.get_leaf_at(start_x, start_y)
    goal_node = quad_tree.get_leaf_at(goal_x, goal_y)

    start_node.dist = 0  # inf from constructor

    # add all cells to Q
    Q = Queue.PriorityQueue()
    # quad_tree.add_to_queue(Q)
    Q.put(start_node)
    while not Q.empty():
        cell = Q.get()  # get node with lowest dist
        for neighbor in cell.neighbors.keys():
            alternetive = cell.dist + cell.neighbors[neighbor]
            if alternetive < neighbor.dist:
                neighbor.dist = alternetive
                neighbor.prev = cell
                # add to prio queue
                Q.put(neighbor)

    path = []

    node = goal_node

    if node == start_node or node.prev != None:
        # We found a path
        while node != None:
            path.append(node)
            node = node.prev

    path.reverse()

    return path
