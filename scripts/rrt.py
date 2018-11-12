#!/usr/bin/env python

import math
import png
from random import randint
import helpers

from Tkinter import *

# draw random sample from map

# Check if sampele is in free space
# Find neraset neighboring node in graph to smaple
# Check if the drawn sample is reachable from nearest node
#   if yes -> add to graph

ROBOT_SIZE = 3.0
RADIUS = 50


class Node:

    def __init__(self, x, y, prev):
        self.x = x
        self.y = y
        self.prev = prev


class RRT:

    def __init__(self, start_x, start_y, goal_x, goal_y, pixels, map_w, map_h):
        self.nodes = [Node(start_x, start_y, None)]
        self.goal = Node(goal_x, goal_y, None)
        self.map_w = map_w
        self.map_h = map_h
        self.pixels = pixels

    def reachable(self, x1, y1, x2, y2, w):
        dx = x2 - x1
        dy = y2 - y1

        if dx == dy == 0:
            return True

        if dx == 0:
            for y in range(y1, y2):
                for x in range(x1 - w, x2 + w, 1):
                    if not helpers.free_space(x, y, 1, 1, self.pixels, self.map_w):
                        return False

        elif dy == 0:
            for x in range(x1, x2):
                for y in range(y1 - w, y2 + w, 1):
                    if not helpers.free_space(x, y, 1, 1, self.pixels, self.map_w):
                        return False

        else:
            dydx = float(dy) / dx
            dydx_perp = -float(dx) / dy
            alpha = math.atan(dydx_perp)
            perp_xrange = range(round(-w * math.cos(alpha)),
                                round(w * math.cos(alpha)), 1)

            for x in range(x1, x2, 1):
                # Loop perpendicular to the line
                y = y1 + x * dydx
                for x_perp in perp_xrange:
                    if not helpers.free_space(x + x_perp, round(y + x_perp * dydx_perp), 1, 1, self.pixels, self.map_w):
                        return False

        return True

    def nearest_neighbor_in_radius(self, x, y, radius):
        min_dist = float('Inf')
        closest = None
        for node in self.nodes:
            dist = distance(x, y, node.x, node.y)
            if dist < min_dist:
                min_dist = dist
                closest = node

        if min_dist > radius:
            return None

        if self.reachable(x, y, closest.x, closest.y, ROBOT_SIZE)
            return closest
        return None

    def add_random_node(self):
        x = randint(math.ceil(ROBOT_SIZE / 2),
                    math.ceil(self.map_w - ROBOT_SIZE / 2))
        y = randint(math.ceil(ROBOT_SIZE / 2),
                    math.ceil(self.map_h - ROBOT_SIZE / 2))

        if not helpers.free_space(int(math.ceil(x - ROBOT_SIZE)), int(math.ceil(y - ROBOT_SIZE)), int(round(ROBOT_SIZE)), int(round(ROBOT_SIZE)), self.pixels, self.map_w):
            return False

        closest = self.nearest_neighbor_in_radius(x, y, RADIUS)
        if closest == None:
            return False

        self.nodes.append(Node(x, y, closest))
        return True

    def draw(self):
        master = Tk()
        canvas = Canvas(master, width=self.map_w, height=self.map_h)
        canvas.pack()
        canvas.create_rectangle(0, 0, self.map_w, self.map_h)
        master.update()

        for node in self.nodes:
            if node.prev != None:
                canvas.create_line(node.x, node.y, node.prev.x, node.prev.y)

        mainloop()


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))


if __name__ == '__main__':
    world_map = "/home/filip/ROS/CSCI5980/src/bitmaps/autolab.png"
    r = png.Reader(
        filename=world_map)
    w, h, pixels, metadata = r.read_flat()
    rrt = RRT(100, 200, 400, 400, pixels, w, h)

    nr = 0
    for i in range(10000):
        if rrt.add_random_node():
            nr += 1

    print nr

    rrt.draw()
