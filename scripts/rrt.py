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

ROBOT_SIZE = 10
RADIUS = 50


class Node:

    def __init__(self, x, y, prev):
        self.x = x
        self.y = y
        self.prev = prev


class RRT:

    def __init__(self, start_x, start_y, goal_x, goal_y, pixels, map_w, map_h):
        self.start = Node(start_x, start_y, None)
        self.goal = Node(goal_x, goal_y, None)
        self.nodes = [self.start]
        self.map_w = map_w
        self.map_h = map_h
        self.pixels = pixels
        self.path = []

    def reachable(self, x1, y1, x2, y2, w):
        dx = x2 - x1
        dy = y2 - y1

        if dx == dy == 0:
            return True

        if dx == 0:
            for y in range(min(y1, y2), max(y1, y2)):
                u = int(round(x1 - ROBOT_SIZE / 2))
                v = int(round(y - ROBOT_SIZE / 2))

                if not helpers.free_space(u, v, ROBOT_SIZE, ROBOT_SIZE, self.pixels, self.map_w):
                    return False

        elif dy == 0:
            for x in range(min(x1, x2), max(x1, x2)):
                u = int(round(x - ROBOT_SIZE / 2))
                v = int(round(y1 - ROBOT_SIZE / 2))

                if not helpers.free_space(u, v, ROBOT_SIZE, ROBOT_SIZE, self.pixels, self.map_w):
                    return False

        else:
            dydx = float(dy) / dx
            for i in helpers.frange(min(x1, x2), max(x1, x2), 0.1):
                y = y1 + (i - x1) * dydx

                u = int(round(i - ROBOT_SIZE / 2))
                v = int(round(y - ROBOT_SIZE / 2))

                # print(str(u) + " " + str(v))

                if helpers.contains_box(0, 0, self.map_w, self.map_h, u, v, ROBOT_SIZE, ROBOT_SIZE):
                    if not helpers.free_space(u, v,
                                              ROBOT_SIZE,  ROBOT_SIZE, self.pixels, self.map_w):

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

        if self.reachable(int(x), int(y), int(closest.x), int(closest.y), int(ROBOT_SIZE)):
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
        canvas = Canvas(master, width=self.map_w,
                        height=self.map_h, bg="#fff")
        canvas.pack()
        canvas.create_rectangle(0, 0, self.map_w, self.map_h)

        img = PhotoImage(width=self.map_w, height=self.map_h)
        canvas.create_image((self.map_w / 2, self.map_h / 2),
                            image=img, state="normal")

        # Draw map
        for i in range(len(self.pixels)):
            u = i % self.map_w
            v = i // self.map_w
            if not self.pixels[i]:
                img.put("#000", (u, v))

        # Draw RRT
        for node in self.nodes:
            if node.prev != None:
                canvas.create_line(node.x, node.y, node.prev.x, node.prev.y)

        # Draw path
        for node in self.path:
            if node.prev != None:
                canvas.create_line(node.x, node.y, node.prev.x,
                                   node.prev.y, width=4, fill="#f00")

        dot_size = 5
        # Mark goal position
        canvas.create_oval(self.goal.x - dot_size, self.goal.y - dot_size, self.goal.x +
                           dot_size, self.goal.y + dot_size, outline="#000", fill="#00f")
        # Mark start position
        canvas.create_oval(self.start.x - dot_size, self.start.y - dot_size, self.start.x +
                           dot_size, self.start.y + dot_size, outline="#000", fill="#00f")

        mainloop()

    def calculate_path(self, iterations):
        nr = 0
        path_found = False
        self.path = []

        for i in range(iterations):
            if self.add_random_node():
                if distance(self.nodes[-1].x, self.nodes[-1].y, self.goal.x, self.goal.y) < RADIUS:
                    if self.reachable(self.nodes[-1].x, self.nodes[-1].y, self.goal.x, self.goal.y, w):
                        self.goal.prev = self.nodes[-1]
                        self.nodes.append(self.goal)
                        path_found = True
                        break
                nr += 1

        if path_found:
            self.path.append(self.goal)
            current = self.goal
            while current.prev != None:
                self.path.insert(0, current.prev)
                current = current.prev

            return self.path

        return None


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2))


if __name__ == '__main__':
    world_map = "/home/filip/ROS/CSCI5980/src/bitmaps/autolab.png"
    r = png.Reader(
        filename=world_map)
    w, h, pixels, metadata = r.read_flat()

    rrt = RRT(100, 200, 600, 500, pixels, w, h)

    rrt.calculate_path(20000)

    rrt.draw()
