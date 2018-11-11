#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from Tkinter import *

import sys
import QuadTree
import png
import numpy
import Queue
import math

x = 0
y = 0
heading = 0
pose_init = False


def map_to_world_transform(x_map, y_map, map_w, map_h, world_w, world_h):
    xw = (x_map - map_w / 2.0) * float(world_w) / map_w
    yw = (-y_map + map_h / 2.0) * float(world_h) / map_h

    return (xw, yw)


def world_to_map_transform(x_world, y_world, map_w, map_h, world_w, world_h):
    x_map = x_world * float(map_w) / world_w + map_w / 2.0
    y_map = (-y_world * float(map_h) / world_h + map_h / 2.0)

    return (x_map, y_map)


def transform_path(path, map_w, map_h, world_w, world_h):
    map_path = []
    for node in path:
        (xw, yw) = map_to_world_transform(
            node.x + node.w / 2, node.y + node.h / 2, map_w, map_h, world_w, world_h)
        map_path.append([xw, yw])

    return map_path


def current_pose_callback(data):
    global x, y, heading, pose_init
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q_ori = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion(
        [q_ori.x, q_ori.y, q_ori.z, q_ori.w])
    heading = yaw
    pose_init = True


def draw(quad_tree, w, h, path, x_goal, y_goal):
    master = Tk()
    canvas = Canvas(master, width=w, height=h)
    canvas.pack()
    canvas.create_rectangle(0, 0, w, h)
    quad_tree.draw(canvas)

    dot_size = 2

    for node in path:
        xc, yc = node.center_point()
        canvas.create_oval(xc - dot_size, yc - dot_size, xc +
                           dot_size, yc + dot_size, outline="#000", fill="#f00")

    canvas.create_oval(x_goal - dot_size, y_goal - dot_size, x_goal +
                       dot_size, y_goal + dot_size, outline="#000", fill="#00f")
    master.update()


def navigate(goal_xw, goal_yw, world_w, world_h, world_map):
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    global x
    global y
    global heading
    global pose_init

    # Create ROS node
    rospy.init_node('quadtree_planner')

    # Subscribe to currnet pose
    rospy.Subscriber('base_pose_ground_truth', Odometry, current_pose_callback)

    # Publisher
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Command to publish
    base_cmd = Twist()

    # Read Map File
    r = png.Reader(
        filename=world_map)
    w, h, pixels, metadata = r.read_flat()

    # Create quadtree
    quad = QuadTree.QuadTree(0, 0, w, h, pixels, w, None)

    # Wait for current pose
    rate = rospy.Rate(10)
    while not pose_init and not rospy.is_shutdown():
        rospy.loginfo("waiting for pose init...")
        rate.sleep()

    x_start, y_start = world_to_map_transform(x, y, w, h, world_w, world_h)
    x_goal, y_goal = world_to_map_transform(
        goal_xw, goal_yw, w, h, world_w, world_h)

    rospy.loginfo("Trying to find a path from (" + str(x) + "," +
                  str(y) + ") to (" + str(goal_xw) + ", " + str(goal_yw) + ").")

    path = QuadTree.dijkstras(x_start, y_start, x_goal, y_goal, quad)

    if path == None:
        rospy.loginfo("No path found.")
        return

    rospy.loginfo("Path found... Let's go!")

    # Render map
    draw(quad, w, h, path, x_goal, y_goal)

    world_path = transform_path(path, w, h, world_w, world_h)
    world_path.append((goal_xw, goal_yw))

    waypoint_idx = 0

    # Let's go
    while not rospy.is_shutdown():
        dx = world_path[waypoint_idx][0] - x
        dy = world_path[waypoint_idx][1] - y

        if (dx * dx + dy * dy < 0.1) and (waypoint_idx < len(world_path) - 1):
            waypoint_idx += 1

        target_heading = math.atan2(dy, dx)
        d_heading = (target_heading - heading)

        if (target_heading > math.pi / 2 and heading < -math.pi / 2) or (target_heading < -math.pi / 2 and heading > math.pi / 2):
            d_heading = - d_heading

        # Drive forward if we are facing the right direction
        if math.fabs(d_heading) < math.pi / 10 and dx * dx + dy * dy > 0.1:
            base_cmd.linear.x = 2
        else:
            base_cmd.linear.x = 0

        base_cmd.angular.z = 1 * d_heading

        velocity_publisher.publish(base_cmd)

#        velocity_publisher.publish(base_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        if len(sys.argv) < 2:
            print("usage: rosrun stage_controll quadPlanner.py goal_x goal_y")
        else:
            navigate(float(sys.argv[1]), float(
                sys.argv[2]), 54, 58.7, "/home/filip/ROS/CSCI5980/src/bitmaps/autolab.png")

    except rospy.ROSInterruptException:
        pass
