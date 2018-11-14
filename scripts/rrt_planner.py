#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from Tkinter import *

import sys
import png
import numpy
import Queue
import math
import helpers
import rrt

x = 0
y = 0
heading = 0
pose_init = False


def current_pose_callback(data):
    global x, y, heading, pose_init
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q_ori = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion(
        [q_ori.x, q_ori.y, q_ori.z, q_ori.w])
    heading = yaw
    pose_init = True


def navigate(goal_xw, goal_yw, world_w, world_h, world_map):
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    global x
    global y
    global heading
    global pose_init

    # Create ROS node
    rospy.init_node('rrt_planner')

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
    #quad = QuadTree.QuadTree(0, 0, w, h, pixels, w, None)

    # Wait for current pose
    rate = rospy.Rate(10)
    while not pose_init and not rospy.is_shutdown():
        rospy.loginfo("waiting for pose init...")
        rate.sleep()

    x_start, y_start = helpers.world_to_map_transform(
        x, y, w, h, world_w, world_h)
    x_goal, y_goal = helpers.world_to_map_transform(
        goal_xw, goal_yw, w, h, world_w, world_h)

    rospy.loginfo("Trying to find a path from (" + str(x) + "," +
                  str(y) + ") to (" + str(goal_xw) + ", " + str(goal_yw) + ").")

    # Create RRT
    rrt_planner = rrt.RRT(x_start, y_start, x_goal, y_goal, pixels, w, h)

    path = rrt_planner.calculate_path(20000)

    #path = QuadTree.dijkstras(x_start, y_start, x_goal, y_goal, quad)

    if path == None:
        rospy.loginfo("No path found.")
        return

    rospy.loginfo("Path found... Let's go!")

    # Render map
    rrt_planner.draw()

    world_path = helpers.transform_path(path, w, h, world_w, world_h)
    #world_path.append((goal_xw, goal_yw))

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
            print("usage: rosrun stage_controll rrt_planner.py goal_x goal_y")
        else:
            navigate(float(sys.argv[1]), float(
                sys.argv[2]), 54, 58.7, "/home/filip/ROS/CSCI5980/src/bitmaps/autolab.png")

    except rospy.ROSInterruptException:
        pass
