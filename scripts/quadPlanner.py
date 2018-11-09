#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import QuadTree
import png
import numpy
import Queue
import math

x = 0
y = 0
heading = 0


def map_to_world_transform(x_map, y_map, map_w, map_h, world_w, world_h):
    xw = (x_map - map_w / 2.0) * float(world_w) / map_w
    yw = (-y_map + map_h / 2.0) * float(world_h) / map_h

    return (xw, yw)


def transform_path(path, map_w, map_h, world_w, world_h):
    map_path = []
    for node in path:
        (xw, yw) = map_to_world_transform(
            node.x, node.y, map_w, map_h, world_w, world_h)
        map_path.append([xw, yw])

    return map_path


def current_pose_callback(data):
    global x, y, heading
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    q_ori = data.pose.pose.orientation
    (roll, pitch, yaw) = euler_from_quaternion(
        [q_ori.x, q_ori.y, q_ori.z, q_ori.w])
    heading = yaw


def init(path):
    #pub = rospy.Publisher('chatter', String, queue_size=10)
    global x
    global y
    global heading

    world_path = transform_path(path, 809, 689, 54, 58.7)
    waypoint_idx = 0

    # Create ROS node
    rospy.init_node('quadtree_planner')

    # Subscribe to currnet pose
    rospy.Subscriber('base_pose_ground_truth', Odometry, current_pose_callback)

    # Publisher
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # Command to publish
    base_cmd = Twist()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        dx = world_path[waypoint_idx][0] - x
        dy = world_path[waypoint_idx][1] - y

        if (dx * dx + dy * dy < 0.1) and (waypoint_idx < len(path) - 1):
            waypoint_idx += 1

        target_heading = math.atan2(dy, dx)
        d_heading = (target_heading - heading)

        rospy.loginfo("x:" + str(x) + " y:" + str(y))
        rospy.loginfo(
            "xp:" + str(world_path[0][0]) + " yp:" + str(world_path[0][0]))
        rospy.loginfo("heading:" + str(heading))
        rospy.loginfo("target_heading:" + str(target_heading))

        if math.fabs(d_heading) < math.pi / 10:
            base_cmd.linear.x = 1
        else:
            base_cmd.linear.x = 0

        base_cmd.angular.z = 1 * d_heading

        velocity_publisher.publish(base_cmd)

#        velocity_publisher.publish(base_cmd)
        rate.sleep()


if __name__ == '__main__':
    try:
        # Read Map File
        r = png.Reader(
            filename="/home/filip/ROS/CSCI5980/src/bitmaps/autolab.png")
        w, h, pixels, metadata = r.read_flat()

        # Create quadtree
        quad = QuadTree.QuadTree(0, 0, w, h, pixels, w, None)

        path = QuadTree.dijkstras(70, 70, 550, 100, quad)

        # Render map as a png bitmap
        quad_map = numpy.ones((h, w))
        quad.draw(quad_map)

        # Mark path on map
        for node in path:
            quad_map[node.y + int(node.h / 2), node.x + int(node.w / 2)] = 0

        f = open('path.png', 'wb')  # binary mode is important
        writer = png.Writer(w, h, greyscale=True, bitdepth=1)
        writer.write(f, quad_map)
        f.close()

        init(path)

    except rospy.ROSInterruptException:
        pass
