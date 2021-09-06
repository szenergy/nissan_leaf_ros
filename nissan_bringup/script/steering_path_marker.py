#!/usr/bin/env python  

"""
Publishes a ROS marker to visualize the current steering with a marker in RVIZ
"""
import numpy as np
from numpy.core.numeric import NaN
import rospy
#import autoware_msgs.msg as auwmsg
import std_msgs.msg as stdmsg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#import matplotlib.pyplot as plt

speed_mps = 0.0


def calcuclate_circle(wheel_ang_deg):
    global speed_mps
    """
    Calculates the circle (future trajectory) aroud the vehicle
    """
    wheel_ang_rad  = np.deg2rad(wheel_ang_deg)
    pos_x = pos_y = theta = 0.0
    delta = 0.1
    length = 2.7 # the disatance between the rear and the front axle TODO from param
    mark_x = Marker()
    if speed_mps < 4:
        speed_mps = 4
    for num in range(100): 
        pos_x += delta * speed_mps * np.cos(theta)
        pos_y += delta * speed_mps * np.sin(theta)
        theta += delta * speed_mps / length * np.tan(wheel_ang_rad)
        point_traj = Point()
        point_traj.x = pos_x
        point_traj.y = pos_y
        mark_x.points.append(point_traj)
    #print(mark_x)
    return mark_x

def wheel_ang_callback(data):
    marker_points = calcuclate_circle(data.data)
    mark_f = Marker()
    mark_f.header.frame_id = "/base_link"
    mark_f.type = mark_f.LINE_STRIP
    mark_f.action = mark_f.ADD
    mark_f.scale.x = 1
    mark_f.color.r = 0.1
    mark_f.color.g = 0.4
    mark_f.color.b = 0.9
    mark_f.color.a = 0.9 # 90% visibility
    mark_f.pose.orientation.x = mark_f.pose.orientation.y = mark_f.pose.orientation.z = 0.0
    mark_f.pose.orientation.w = 1.0
    mark_f.pose.position.x = mark_f.pose.position.y = mark_f.pose.position.z = 0.0
    mark_f.ns = "steering"
    mark_f.header.stamp = rospy.Time.now()
    mark_f.points = marker_points.points
    pub_visualize.publish(mark_f)

def veh_speed_callback(speed_kmph):
    global speed_mps
    speed_mps = speed_kmph.data / 3.6


if __name__ == '__main__':
    rospy.init_node('steering_path_marker')
    rospy.Subscriber('/wheel_angle_deg', stdmsg.Float32,  wheel_ang_callback)
    rospy.Subscriber('/vehicle_speed_kmph', stdmsg.Float32,  veh_speed_callback)
    pub_visualize = rospy.Publisher("/vehicle/steering", Marker, queue_size=1, latch=True)
    rospy.loginfo("steering_path_marker started")
    rospy.spin()
