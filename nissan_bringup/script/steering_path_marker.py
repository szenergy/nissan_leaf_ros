#!/usr/bin/env python  

"""
Publishes a ROS marker to visualize the current steering with a marker in RVIZ
"""
import numpy as np
from numpy.core.numeric import NaN
import rospy
import autoware_msgs.msg as auwmsg
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
#import matplotlib.pyplot as plt


def plot_circle(wheel_ang):
    """
    Draws a circle aroud the vehicle
    """
    # todo - replace with bicycle model
    try:
        r = -1 / wheel_ang
    except:
        r = 10000
    x1 = np.linspace(-50, 50, 20)
    if r < 0:
        y1 = -1 * np.sqrt(-x1**2. + r**2.)
    else:
        y1 = np.sqrt(-x1**2. + r**2.)
    y1 -= r
    circ = np.column_stack((x1, y1))
    #print(circ)
    mark_x = Marker()
    for i in range(len(x1)):
        if not np.isnan(y1[i]):
            point_circ = Point()
            point_circ.x = x1[i]
            point_circ.y = y1[i]
            point_circ.z = 0
            mark_x.points.append(point_circ)
            #mark_x.points.append(point_circ)
            print(point_circ)
    #print(mark_x)
    return mark_x
    #plt.plot(circ[:, 1], circ[:, 0], label=str(wheel_ang))

"""
#for i in range(-25,25):
#    plot_circle(i/500)
plot_circle(0.08)
plot_circle(0.02)
plot_circle(-0.02)
plot_circle(0.0)
plot_circle(0.001)
plot_circle(0.008)
plt.axis('equal')
plt.legend()
plt.show()
"""
def veh_status_callback(data):
    marker_points = plot_circle(data.angle)
    mark_f = Marker()
    mark_f.header.frame_id = "/base_link"
    mark_f.type = mark_f.LINE_LIST
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
    #print(marker_points)
    pub_visualize.publish(mark_f)


if __name__ == '__main__':
    rospy.init_node('steering_path_marker')
    rospy.Subscriber('/vehicle_status', auwmsg.VehicleStatus,  veh_status_callback)
    pub_visualize = rospy.Publisher("/vehicle/steering", Marker, queue_size=1, latch=True)
    rospy.loginfo("steering_path_marker started")
    rospy.spin()