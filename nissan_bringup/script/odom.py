#!/usr/bin/env python  

"""
Publishes a ROS marker to visualize the current steering with a marker in RVIZ
"""
import numpy as np
import rospy
import autoware_msgs.msg as autmsg
import geometry_msgs.msg as geomsg
import std_msgs.msg as stdmsg
#import matplotlib.pyplot as plt

speed_mps = 0.0
t1=0.0
x=y=0.0
pos_x = pos_y = theta = 0.0 


def vehicle_status_callback(data):
    global t1,pos_x,pos_y,x,y,theta,speed_mps
   
    previous_speed=speed_mps
    wheel_ang_rad=data.angle
    speed_mps=data.speed
    length = 2.7 # the disatance between the rear and the front axle TODO from param

    if speed_mps > 20:
        speed_mps = previous_speed
    
    previous_t=t1
    t1=rospy.Time.now().to_sec() 
    delta=t1-previous_t

    if delta > 1:
      delta=0.01
    
    theta += delta * speed_mps / length * np.tan(wheel_ang_rad) 

    
    pos_x = delta * speed_mps * np.cos(theta * 0.86)
    pos_y = delta * speed_mps * np.sin(theta * 0.86)
        
    x=x+pos_x
    y=y+pos_y
    

    pose=geomsg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id="base_link"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = pose.pose.orientation.y = pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    th = stdmsg.Float32()
    th.data = theta
    pub_theta.publish(th)
    pub_pose.publish(pose)



if __name__ == '__main__':
    global pub_pose
    rospy.init_node('odometry')
    rospy.Subscriber('/vehicle_status', autmsg.VehicleStatus,  vehicle_status_callback)
    pub_pose = rospy.Publisher("/odom", geomsg.PoseStamped, queue_size=1, latch=True)
    pub_theta = rospy.Publisher("/theta", stdmsg.Float32, queue_size=1, latch=True)
    rospy.loginfo("odom started")
    rospy.spin()
