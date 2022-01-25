#!/usr/bin/env python  

import numpy as np
import rospy
import autoware_msgs.msg as autmsg
import geometry_msgs.msg as geomsg
import std_msgs.msg as stdmsg
import math
import tf2_ros



speed_mps = 0.0
t1 = 0.0

pos_x = 0.0
pos_y = 0.0
dx = dy = 0.0 
d=0.0
theta =x=y = 0.0
wheel_ang_rad=0.0
mod_theta=0.0
reversed = None
params=rospy.get_param(rospy.get_param("car_name"))
wheelbase = params['wheelbase'] 



def vehicle_status_callback(data):
    global t1,pos_x,pos_y,x,y,theta,speed_mps,dx,dy,d,wheel_ang_rad,mod_theta,wheelbase,reversed

    previous_angle = wheel_ang_rad
    previous_speed=speed_mps
    wheel_ang_rad=data.angle
    speed_mps=data.speed 
    mod_theta= theta

    if mod_theta > math.pi:
        mod_theta = theta - (2*math.pi)
    elif mod_theta < - math.pi :
        mod_theta = theta + (2*math.pi)

    if wheel_ang_rad > 0.541052 or wheel_ang_rad < - 0.541052:
        wheel_ang_rad = previous_angle

    if speed_mps > 20:
        speed_mps = previous_speed
    

    vx = speed_mps * np.cos(wheel_ang_rad)

    previous_t=t1
    t1=rospy.Time.now().to_sec() 
    delta=t1-previous_t

    if delta > 1:
      delta=0.01  

    if wheel_ang_rad > 0.0:                                #balra kanyar
        multipled_wheel_angle_rad = 1.0 * wheel_ang_rad
    elif wheel_ang_rad < 0.0:                              #jobbra kanyar
        multipled_wheel_angle_rad = 1.0 * wheel_ang_rad
    else:
        multipled_wheel_angle_rad = 0.0

    pub_wheel_angle_mult.publish(multipled_wheel_angle_rad)

    

    pos_x = delta * vx * np.cos(mod_theta)
    pos_y = delta * vx * np.sin(mod_theta)

    if reversed == False:
        x=x+pos_x
        y=y+pos_y
        theta += delta * vx / wheelbase * np.tan(wheel_ang_rad) 

        dx=dx+abs(pos_x)
        dy=dy+abs(pos_y)
        
        d=math.sqrt((dx**2)+(dy**2))
    else:
        x=y=theta=dx=dy=d=0.0
    
    pose=geomsg.PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id="odom"
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    if reversed == False:
        pose.pose.orientation.z = math.sin(mod_theta/2.0)
        pose.pose.orientation.w = math.cos(mod_theta/2.0)
    else:
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
    
    br = tf2_ros.TransformBroadcaster()
    t = geomsg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = 0.0
    
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    if reversed == False:
        t.transform.rotation.z = math.sin(mod_theta/2.0)
        t.transform.rotation.w = math.cos(mod_theta/2.0)
    else:
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

    br.sendTransform(t)


    

    th = stdmsg.Float32()
    th.data = mod_theta
    pub_theta.publish(th)

    dist = stdmsg.Float32()
    dist.data = d
    pub_dist.publish(dist)
    pub_pose.publish(pose)

def TransmissionStateCallback(msg):
    global reversed
    if msg.data=="DRIVE":
        reversed=False
    elif msg.data=="REVERSE":
        reversed=True


if __name__ == '__main__':
    global pub_pose,pub_theta,pub_dist,pub_dist_gps,pub_dist_delta,pub_theta_diff
    rospy.init_node('odometry')
    rospy.Subscriber('/vehicle_status', autmsg.VehicleStatus,  vehicle_status_callback)
    rospy.Subscriber('/transmissionstate',stdmsg.String,TransmissionStateCallback)
    pub_pose = rospy.Publisher("/odom", geomsg.PoseStamped, queue_size=1, latch=True)
    pub_theta = rospy.Publisher("/theta", stdmsg.Float32, queue_size=1, latch=True)
    pub_dist = rospy.Publisher("/distance", stdmsg.Float32, queue_size=1, latch=True)
    pub_wheel_angle_mult = rospy.Publisher("/wheel_ang_mult", stdmsg.Float32, queue_size=1, latch=True)   
    rospy.loginfo("odom started")
    rospy.spin()
