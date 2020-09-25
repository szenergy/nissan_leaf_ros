#!/usr/bin/env python  
import roslib
import tf2_ros
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('current_pose_from_tf')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    current_pub = rospy.Publisher("/current_pose", geometry_msgs.msg.PoseStamped,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue


        cmd = geometry_msgs.msg.PoseStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "map"
        cmd.pose.position.x = trans.transform.translation.x
        cmd.pose.position.y = trans.transform.translation.y
        cmd.pose.position.z = trans.transform.translation.z
        cmd.pose.orientation.w = trans.transform.rotation.w
        cmd.pose.orientation.x = trans.transform.rotation.x
        cmd.pose.orientation.y = trans.transform.rotation.y
        cmd.pose.orientation.z = trans.transform.rotation.z
        current_pub.publish(cmd)

        rate.sleep()
