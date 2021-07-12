#!/usr/bin/env python  
import tf2_ros
import rospy
#import tf
import geometry_msgs.msg

def handle_gps_pos(msg):
    br = tf2_ros.TransformBroadcaster()

    #print(msg)
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "gps_alt"
    t.transform.translation.x = msg.pose.position.x
    t.transform.translation.y = msg.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = msg.pose.orientation.x
    t.transform.rotation.y = msg.pose.orientation.y
    t.transform.rotation.z = msg.pose.orientation.z
    t.transform.rotation.w = msg.pose.orientation.w

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('gps_tf_pub_alt_debug_py')
    # todo: make it param
    rospy.Subscriber('/gps/duro/current_pose', geometry_msgs.msg.PoseStamped,  handle_gps_pos)
    rospy.spin()