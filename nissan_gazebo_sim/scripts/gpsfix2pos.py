#!/usr/bin/env python
import utm
import rospy
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import NavSatFix, Imu

class GpsToCurrentPose(object):
    def __init__(self):
        # Get your parameters        
        self.rel_pos = rospy.get_param("~rel_pos", False)
        self.pos_topic_name = rospy.get_param("~gnss_pose_topicname", "/gnss_pose")
        self.init_pos = None
        # Setup communication layer
        if self.rel_pos:
            rospy.loginfo("Relative position set")
        else:
            rospy.loginfo("Absolute UTM position set")
        rospy.loginfo("Setting pose topic to: "+self.pos_topic_name)
        self.pub_pose = rospy.Publisher(self.pos_topic_name, PoseStamped, queue_size=10)
        self.sub_navsat = rospy.Subscriber("/gps/fix", NavSatFix, self.cbNavSatFix)
        self.sub_navsat = rospy.Subscriber("/imu", Imu, self.cbImu)
        self.last_orientation = Imu()
        self.msg_current_pose = PoseStamped()
        self.msg_current_pose.header.frame_id = "map"
        
    
    def cbImu(self, data):
        self.msg_current_pose.pose.orientation = data.orientation

    
    def cbNavSatFix(self, data):
        pos = utm.from_latlon(data.latitude, data.longitude)
        if not self.rel_pos:
            self.msg_current_pose.pose.position.x = pos[0]
            self.msg_current_pose.pose.position.y = pos[1]
            self.msg_current_pose.pose.position.z = data.altitude
            self.msg_current_pose.header.stamp = data.header.stamp
        else:
            if self.init_pos is None:
                self.init_pos = Pose()
                self.init_pos.position.x = pos[0]
                self.init_pos.position.y = pos[1]
                self.init_pos.position.z = data.altitude
            self.msg_current_pose.pose.position.x = pos[0] - self.init_pos.position.x
            self.msg_current_pose.pose.position.y = pos[1] - self.init_pos.position.y
            self.msg_current_pose.pose.position.z = data.altitude - self.init_pos.position.z
            self.msg_current_pose.header.stamp = data.header.stamp
        self.pub_pose.publish(self.msg_current_pose)
            
            


def main():
    rospy.init_node("gps2fix")
    gps2pose = GpsToCurrentPose()

    rospy.spin()

if __name__=="__main__":
    main()


