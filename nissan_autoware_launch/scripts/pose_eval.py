#!/usr/bin/env python 

#
# Created by: @kyberszittya (Hajdu Csaba)
#

import rospy

from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Float64

import math

class PoseMonitor(object):   

    def __init__(self):
        self.err_ndt = Float64()
        self.err_kalman = Float64()
        self.pub_dist = rospy.Publisher("/distance_difference_ndt_gnss", Float64, queue_size=10)
        self.pub_dist_ekf = rospy.Publisher("/distance_difference_ekf_gnss", Float64, queue_size=10)
        self.sub_ndt_pose = rospy.Subscriber("/ndt_pose", PoseStamped, self.cbNdtPose)
        self.sub_gnss_pose = rospy.Subscriber("/gnss_pose", PoseStamped, self.cbGnssPose)
        self.sub_gnss_pose = rospy.Subscriber("/ekf_pose", PoseStamped, self.cbEkfPose)
        self.eval_timer = rospy.Timer(rospy.Duration(1.0/10.0), self.cbTimer, reset=True)        

    def cbGnssPose(self, data):
        self.gnss_pose = data

    def cbNdtPose(self, data):
        self.ndt_pose = data

    def cbEkfPose(self, data):
        self.ekf_pose = data

    def cbTimer(self, event):
        if (abs(self.gnss_pose.header.stamp.to_sec() - self.ndt_pose.header.stamp.to_sec()) < 0.2):            
            dx_ndt = self.gnss_pose.pose.position.x - self.ndt_pose.pose.position.x
            dy_ndt = self.gnss_pose.pose.position.y - self.ndt_pose.pose.position.y
            # EKF
            dx_ekf = self.gnss_pose.pose.position.x - self.ekf_pose.pose.position.x
            dy_ekf = self.gnss_pose.pose.position.y - self.ekf_pose.pose.position.y
            self.err_ndt.data = math.sqrt(dx_ndt*dx_ndt + dy_ndt*dy_ndt)
            self.err_kalman.data = math.sqrt(dx_ekf*dx_ekf + dy_ekf*dy_ekf)
            if self.err_ndt.data < 3.0:            
                self.pub_dist.publish(self.err_ndt)
                self.pub_dist_ekf.publish(self.err_kalman)

def main():
    rospy.init_node("ndt_gnss_pose_evaluation")
    pomo = PoseMonitor()
    rospy.loginfo("Starting distance evaluation")
    rospy.spin()


if __name__=="__main__":
    main()