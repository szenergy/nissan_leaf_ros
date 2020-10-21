#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from autoware_msgs.msg import Lane
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker
import math

class WaypointMonitor(object):

    def __init__(self, hz, visualize=False):
        self.pub_closest_waypoint = rospy.Publisher("/closest_waypoint", Int32, queue_size=1)
        self.sub_global_waypoints = rospy.Subscriber("/base_waypoints", Lane,
                self.cb_incoming_waypoints, queue_size=1)
        self.sub_global_waypoints = rospy.Subscriber("/current_pose", PoseStamped,
                 self.cb_current_pose, queue_size=1)
        self.visualize = visualize
        self.hz = hz
        # Current state
        self.current_lane = None
        self.current_pose = None

        self.msg_closest_waypoint = Int32()
        if visualize:
            self.waypoint_marker = Marker()
            self.waypoint_marker.header.frame_id = "map"
            self.waypoint_marker.type = Marker.SPHERE
            self.waypoint_marker.color.r = 1.0
            self.waypoint_marker.color.g = 1.0
            self.waypoint_marker.color.b = 0.0
            self.waypoint_marker.color.a = 1.0
            self.waypoint_marker.scale.x = 1.0
            self.waypoint_marker.scale.y = 1.0
            self.waypoint_marker.scale.z = 1.0
            self.pub_visualize = rospy.Publisher("closest_waypoints/visualization", Marker, queue_size=1)

    def is_ready(self):
        return self.current_lane is not None and self.current_pose is not None

    def cb_incoming_waypoints(self, lane):
        self.current_lane = lane

    def cb_current_pose(self, pose):
        self.current_pose = pose

    def initialize_timer(self):
        self.timer_closest_waypoint = rospy.Timer(rospy.Duration(1.0 / self.hz), self.cb_timer_lane, reset=True)
        if self.visualize:
            self.timer_visualization = rospy.Timer(rospy.Duration(1.0 / (self.hz / 2.0)),
                                                   self.timer_visualization, reset=True)

    def timer_visualization(self, event):
        if self.is_ready():
            wp = self.current_lane.waypoints[self.msg_closest_waypoint.data]
            self.waypoint_marker.pose.position.x = wp.pose.pose.position.x
            self.waypoint_marker.pose.position.y = wp.pose.pose.position.y
            self.waypoint_marker.pose.position.z = wp.pose.pose.position.z
            self.pub_visualize.publish(self.waypoint_marker)


    def cb_timer_lane(self, event):
        if self.is_ready():
            min_distance, min_index = 10000000, 0
            for i, wp in enumerate(self.current_lane.waypoints):
                dx = wp.pose.pose.position.x - self.current_pose.pose.position.x
                dy = wp.pose.pose.position.y - self.current_pose.pose.position.y
                distance = dx**2 + dy**2
                if distance < min_distance:
                    min_distance, min_index = distance, i
            self.msg_closest_waypoint.data = min_index
            self.pub_closest_waypoint.publish(self.msg_closest_waypoint)

def main():
    rospy.init_node("global_waypoint_monitor_py")
    way_mon = WaypointMonitor(10.0, True)
    way_mon.initialize_timer()
    rospy.loginfo("All set, waiting for: current_pose, global_waypoints")
    rospy.spin()


if __name__=="__main__":
    main()