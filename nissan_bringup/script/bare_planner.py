#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PointStamped, Point, PoseStamped
from std_msgs.msg import Int32
from autoware_msgs.msg import Lane


class PlannerState(object):

    def __init__(self):
        #
        # INPUTS
        #
        # Current pose of the robot (or vehicle)
        self.current_pose = None
        # Glboal waypoints received (i.e. base_waypoints)
        self.global_waypoints = None
        # Closest waypoint
        self.closest_waypoint = None
        # Obstacles
        self.obstacles = None
        #
        # OUTPUTS
        #
        # Final (relayed or replanned) trajectory
        self.final_lane = Lane()
        #
        # SIGNALS AND EVENTS
        #
        self.signal_obstacle_detected = False

    def is_initialized(self):
        return self.current_pose is not None \
               and self.global_waypoints is not None \
               and self.closest_waypoint is not None

    def is_obstacle_detected(self):
        return self.signal_obstacle_detected


class AbstractPlanner(object):

    def __init__(self, hz, global_frame="map",
                 robot_frame="base_link"):
        #
        self.hz = hz
        # Internal state definition
        self.planner_state = PlannerState()
        # Publisher initialization
        self.pub_final_trajectory = rospy.Publisher("/final_waypoints", Lane, queue_size=1)
        # Subscriber initialization
        # Subscribe the current pose of the vehicle
        self.sub_current_pose = rospy.Subscriber("/current_pose", PoseStamped, self.cb_current_pose)
        # Glboal waypoints
        self.sub_current_lane = rospy.Subscriber("/base_waypoints", Lane, self.cb_lane_waypoints)

        self.sub_closest_waypoint = rospy.Subscriber("/closest_waypoint", Int32, self.cb_closest_waypoints)
        #
        #self.sub_obstacles = rospy.Subscriber("/detected_obstacles", DetectedObstacles, self.cb_obstacle_detection)

    def initialize_timers(self):
        # Timer ROS
        self.timer_pub_lane = rospy.Timer(rospy.Duration(1.0/self.hz), self.cb_plan_timer)

    # REGION
    # Behavior functions
    def relay(self):
        self.planner_state.final_lane.waypoints = []
        for i in range(self.planner_state.closest_waypoint,
                       len(self.planner_state.global_waypoints.waypoints)):
            self.planner_state.final_lane.waypoints.append(self.planner_state.global_waypoints.waypoints[i])

    # REGION
    # Timer callbacks
    def cb_plan_timer(self, event):
        if self.planner_state.is_initialized():
            self.relay()
            self.publish_final_waypoints()

    # REGION:
    # Subscriber callbacks
    #
    def cb_current_pose(self, data):
        self.planner_state.current_pose = data



    def cb_lane_waypoints(self, data):
        self.planner_state.global_waypoints = data

    def cb_closest_waypoints(self, data):
        self.planner_state.closest_waypoint = data.data

    def cb_obstacle_detection(self, data):
        self.planner_state.obstacles = data

    # REGION:
    # Publish messages
    #
    def publish_final_waypoints(self):
        self.pub_final_trajectory.publish(self.planner_state.final_lane)


def main():
    rospy.init_node("bare_planner_py")
    abstract_planner = AbstractPlanner(20)
    abstract_planner.initialize_timers()
    rospy.spin()

if __name__=="__main__":
    main()
