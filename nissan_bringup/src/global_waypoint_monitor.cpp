#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <autoware_msgs/Lane.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

class WaypointMonitor{
private:
    float m_hz;
    bool m_visualize=false;
    std_msgs::Int32 msg_closest_waypoint;
    geometry_msgs::PoseStampedConstPtr current_pose;
    autoware_msgs::LaneConstPtr current_lane;
    visualization_msgs::Marker waypoint_marker;
    ros::Publisher pub_closest_waypoint;
    ros::Publisher pub_visualize;
    ros::Subscriber sub_global_waypoints;
    ros::Timer m_timer_closest_waypoint;
    ros::Timer m_timer_visualization;

public:
    WaypointMonitor(ros::NodeHandle* nh,float hz,bool visualize):m_hz(hz),m_visualize(visualize){
        pub_closest_waypoint = nh->advertise<std_msgs::Int32>("/closest_waypoint",1);
        sub_global_waypoints = nh->subscribe<autoware_msgs::Lane>("/base_waypoints",1,cb_incoming_waypoints);
        sub_global_waypoints = nh->subscribe<geometry_msgs::PoseStamped>("/current_pose",1,cb_current_pose);
        //Current state
        current_lane=nullptr;
        current_pose=nullptr;

        if(visualize){
            waypoint_marker.header.frame_id = "map";
            waypoint_marker.type = visualization_msgs::Marker::SPHERE;
            waypoint_marker.color.r = 1.0f;
            waypoint_marker.color.g = 1.0f;
            waypoint_marker.color.b = 0.0f;
            waypoint_marker.color.a = 1.0f;
            waypoint_marker.scale.x = 1.0f;
            waypoint_marker.scale.y = 1.0f;
            waypoint_marker.scale.z = 1.0f;
            pub_visualize=nh->advertise<visualization_msgs::Marker>("closest_waypoints/visualization",1);
        }
    }

    bool is_ready(){
        return (current_lane!=nullptr && current_pose!=nullptr);
    }

    void cb_incoming_waypoints(autoware_msgs::LaneConstPtr lane){
        current_lane = lane;
    }

    void cb_current_pose(geometry_msgs::PoseStampedConstPtr pose){
        current_pose = pose;
    }

    void initialize_timer(ros::NodeHandle* nh){
        m_timer_closest_waypoint=nh->createTimer(ros::Duration(1.0f / m_hz),cb_timer_lane,true);
        if(m_visualize){
            m_timer_visualization=nh->createTimer(ros::Duration(1.0f / (m_hz / 2.0f)),timer_visualization,true);
        }
    }

    void timer_visualization(){
        if(is_ready()){
           autoware_msgs::Waypoint wp = (*current_lane).waypoints[msg_closest_waypoint.data];
           waypoint_marker.pose.position.x = wp.pose.pose.position.x;
           waypoint_marker.pose.position.y = wp.pose.pose.position.y;
           waypoint_marker.pose.position.z = wp.pose.pose.position.z;
           pub_visualize.publish(waypoint_marker);
        }
    }

    void cb_timer_lane(){
        if(is_ready()){
            int min_distance = 10000000,min_index = 0,i = 0;
            for(autoware_msgs::Waypoint wp:(*current_lane).waypoints){
                double dx = wp.pose.pose.position.x - (*current_pose).pose.position.x;
                double dy = wp.pose.pose.position.y - (*current_pose).pose.position.y;
                double distance = pow(dx,2) + pow(dy,2);
                if(distance< min_distance){
                    min_distance=distance;
                    min_index=i;
                }
                i++;
            }
            msg_closest_waypoint.data = min_index;
            pub_closest_waypoint.publish(msg_closest_waypoint);
        }
    }
};


int main(int argc,char** argv){
    ros::init(argc,argv,"global_waypoint_monitor");
    ros::NodeHandle nh;
    WaypointMonitor way_mon(&nh,10.0f,true);
    way_mon.initialize_timer(&nh);
    ROS_INFO("All set, waiting for: current_pose, global_waypoints");
    ros::spin();
}