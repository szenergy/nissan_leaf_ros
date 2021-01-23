#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <autoware_msgs/Lane.h>

using std::string;

class AbstractPlanner{
private:
    class PlannerState{
    private:
        bool signal_obstacle_detected;
        //*obstacles
    public:
        geometry_msgs::PoseStampedConstPtr current_pose;
        autoware_msgs::LanePtr global_waypoints;
        autoware_msgs::LanePtr closest_waypoint;
        autoware_msgs::LanePtr final_lane;
        PlannerState(){
            //Current pose of the robot (or vehicle)
            current_pose=nullptr;
            //Glboal waypoints received (i.e. base_waypoints)
            global_waypoints=nullptr;
            //Closest waypoint
            closest_waypoint=nullptr;
            //obstacles = none;
            //SIGNALS AND EVENTS
            signal_obstacle_detected=false;
        }

        bool is_initialized(){
            return (current_pose!=nullptr && global_waypoints!=nullptr && closest_waypoint!=nullptr);
        }

        bool is_obstacle_detected(){
            return signal_obstacle_detected;
        }
    };
    float m_hz;
    string global_frame="map";
    string robot_frame="base_link";
    //Internal state definition
    PlannerState planner_state;
    ros::Publisher pub_final_trajectory;
    ros::Subscriber sub_current_pose;
    ros::Subscriber sub_current_lane;
    ros::Subscriber sub_closest_waypoint;
    ros::Timer timer_pub_lane;
public:
    AbstractPlanner(ros::NodeHandle* nh,float hz):m_hz(hz){
        //Publisher initialization
        pub_final_trajectory=nh->advertise<autoware_msgs::Lane>("/final_waypoints",1);
        // Subscriber initialization
        // Subscribe the current pose of the vehicle
        sub_current_pose=nh->subscribe<geometry_msgs::PoseStamped>("/current_pose",1,cb_current_pose);
        //Global waypoints
        sub_current_lane=nh->subscribe<autoware_msgs::Lane>("/base_waypoints",1,cb_lane_waypoint);

        sub_closest_waypoint=nh->subscribe<std_msgs::Int32>("closest_waypoint",1,cb_closest_waypoints);      
    }

    void initialize_timers(ros::NodeHandle* nh){
        //Timer ROS
        timer_pub_lane=nh->createTimer(ros::Duration(1.0/m_hz),cb_plan_timer);
    }
    //REGION
    //Behavior functions
    void relay(){
        planner_state.final_lane->waypoints={};
        for(auto it=planner_state.closest_waypoint->waypoints.begin();it!=planner_state.global_waypoints->waypoints.end();++it){
            planner_state.final_lane->waypoints.push_back(*it);
        }
    }
    //REGION
    //Timer callbacks
    void cb_plan_timer(){
        if(planner_state.is_initialized()){
            relay();
            publish_final_waypoints();
        }
    }
    // REGION:
    // Subscriber callbacks
    //
    void cb_current_pose(geometry_msgs::PoseStampedConstPtr data){
        planner_state.current_pose=data;
    }

    void cb_lane_waypoint(autoware_msgs::LanePtr data){
        planner_state.global_waypoints=data;
    }

    void cb_closest_waypoints(autoware_msgs::LanePtr data){
        planner_state.closest_waypoint=data;
    }

    /*void cb_obstacle_detection(obstacle data){
        planner_state.obstacles =data;
    }*/
    // REGION:
    //Publish messages  
    void publish_final_waypoints(){
        pub_final_trajectory.publish(planner_state.final_lane);
    }
};

int main(int argc,char** argv){
    ros::init(argc,argv,"bare_planner_node");
    ros::NodeHandle nh;

    AbstractPlanner abstract_planner(&nh,20);
    abstract_planner.initialize_timers(&nh);
    ros::spin();
}