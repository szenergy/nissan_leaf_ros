// publishes nav_msgs/Path and steering marker for rviz

#include <math.h>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>

double wheelbase = 2.7; // TODO use param vehicle_model_wheelbase instead of constant
double steering_angle; 
int path_size;
ros::Publisher marker_pub, path1_pub, path2_pub;
nav_msgs::Path path1, path2;
geometry_msgs::PoseStamped actual_pose1, actual_pose2;

// Callback for steering wheel messages
void vehicleSteeringCallback(const std_msgs::Float32 &status_msg){
    steering_angle = status_msg.data * M_PI / 180; // deg2rad
}

// Callback for pose messages
void vehiclePoseCallback1(const geometry_msgs::PoseStamped &pos_msg){
    actual_pose1 = pos_msg;
}
void vehiclePoseCallback2(const geometry_msgs::PoseStamped &pos_msg){
    actual_pose2 = pos_msg;
}

void loop(){
    visualization_msgs::Marker steer_marker;
    steer_marker.header.frame_id = "base_link";
    steer_marker.header.stamp = ros::Time::now();
    steer_marker.ns = "steering_path";
    steer_marker.id = 0;
    steer_marker.type = steer_marker.LINE_STRIP;
    steer_marker.action = visualization_msgs::Marker::ADD;
    steer_marker.pose.position.x = 0;
    steer_marker.pose.position.y = 0;
    steer_marker.pose.position.z = 0;
    steer_marker.pose.orientation.x = 0.0;
    steer_marker.pose.orientation.y = 0.0;
    steer_marker.pose.orientation.z = 0.0;
    steer_marker.pose.orientation.w = 1.0;
    steer_marker.scale.x = 0.6;
    steer_marker.color.r = 0.94f;
    steer_marker.color.g = 0.83f;
    steer_marker.color.b = 0.07f;
    steer_marker.color.a = 1.0;
    steer_marker.lifetime = ros::Duration();
    double marker_pos_x = 0.0, marker_pos_y = 0.0, theta = 0.0;
    for (int i = 0; i < 100; i++)
    {
        marker_pos_x += 0.01 * 10 * cos(theta);
        marker_pos_y += 0.01 * 10 * sin(theta);
        theta += 0.01 * 10 / wheelbase * tan(steering_angle);
        geometry_msgs::Point p;
        p.x = marker_pos_x;
        p.y = marker_pos_y;
        steer_marker.points.push_back(p);
    }
    marker_pub.publish(steer_marker);
    steer_marker.points.clear();
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position = actual_pose1.pose.position; 
    pose.pose.orientation = actual_pose1.pose.orientation; 
    path1.poses.push_back(pose);
    path1.header.frame_id = "map";
    path1.header.stamp = ros::Time::now();
    // keep only the last n (path_size) path message
    if (path1.poses.size() > path_size){
        int shift = path1.poses.size() - path_size;
        path1.poses.erase(path1.poses.begin(), path1.poses.begin() + shift);
    }

    path1_pub.publish(path1);

}

int main(int argc, char **argv)
{
    std::string pose_topic1, pose_topic2, marker_topic, path_topic1, path_topic2;
    ros::init(argc, argv, "speed_control");
    ros::NodeHandle n;
    ros::NodeHandle n_private("~");
    n_private.param<std::string>("pose_topic1", pose_topic1, "/current_pose");
    n_private.param<std::string>("pose_topic2", pose_topic2, "/estimated_pose");
    n_private.param<std::string>("path_topic1", path_topic1, "/marker_path1");
    n_private.param<std::string>("path_topic2", path_topic2, "/marker_path2");
    n_private.param<std::string>("marker_topic", marker_topic, "/marker_steering");
    n_private.param<int>("path_size", path_size, 100);
    ros::Subscriber sub_steer = n.subscribe("/wheel_angle_deg", 1, vehicleSteeringCallback);
    ros::Subscriber sub_current_pose1 = n.subscribe(pose_topic1, 1, vehiclePoseCallback1);
    ros::Subscriber sub_current_pose2 = n.subscribe(pose_topic2, 1, vehiclePoseCallback2);
    marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1);
    path1_pub = n.advertise<nav_msgs::Path>(path_topic1, 1);
    path2_pub = n.advertise<nav_msgs::Path>(path_topic2, 1);
    ROS_INFO_STREAM("Node started: " << ros::this_node::getName() << " subscribed: " << pose_topic1 << " " << pose_topic2 << " publishing: " << marker_topic << " " << path_topic1 << " " << path_topic2);
    ros::Rate rate(20); // ROS Rate at 20Hz
    while (ros::ok()) {
        loop();
        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}