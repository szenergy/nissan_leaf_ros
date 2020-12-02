//This node publishes a /current_pose topic based on /tf map->base_link transform
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc,char **argv){
    ros::init(argc,argv,"current_pose_from_tf");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener listener(tfBuffer);

    ros::Publisher current_pub= nh.advertise<geometry_msgs::PoseStamped>("/current_pose",1);

    ros::Rate rate(20.0);
    while(ros::ok()){
        
        geometry_msgs::TransformStamped trans;
        try{
            trans = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
        }catch(tf2::LookupException& ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(0.05).sleep();
            continue;
        }
        catch(tf2::ConnectivityException& ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(0.05).sleep();
            continue;
        }
        catch(tf2::ExtrapolationException& ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(0.05).sleep();
            continue;
        }
        
        geometry_msgs::PoseStamped cmd;
        cmd.header.stamp=ros::Time::now();
        cmd.header.frame_id="map";
        cmd.pose.position.x=trans.transform.translation.x;
        cmd.pose.position.y=trans.transform.translation.y;
        cmd.pose.position.z=trans.transform.translation.z;
        cmd.pose.orientation.w=trans.transform.rotation.w;
        cmd.pose.orientation.x=trans.transform.rotation.x;
        cmd.pose.orientation.y=trans.transform.rotation.y;
        cmd.pose.orientation.z=trans.transform.rotation.z;
        
        current_pub.publish(cmd);

        rate.sleep();
    }
}