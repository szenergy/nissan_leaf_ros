#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <math.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_msgs/Lane.h>
#include <autoware_msgs/Waypoint.h>
#include <fstream>
#include <sstream>

using std::list;
using std::string;

class StructWaypoint {
private:
  double m_x, m_y,m_yaw,m_lin_vel;

public:
  StructWaypoint(double x, double y, double yaw, double lin_vel)
      : m_x(x), m_y(y), m_yaw(yaw), m_lin_vel(lin_vel) {}
};

class WaypointLoaderGlobalPlanner {
private:
  float m_hz;
  bool m_visualize = false;
  ros::Publisher pub_waypoint;
  ros::Publisher pub_visualize;
  autoware_msgs::Lane msg_pub_lane;
  visualization_msgs::MarkerArray waypoint_marker;
  visualization_msgs::Marker lane_marker;
  ros::Timer m_timer;
  ros::Timer m_visualization_timer;
public:
  WaypointLoaderGlobalPlanner(ros::NodeHandle *nh, float hz,bool visualize): m_hz(hz), m_visualize(visualize) {
    pub_waypoint = nh->advertise<autoware_msgs::Lane>("/base_waypoints", 2);
    if(visualize){
      pub_visualize=nh->advertise<visualization_msgs::MarkerArray>("/global_waypoints/visualization",1,true);
      lane_marker.header.frame_id="map";
      lane_marker.ns="waypoint_strip";
      lane_marker.color.r = 0.7f;
      lane_marker.color.g = 0.4f;
      lane_marker.color.b = 0.8f;
      lane_marker.color.a = 0.5f;
      lane_marker.scale.x = 0.2f;
      //lane_marker.scale.z = 0.2f;
      lane_marker.type=visualization_msgs::Marker::LINE_STRIP;
    }
  }

  float mapval(float x,float in_min,float in_max,float out_min,float out_max){
    return (x-in_min) * (out_max-out_min) / (in_max - in_min) + out_min;
  }

  float* gradient (float speed,float max){
    static float green [3]={92.f/255.f,255.f/255.f,236.f/255.f};
    static float red [3]={255.f/255.f,92.f/255.f,193.f/255.f};
    static float blue [3]={92.f/255.f,187.f/255.f,255.f/255.f};
    if(speed<0){
      //reversed speed
      return green;
    }
    else if (speed>max){
      return red;
    }
    else if( 0 < speed < max / 2){
      float c0 = mapval(speed,0,max/2,green[0],blue[0]);
      float c1 = mapval(speed,0,max/2,green[1],blue[1]);
      float c2 = mapval(speed,0,max/2,green[2],blue[2]);
      static float res[3]={c0,c1,c2};
      return res;
    }
    else{
      float c0 = mapval(speed,0,max/2,blue[0],red[0]);
      float c1 = mapval(speed,0,max/2,blue[1],red[1]);
      float c2 = mapval(speed,0,max/2,blue[2],red[2]);
      static float res[3]={c0,c1,c2};
      return res;
    }
  }

  list<StructWaypoint> load_csv(const string& path){
    list<StructWaypoint> waypoint_list;
    double x,y,z,yaw,lin_vel;
    std::ifstream f(path); 
    if(f.fail()){
      throw ros::Exception( path +" does not exist or waypoint_file_name param is missing.");
    }
    string header;
    std::getline(f,header);
    int i=0;   
    while(std::getline(f,header)){
      std::istringstream ss(header);
      double values[5];
      for(int j=0;j<5;j++){
        std::getline(ss,header,',');
        values[j]=std::stod(header);
      }
      x=values[0];
      y=values[1];
      z=values[2];
      yaw=values[3];
      lin_vel=values[4];
      StructWaypoint wp(x,y,yaw,lin_vel);
      waypoint_list.push_back(wp);
      autoware_msgs::Waypoint w0;
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = z;
      pose.pose.orientation.z = sin(yaw/2.0f);
      pose.pose.orientation.w = cos(yaw/2.0f);
      w0.pose=pose;
      w0.twist=geometry_msgs::TwistStamped();
      w0.twist.twist.linear.x = lin_vel;
      msg_pub_lane.waypoints.push_back(w0);
      geometry_msgs::Point p;
      if (m_visualize){
        p.x=x;
        p.y=y;
        p.z=z;
        lane_marker.points.push_back(p);
      }
      //Orientation
      visualization_msgs::Marker ori_arrow;
      ori_arrow.type=visualization_msgs::Marker::ARROW;
      ori_arrow.pose.position.x = x;
      ori_arrow.pose.position.y = y;
      ori_arrow.pose.position.z = z;
      ori_arrow.pose.orientation.z = sin(yaw/2.0f);
      ori_arrow.pose.orientation.w = cos(yaw/2.0f);
      ori_arrow.action=visualization_msgs::Marker::ADD;
      ori_arrow.color.r=0.2f;
      ori_arrow.color.g=1.0f;
      ori_arrow.color.b=0.6f;
      ori_arrow.color.a=1.0f;
      ori_arrow.scale.x=1.0f;
      ori_arrow.scale.y=0.2f;
      ori_arrow.id=i;
      ori_arrow.ns="orientation";
      ori_arrow.header.frame_id="map";
      waypoint_marker.markers.push_back(ori_arrow);
      //velocity
      visualization_msgs::Marker marker_lin_vel;
      marker_lin_vel.type=visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker_lin_vel.pose.position.x = x;
      marker_lin_vel.pose.position.y = y;
      marker_lin_vel.pose.position.z = z + 0.7;
      marker_lin_vel.ns = "linvel";
      marker_lin_vel.header.frame_id = "map";
      marker_lin_vel.color.r = 1.0f;
      marker_lin_vel.color.g = 1.0f;
      marker_lin_vel.color.a = 1.0f;
      marker_lin_vel.scale.z = 0.5f;
      marker_lin_vel.id = i;
      //Pontosan 1 tizedes pontosságú számot reprezentáló string készül:
      std::stringstream valconv;
      string val;
      valconv<<std::fixed<<std::setprecision(1)<<lin_vel;
      valconv>>val;
      marker_lin_vel.text = val;
      //Nem teljesen egy tizedes pontosságú számot reprezentáló string keszül:
      //marker_lin_vel.text = std::to_string(round(lin_vel*10)/10);
      waypoint_marker.markers.push_back(marker_lin_vel);
      //Sphere
      visualization_msgs::Marker marker_lane_points;
      marker_lane_points.header.frame_id = "map";
      marker_lane_points.ns = "lane_points";
      marker_lane_points.type = visualization_msgs::Marker::SPHERE;
      float* sphere_color = gradient(lin_vel, 10);// max speed color (red) is 10
      marker_lane_points.color.r = sphere_color[0];
      marker_lane_points.color.g = sphere_color[1];
      marker_lane_points.color.b = sphere_color[2];
      marker_lane_points.color.a = 1.0;
      marker_lane_points.scale.x = 0.2;
      marker_lane_points.scale.y = 0.2;
      marker_lane_points.scale.z = 0.2;
      marker_lane_points.pose.position = p;
      marker_lane_points.pose.orientation.w = 1.0;
      marker_lane_points.id = i;
      //marker_lane_points.Duration = 
      waypoint_marker.markers.push_back(marker_lane_points);
      if(m_visualize){
        waypoint_marker.markers.push_back(lane_marker);
      }
      i++;
    }
    ROS_INFO("Successfully loaded lane description (CSV)");
    msg_pub_lane.header.frame_id="map";
    f.close();
  }

  void initialize_timer(ros::NodeHandle* nh){
    m_timer=nh->createTimer(ros::Duration(1.0 / m_hz),timer_waypoint,true);
    if (m_visualize){
      m_visualization_timer=nh->createTimer(ros::Duration(1.0/(m_hz/2.0)),timer_visualization,true);
    }
  }

  void timer_visualization(){
    pub_visualize.publish(waypoint_marker);
  }

  void timer_waypoint(){
    msg_pub_lane.header.stamp=ros::Time::now();
    pub_waypoint.publish(msg_pub_lane);
  }
};

int main(int argc, char **argv) {
  ros::init(argc,argv,"waypoint_loader");
  ros::NodeHandle nh;

  WaypointLoaderGlobalPlanner global_planner(&nh,20,true);

  global_planner.initialize_timer(&nh);

  string file;
  nh.getParam("waypoint_file_name",file);
  bool file_ok;
  try{
    list<StructWaypoint> waypoint_list=global_planner.load_csv(file);
    ROS_INFO("%s",file);
    ROS_INFO("All set,publishing lane information");
    file_ok = true;
  }catch(const ros::Exception& e){ ROS_INFO("%s",e.what());file_ok=false;}
  if(file_ok)
    ros::spin();
}
