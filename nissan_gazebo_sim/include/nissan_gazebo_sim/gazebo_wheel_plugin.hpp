/*
 * Header file for general szenergy Gazebo plugins
 */

#ifndef GAZEBO_JKK_WHEEL_PLUGIN_HPP
#define GAZEBO_JKK_WHEEL_PLUGIN_HPP

#include <algorithm>
#include <fstream>
#include <mutex>
#include <thread>
#include <memory>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <szelectricity_common/VehicleParameters.hpp>

const double MAXTORQUE = 20.0;
const double CMD_TORQUE_EPSILON = 0.0001;
const double STEER_SQR_EPSILON = 1e-5;
namespace gazebo    
{
    enum CarControlStates {INIT, IDLE, THROTTLE, BRAKE};

    class PluginControlState
    {
    public:
        double handWheelAngle = 0;
        double flSteeringAngle = 0;
        double frSteeringAngle = 0;
        double flWheelSteeringCmd = 0;
        double frWheelSteeringCmd = 0;
        double handWheelCmd = 0;
        double steeringRatio = 0;
        double frontTorque = 0;
        double prev_steering = 0;
        double cmdSteerAngle = 0.0;
        double cmdThrottle = 0.0;
        bool cmdBrake;
        

        PluginControlState():
            cmdBrake(false),
            cmdThrottle(0),
            cmdSteerAngle(0)
        {}
    };

    
    class ControlLogic
    {
    protected:
        const szenergy::VehicleParameters param;
        std::unique_ptr<PluginControlState> plugincontrolstate;
    public:        
        ControlLogic(): param(
            "szelectricity",
            szenergy::szelectricity::SZELECTRICITY_WHEEL_RADIUS, 
            szenergy::szelectricity::SZELECTRICITY_FRONT_WHEEL, 
            szenergy::szelectricity::SZELECTRICITY_WHEELBASE, 
            szenergy::szelectricity::SZELECTRICITY_REAR_WHEEL,
            0.43)
        {
            plugincontrolstate = std::unique_ptr<PluginControlState>(new PluginControlState());
        }

        virtual ~ControlLogic()
        {
            plugincontrolstate.reset();
        }

        void setSteerCommand(double wheelangle)
        {
            double tanSteer = szenergy::steerTransmissionDataPoly(wheelangle);
            plugincontrolstate->frWheelSteeringCmd = -szenergy::steerTransmissionDataPoly(-wheelangle);
            plugincontrolstate->flWheelSteeringCmd = szenergy::steerTransmissionDataPoly(wheelangle);
            
        }

        void setSteerCommandAckermann(double wheelangle)
        {
            double tanSteer = szenergy::steerTransmissionPoly(wheelangle);
            plugincontrolstate->frWheelSteeringCmd = atan2(tanSteer,
                    1 - param.front_track/2/param.wheelbase * tanSteer);
            plugincontrolstate->flWheelSteeringCmd = atan2(tanSteer,
                    1 + param.front_track/2/param.wheelbase * tanSteer);
            
        }
        
        double getRightWheelCommand()
        {
            return plugincontrolstate->frWheelSteeringCmd;
        }

        double getLeftWheelCommand()
        {
            return plugincontrolstate->flWheelSteeringCmd;
        }
    };

    

    class PluginState
    {
    public:
        common::Time lastSimTime;

        common::PID handWheelPID;
        common::PID frWheelSteeringPID;
        common::PID flWheelSteeringPID;

        physics::JointPtr flWheelSteeringJoint;

        physics::JointPtr frWheelSteeringJoint;

        physics::JointPtr blWheelJoint;
        
        physics::LinkPtr baseLink;
        physics::ModelPtr model;
        physics::WorldPtr world;
        
        std_msgs::Float64 torque_ref_msg;
    };

    class SteerWheelPlugin: public ControlLogic, public ModelPlugin
    {
    protected:
        const std::string NODE_NAME;
        // Containment
        
        std::unique_ptr<PluginState> pluginstate;
        // ROS
        ros::Subscriber sub_angle;
        ros::Subscriber sub_throttle;
        ros::Subscriber sub_brake;
        ros::Publisher pub_torque;
        std::shared_ptr<ros::NodeHandle> nh;
        // Connection handler
        event::ConnectionPtr updateConnection;
    public:
        SteerWheelPlugin(): ControlLogic(), NODE_NAME("steer_wheel_plugin_")
        {
            pluginstate = std::unique_ptr<PluginState>(new PluginState());
        }

        virtual ~SteerWheelPlugin()
        {
        	updateConnection.reset();
            pluginstate.reset();
        }

        void initRosNode()
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, NODE_NAME + pluginstate->model->GetName(),
                ros::init_options::NoSigintHandler);        
            nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
            if (nh!=nullptr)
            {
				sub_angle = nh->subscribe("/szelectricitycar/steerangle", 10, &SteerWheelPlugin::onSteerAngle, this);
				sub_throttle = nh->subscribe("/szelectricitycar/throttle", 10, &SteerWheelPlugin::onThrottle, this);
				sub_brake = nh->subscribe("/szelectricitycar/brake", 10, &SteerWheelPlugin::onBrake, this);
				pub_torque = nh->advertise<std_msgs::Float64>("/szelectricitycar/reftorque", 10);
				ROS_INFO("Starting ROS node");
            }
            else
            {
            	std::cerr << "Unable to start ROS ecosystem, skipping ROS interface\n";
            }

        }

        static double fieldWeakening(double rpm)
        {
            return rpm > 0 ? -1e-7*rpm*rpm*rpm+7e-5*rpm*rpm-0.0126*rpm-0.0562: 0.0;
        }

        void onSteerAngle(const std_msgs::Float64::ConstPtr &msg)
        {
            plugincontrolstate->cmdSteerAngle = 
                szenergy::Clamp(
                    msg->data, 
                    -szenergy::szelectricity::MAX_STEER_DEG, 
                    szenergy::szelectricity::MAX_STEER_DEG);
        }

        void onBrake(const std_msgs::Bool::ConstPtr &msg)
        {
            // Brake: boolean value of reverse button pressed
            plugincontrolstate->cmdBrake = msg->data;
        }        

        void onThrottle(const std_msgs::Float64::ConstPtr &msg)
        {
            // Throttle: value between 0..1
            plugincontrolstate->cmdThrottle = szenergy::Clamp(msg->data, 0.0, 1.0);
        }

        

        void updateSteerCommand()
        {
            setSteerCommand(plugincontrolstate->cmdSteerAngle);
        }

        void setSteerControl(double dt)
        {
            plugincontrolstate->frSteeringAngle = pluginstate->frWheelSteeringJoint->Position();
            plugincontrolstate->flSteeringAngle = pluginstate->flWheelSteeringJoint->Position();
            double flwsError = plugincontrolstate->flSteeringAngle - (plugincontrolstate->flWheelSteeringCmd);
            double flwsCmd = pluginstate->flWheelSteeringPID.Update(flwsError, dt);
            pluginstate->flWheelSteeringJoint->SetForce(0, flwsCmd);
    
            double frwsError = plugincontrolstate->frSteeringAngle - (plugincontrolstate->frWheelSteeringCmd);
            double frwsCmd =  pluginstate->frWheelSteeringPID.Update(frwsError, dt);
            pluginstate->frWheelSteeringJoint->SetForce(0, frwsCmd);
        }

        

        void setup(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            pluginstate->model = _model;
            pluginstate->world = pluginstate->model->GetWorld();
            std::cout << "Using model name: " << pluginstate->model->GetName() << std::endl;
            // Get steer joints
            pluginstate->flWheelSteeringJoint = pluginstate->model->GetJoint("front_axial_steer_left_wheel");
            pluginstate->baseLink = pluginstate->model->GetLink("base_footprint");
            if (!pluginstate->baseLink)
            {
                std::cerr << "could not find base footprint" <<std::endl;
                return;
            }
            if (!pluginstate->flWheelSteeringJoint)
            {
                std::cerr << "could not find front left wheel joint" <<std::endl;
                return;
            }
            pluginstate->frWheelSteeringJoint = pluginstate->model->GetJoint("front_axial_steer_right_wheel");
            if (!pluginstate->frWheelSteeringJoint)
            {
                std::cerr << "could not find front right wheel joint" <<std::endl;
                return;
            }

            pluginstate->blWheelJoint = pluginstate->model->GetJoint("back_axial_left_wheel");
            if (!pluginstate->frWheelSteeringJoint)
            {
                std::cerr << "could not find back axial wheel joint" <<std::endl;
                return;
            }
            std::cout << "--- Vehicle parameters ---" << std::endl;
            std::cout << "Front track width:\t" << param.front_track << std::endl;
            std::cout << "Wheel base:\t" << param.wheelbase << std::endl;
            // Setup controllers
            // TODO: now we are using built-in PID
            pluginstate->frWheelSteeringPID.Reset();
            pluginstate->flWheelSteeringPID.Reset();
            double P = 20;
            double I = 2;
            double D = 5;
            pluginstate->frWheelSteeringPID.SetPGain(P);
            pluginstate->frWheelSteeringPID.SetIGain(I);
            pluginstate->frWheelSteeringPID.SetDGain(D);
            pluginstate->flWheelSteeringPID.SetPGain(P);
            pluginstate->flWheelSteeringPID.SetIGain(I);
            pluginstate->flWheelSteeringPID.SetDGain(D);
            std::cout << "Setup completed" << std::endl;
        }

        
    };

    class SteerWheelJoints : public SteerWheelPlugin
    {
    private:
      
        // Error definition
      
        // States to trace
        CarControlStates state;
      
    public: 
        SteerWheelJoints(): state(INIT)
        {
        
        }
        ~SteerWheelJoints()
        {
        
        }
      
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        void update();      
    };
}

#endif
