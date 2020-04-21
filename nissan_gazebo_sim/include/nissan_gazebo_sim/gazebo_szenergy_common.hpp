#ifndef GAZEBO_JKK_COMMON_HPP
#define GAZEBO_JKK_COMMON_HPP

#include <thread>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <ros/advertise_options.h>

#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

#include <geometry_msgs/TwistStamped.h>

#include <szelectricity_common/VehicleParameters.hpp>

#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/ControlCommandStamped.h>

#include <ignition/math/Matrix3.hh>

namespace gazebo {


enum class DriveMode
{
	REAR_WHEEL, FRONT_WHEEL, FOURWHEELDRIVE
};

class SzenergyGazeboControlState
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
	double cmdVelocity = 0.0;
	bool cmdBrake;
	DriveMode drive_mode;
	friend class SzemissionControlLogic;
	friend class SzemissionSimulationPlugin;

	SzenergyGazeboControlState(DriveMode drive_mode):
		cmdBrake(false),
		cmdVelocity(0),
		cmdSteerAngle(0),
		drive_mode(drive_mode)
	{}
};

struct ControlParameters
{
	double P;
	double I;
	double D;
};

class SzenergyGazeboPluginState
{
public:
	common::Time lastSimTime;

	common::PID frWheelSteeringPID;
	common::PID flWheelSteeringPID;
	common::PID rvelocityPID;
	common::PID lvelocityPID;

	physics::JointPtr flWheelSteeringJoint;
	physics::JointPtr frWheelSteeringJoint;

	physics::JointPtr blWheelJoint;
	physics::JointPtr brWheelJoint;
	physics::JointPtr flWheelJoint;
	physics::JointPtr frWheelJoint;
	std::unique_ptr<physics::JointController> driven_axis_controller_right; ///< Your ol' PID velocity controller
	std::unique_ptr<physics::JointController> driven_axis_controller_left;


	physics::LinkPtr baseLink;
	physics::ModelPtr model;
	physics::WorldPtr world;

	std_msgs::Float64 velocity_ref_msg;

	// State updates
	geometry_msgs::TwistStamped vehicle_firstorder_state;
	// Vehicle status
	autoware_msgs::VehicleStatus vehicle_status;
	// Steer control PID
	ControlParameters steering_control_parameters;
	ControlParameters velocity_control_parameters;

	// Tuning parameters
	bool is_steer_tuning;
	bool is_velocity_tuning;

	friend class SzemissionControlLogic;
	friend class SzemissionSimulationPlugin;
	// Callback queue

};

enum class PluginStateType {START, INIT, STOP};

class SzenergyGazeboControlLogic
{
private:
	void queueThread()
	{
		static const double timeout = 0.01;
		while (this->nh->ok() && state_control!=PluginStateType::STOP)
		{
			this->callback_queue.callAvailable(ros::WallDuration(timeout));
		}
	}
protected:
	std::unique_ptr<SzenergyGazeboControlState> plugincontrolstate;
	std::unique_ptr<szenergy::VehicleParameters> param;             ///< Vehicle kinematic parameters
	std::unique_ptr<szenergy::VehicleDynamicParameters> dyn_param;  ///< Vehicle dynamic parameters
	std::unique_ptr<szenergy::VehicleKinematicLimits> kin_limits;   ///< Vehicle kinematic limits
	// State
	std::unique_ptr<SzenergyGazeboPluginState> pluginstate;
	// ROS related stuff
	std::shared_ptr<ros::NodeHandle> nh;
	// ROS control interface
	ros::Subscriber sub_angle;
	ros::Subscriber sub_velocity;
	ros::Subscriber sub_twist_cmd;
	ros::Subscriber sub_ctrl_cmd;
	// Publish state
	ros::Publisher  pub_current_velocity;
	ros::Publisher  pub_vehicle_status;   ///< Autoware vehicle status
	ros::Timer      state_timer;
	// Tuning parameter
	ros::Subscriber sub_steer_setP;
	ros::Subscriber sub_steer_setI;
	ros::Subscriber sub_steer_setD;
	//
	ros::Subscriber sub_velocity_setP;
	ros::Subscriber sub_velocity_setI;
	ros::Subscriber sub_velocity_setD;
	// Connection handler
	event::ConnectionPtr updateConnection;
	// Callback queue
	PluginStateType state_control;
	std::thread queue_thread_1;
	ros::CallbackQueue callback_queue;

public:
	SzenergyGazeboControlLogic(DriveMode drivemode): state_control(PluginStateType::START)
	{
		pluginstate = std::unique_ptr<SzenergyGazeboPluginState>(
				new SzenergyGazeboPluginState());
		plugincontrolstate = std::unique_ptr<SzenergyGazeboControlState>(
				new SzenergyGazeboControlState(drivemode));
		/// Plugin state
		pluginstate->vehicle_firstorder_state.header.frame_id = "base_link";
		pluginstate->vehicle_status.header.frame_id = "base_link";
	}

	virtual ~SzenergyGazeboControlLogic()
	{
		state_control = PluginStateType::STOP;
		// Tear-down
		// Stop timer
		state_timer.stop();
		// Reset pointers
		queue_thread_1.join();
		updateConnection.reset();
		plugincontrolstate.reset();
		pluginstate.reset();
	}


	inline double getRightWheelCommand()
	{
		return plugincontrolstate->frWheelSteeringCmd;
	}

	inline double getLeftWheelCommand()
	{
		return plugincontrolstate->flWheelSteeringCmd;
	}

	inline void updateSteerCommand()
	{
		setSteerCommandAckermann(plugincontrolstate->cmdSteerAngle);
	}

	void initializeRosControlInterface();

	void setSteerCommandAckermann(const double wheelangle);
	void setSteerControl(const double dt);
	void setupPlugin(physics::ModelPtr _model, sdf::ElementPtr _sdf);
	void updateVehicle();
	void updateTwistCommand(const geometry_msgs::TwistStamped::ConstPtr& msg);
	void updateTwistCommand(const double wheel_angle, const double velocity_cmd);

	inline void updateSteerAngleCommand(const std_msgs::Float64::ConstPtr &msg)
	{
		plugincontrolstate->cmdSteerAngle =  szenergy::Clamp(
				msg->data,
				-kin_limits->wheel_angle_limit,
				kin_limits->wheel_angle_limit);
	}

	inline void updateVelocityCommand(const std_msgs::Float64::ConstPtr &msg)
	{
		plugincontrolstate->cmdVelocity = msg->data;
	}

	double updateFirstOrderState()
	{
		const double _velocity = param->wheelradius*(
				pluginstate->blWheelJoint->GetVelocity(0) + pluginstate->brWheelJoint->GetVelocity(0))/2.0;
		const double _wheelangle = (pluginstate->frWheelSteeringJoint->Position(0) + pluginstate->flWheelSteeringJoint->Position(0))/2.0;
		pluginstate->vehicle_firstorder_state.twist.linear.x = _velocity;

		pluginstate->vehicle_firstorder_state.twist.angular.z = _wheelangle;

		// Update vehicle status according to wheel angle and velocity
		pluginstate->vehicle_status.speed = _velocity;
		pluginstate->vehicle_status.angle = _wheelangle;
	}

	void onCtrlCmd(const autoware_msgs::ControlCommandStamped::ConstPtr& msg)
	{
		updateTwistCommand(msg->cmd.steering_angle, msg->cmd.linear_velocity);
	}
	/**
	 *
	 * */
	void onTwistCmd(const geometry_msgs::TwistStamped::ConstPtr& msg)
	{
		updateTwistCommand(msg);
	}

	void onSteerAngle(const std_msgs::Float64::ConstPtr &msg)
	{
		updateSteerAngleCommand(msg);
	}

	void onVelocity(const std_msgs::Float64::ConstPtr &msg)
	{
		updateVelocityCommand(msg);
	}

	void onSetSteerP(const std_msgs::Float64 &msg)
	{
		pluginstate->frWheelSteeringPID.SetPGain(msg.data);
		pluginstate->flWheelSteeringPID.SetPGain(msg.data);
	}

	void onSetSteerI(const std_msgs::Float64 &msg)
	{
		pluginstate->frWheelSteeringPID.SetIGain(msg.data);
		pluginstate->flWheelSteeringPID.SetIGain(msg.data);
	}

	void onSetSteerD(const std_msgs::Float64 &msg)
	{
		pluginstate->frWheelSteeringPID.SetDGain(msg.data);
		pluginstate->flWheelSteeringPID.SetDGain(msg.data);
	}

	void onSetVelocityP(const std_msgs::Float64 &msg)
	{
		pluginstate->lvelocityPID.SetPGain(msg.data);
		pluginstate->rvelocityPID.SetPGain(msg.data);
	}

	void onSetVelocityI(const std_msgs::Float64 &msg)
	{
		pluginstate->lvelocityPID.SetIGain(msg.data);
		pluginstate->rvelocityPID.SetIGain(msg.data);
	}

	void onSetVelocityD(const std_msgs::Float64 &msg)
	{
		pluginstate->lvelocityPID.SetDGain(msg.data);
		pluginstate->rvelocityPID.SetDGain(msg.data);
	}

	void callbackUpdateState(const ros::TimerEvent&)
	{

		updateFirstOrderState();
		auto t = ros::Time::now();
		pluginstate->vehicle_firstorder_state.header.stamp = t;					///< REQ1.1 All messages published from Gazebo vehicle plugin shall be timestamped
		pluginstate->vehicle_status.header.stamp = t;							///< REQ1.1
		if (pub_current_velocity.getNumSubscribers() > 0)
		{
			pub_current_velocity.publish(pluginstate->vehicle_firstorder_state);    ///< REQ2.1 Gazebo vehicle plugins shall publish current velcoity as standard geometry message
		}
		if (pub_vehicle_status.getNumSubscribers() > 0)
		{
			pub_vehicle_status.publish(pluginstate->vehicle_status); 				///< REQ9.1 Gazebo vehicle plugins should publish Autoware vehicle stati
		}
	}


};
}
#endif
