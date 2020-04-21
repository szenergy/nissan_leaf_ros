#include <nissan_gazebo_sim/gazebo_szenergy_common.hpp>

namespace gazebo
{
    void SzenergyGazeboControlLogic::setSteerCommandAckermann(const double wheelangle)
    {
        double tanSteer = wheelangle;
        plugincontrolstate->frWheelSteeringCmd = atan2(tanSteer,
                1 - param->front_track/2/param->wheelbase * tanSteer);
        plugincontrolstate->flWheelSteeringCmd = atan2(tanSteer,
                1 + param->front_track/2/param->wheelbase * tanSteer);
    }
    

    void SzenergyGazeboControlLogic::setSteerControl(const double dt)
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

    void SzenergyGazeboControlLogic::setupPlugin(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        pluginstate->model = _model;
        pluginstate->world = pluginstate->model->GetWorld();
        std::cout << "Using model name: " << pluginstate->model->GetName() << std::endl;
        pluginstate->baseLink = pluginstate->model->GetLink("base_link");
        if (!pluginstate->baseLink)
        {
            std::cerr << "could not find base footprint" <<std::endl;
            return;
        }
        // Get steer joints
        pluginstate->flWheelSteeringJoint = pluginstate->model->GetJoint("jnt_front_left_steer");
        if (!pluginstate->flWheelSteeringJoint)
        {
            std::cerr << "could not find front left wheel joint" <<std::endl;
            return;
        }
        pluginstate->frWheelSteeringJoint = pluginstate->model->GetJoint("jnt_front_right_steer");
        if (!pluginstate->frWheelSteeringJoint)
        {
            std::cerr << "could not find front right wheel joint" <<std::endl;
            return;
        }

        pluginstate->frWheelJoint = pluginstate->model->GetJoint("joint_wheel_front_right");
        if (!pluginstate->frWheelJoint)
        {
            std::cerr << "could not find front axial wheel joint" <<std::endl;
            return;
        }
        pluginstate->flWheelJoint = pluginstate->model->GetJoint("joint_wheel_front_left");
        if (!pluginstate->flWheelJoint)
        {
            std::cerr << "could not find front axial wheel joint" <<std::endl;
            return;
        }

        pluginstate->brWheelJoint = pluginstate->model->GetJoint("joint_wheel_rear_right");
        if (!pluginstate->brWheelJoint)
        {
            std::cerr << "could not find back axial wheel joint" <<std::endl;
            return;
        }
        pluginstate->blWheelJoint = pluginstate->model->GetJoint("joint_wheel_rear_left");
        if (!pluginstate->blWheelJoint)
        {
            std::cerr << "could not find back axial wheel joint" <<std::endl;
            return;
        }
        std::cout << "--- Vehicle parameters ---" << std::endl;
        std::cout << "Front track width:\t" << param->front_track << std::endl;
        std::cout << "Wheel base:\t" << param->wheelbase << std::endl;
        // Setup controllers
        // TODO: now we are using built-in PID
        if (_sdf->HasElement("steer_control_tune"))
        {
        	pluginstate->is_steer_tuning = _sdf->Get<bool>("steer_control_tune");
        }
        else
        {
        	pluginstate->is_steer_tuning = false;
        }
        if (_sdf->HasElement("velocity_control_tune"))
		{
			pluginstate->is_velocity_tuning = _sdf->Get<bool>("velocity_control_tune");
		}
		else
		{
			pluginstate->is_velocity_tuning = false;
		}
        pluginstate->frWheelSteeringPID.Reset();
        pluginstate->flWheelSteeringPID.Reset();
        pluginstate->steering_control_parameters.P = _sdf->Get<double>("start_steer_p_gain");
        //double P = 1000;
        pluginstate->steering_control_parameters.I = _sdf->Get<double>("start_steer_i_gain");
        //double I = 1;
        pluginstate->steering_control_parameters.D = _sdf->Get<double>("start_steer_d_gain");
        //double D = 500;
        pluginstate->frWheelSteeringPID.SetPGain(pluginstate->steering_control_parameters.P);
        pluginstate->frWheelSteeringPID.SetIGain(pluginstate->steering_control_parameters.I);
        pluginstate->frWheelSteeringPID.SetDGain(pluginstate->steering_control_parameters.D);
        pluginstate->flWheelSteeringPID.SetPGain(pluginstate->steering_control_parameters.P);
        pluginstate->flWheelSteeringPID.SetIGain(pluginstate->steering_control_parameters.I);
        pluginstate->flWheelSteeringPID.SetDGain(pluginstate->steering_control_parameters.D);
        // Velocity tuning
        pluginstate->velocity_control_parameters.P = _sdf->Get<double>("start_velocity_p_gain");
        pluginstate->velocity_control_parameters.I = _sdf->Get<double>("start_velocity_i_gain");
        pluginstate->velocity_control_parameters.D = _sdf->Get<double>("start_velocity_d_gain");
        pluginstate->flWheelJoint->SetEffortLimit(0, 820);
        pluginstate->frWheelJoint->SetEffortLimit(0, 820);
        pluginstate->lvelocityPID.SetPGain(pluginstate->velocity_control_parameters.P);
        pluginstate->lvelocityPID.SetIGain(pluginstate->velocity_control_parameters.I);
        pluginstate->lvelocityPID.SetDGain(pluginstate->velocity_control_parameters.D);
        pluginstate->rvelocityPID.SetPGain(pluginstate->velocity_control_parameters.P);
		pluginstate->rvelocityPID.SetIGain(pluginstate->velocity_control_parameters.I);
		pluginstate->rvelocityPID.SetDGain(pluginstate->velocity_control_parameters.D);
    }

    void SzenergyGazeboControlLogic::updateVehicle()
    {
    	auto baseLinearVelocity = pluginstate->baseLink->WorldCoGLinearVel();
        auto aeroDrag = -(dyn_param->aerodrag*baseLinearVelocity.SquaredLength()
            *baseLinearVelocity.Normalized());
        pluginstate->baseLink->AddForce(aeroDrag);
        common::Time curTime = pluginstate->world->SimTime();
        double dt = (curTime - pluginstate->lastSimTime).Double();
        updateSteerCommand();
        setSteerControl(dt);
        pluginstate->baseLink->AddForce(
            -1.0*baseLinearVelocity.Normalized()*dyn_param->motordrag*baseLinearVelocity.Length()
        );
        if (baseLinearVelocity.SquaredLength() > 0.0)
        {
            pluginstate->baseLink->AddForce(
                -1.0*baseLinearVelocity.Normalized()
                *(dyn_param->rollingcoefficient+1.2+0.5)
            );
        }
        double s = szenergy::Sgn<double>(plugincontrolstate->cmdVelocity);
        // TODO: PID control, wheel radius
        double right_velocityerror = plugincontrolstate->cmdVelocity - pluginstate->frWheelJoint->GetVelocity(0)*param->wheelradius;
        //double right_velocityerror = plugincontrolstate->cmdVelocity - baseLinearVelocity.Length()*s;
        double left_velocityerror = plugincontrolstate->cmdVelocity - pluginstate->flWheelJoint->GetVelocity(0)*param->wheelradius;
        //double left_velocityerror = plugincontrolstate->cmdVelocity - baseLinearVelocity.Length()*s;
		double right_velocitycmd = pluginstate->rvelocityPID.Update(right_velocityerror, dt);
		double left_velocitycmd = pluginstate->lvelocityPID.Update(left_velocityerror, dt);
		//pluginstate->flWheelJoint->SetForce(0, right_velocitycmd);
		pluginstate->frWheelJoint->SetForce(0, -right_velocitycmd);
		pluginstate->flWheelJoint->SetForce(0, -left_velocitycmd);
		//pluginstate->driven_axis_controller_left->Update();
		//pluginstate->driven_axis_controller_right->Update();
        /*
        
        */
    }   

    void SzenergyGazeboControlLogic::updateTwistCommand(const geometry_msgs::TwistStamped::ConstPtr& msg)
    {
        plugincontrolstate->cmdVelocity = msg->twist.linear.x;
        plugincontrolstate->cmdSteerAngle =
            szenergy::Clamp(
                msg->twist.angular.z,
                -kin_limits->wheel_angle_limit,
                kin_limits->wheel_angle_limit);
    }

    void SzenergyGazeboControlLogic::updateTwistCommand(const double wheel_angle, const double velocity_cmd)
    {
        plugincontrolstate->cmdVelocity = velocity_cmd;
        plugincontrolstate->cmdSteerAngle =
            szenergy::Clamp(
                wheel_angle,
                -kin_limits->wheel_angle_limit,
                kin_limits->wheel_angle_limit);
    }

    void SzenergyGazeboControlLogic::initializeRosControlInterface()
    {
        nh = std::shared_ptr<ros::NodeHandle>(new ros::NodeHandle());
        // TODO: callback queues
        sub_twist_cmd = nh->subscribe("/twist_cmd", 10, &SzenergyGazeboControlLogic::onTwistCmd, this);
        ros::SubscribeOptions so_ctrl_cmd =
        		ros::SubscribeOptions::create<autoware_msgs::ControlCommandStamped>(
        				"/ctrl_cmd", 10,
						boost::bind(&SzenergyGazeboControlLogic::onCtrlCmd, this, _1),
						ros::VoidPtr(), &this->callback_queue);
        sub_twist_cmd = nh->subscribe("/ctrl_cmd", 10, &SzenergyGazeboControlLogic::onCtrlCmd, this);
        //sub_ctrl_cmd = nh->subscribe(so_ctrl_cmd);
        // SImple interfaces
        sub_angle = nh->subscribe(param->vehicle_name+"/steerangle", 10, &SzenergyGazeboControlLogic::onSteerAngle, this);
        sub_velocity = nh->subscribe(param->vehicle_name+"/refvelocity", 10, &SzenergyGazeboControlLogic::onVelocity, this);
        /// State publisher for autoware
        pub_current_velocity = nh->advertise<geometry_msgs::TwistStamped>("/current_velocity", 10);						///< Initialize publisher (current_velocity)
        pub_vehicle_status = nh->advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 10);						///< Initialize publisher (vehicle_status)
        state_timer = nh->createTimer(ros::Duration(1.0/40.0), &SzenergyGazeboControlLogic::callbackUpdateState, this);
        ROS_INFO("Starting ROS node");
        queue_thread_1 = std::thread(std::bind(&SzenergyGazeboControlLogic::queueThread, this));
        state_control = PluginStateType::INIT;
        if (pluginstate->is_steer_tuning)
        {
        	sub_steer_setP = nh->subscribe("set_steer_p", 10, &SzenergyGazeboControlLogic::onSetSteerP, this);
        	sub_steer_setI = nh->subscribe("set_steer_i", 10, &SzenergyGazeboControlLogic::onSetSteerI, this);
        	sub_steer_setD = nh->subscribe("set_steer_d", 10, &SzenergyGazeboControlLogic::onSetSteerD, this);
        }
        if (pluginstate->is_velocity_tuning)
        {
        	// TODO: tuning interface
        	sub_velocity_setP = nh->subscribe("set_velocity_p", 10, &SzenergyGazeboControlLogic::onSetVelocityP, this);
			sub_velocity_setI = nh->subscribe("set_velocity_i", 10, &SzenergyGazeboControlLogic::onSetVelocityI, this);
			sub_velocity_setD = nh->subscribe("set_velocity_d", 10, &SzenergyGazeboControlLogic::onSetVelocityD, this);
        }
    }
}

