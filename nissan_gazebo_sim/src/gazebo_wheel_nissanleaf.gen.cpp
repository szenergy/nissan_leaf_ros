/**
  * This plugin is generated from kino-dynamic description 
  */
#include <nissan_gazebo_sim/gazebo_wheel_plugin.hpp>
#include <nissan_gazebo_sim/gazebo_szenergy_common.hpp>

#include <autoware_msgs/VehicleStatus.h>
#include <autoware_msgs/ControlCommandStamped.h>

using namespace szenergy::vehicle_dynamics_parameter;
using namespace szenergy;

namespace gazebo
{	
    class NissanleafSimulationPlugin: public SzenergyGazeboControlLogic, public ModelPlugin
    {
    protected:
      const std::string NODE_NAME;
      
    public:
        NissanleafSimulationPlugin():
        	SzenergyGazeboControlLogic(DriveMode::FRONT_WHEEL)
        {
        	param = std::unique_ptr<szenergy::VehicleParameters>(new 
                szenergy::VehicleParameters(
                    "nissanleaf",
                    0.31623,
                    2.7,
                    1.54,
                    1.535,
                    0.43
                )
            );
            dyn_param = std::unique_ptr<szenergy::VehicleDynamicParameters>(
                new szenergy::VehicleDynamicParameters(
                    0.31,
                    0.09228,
                    2.76
                )
            );
            kin_limits = std::unique_ptr<szenergy::VehicleKinematicLimits>(
                new szenergy::VehicleKinematicLimits(
                    0.4886921905584123,
                    6.12
                )
            );
        }
        virtual ~NissanleafSimulationPlugin()
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
            initializeRosControlInterface();
        }


        

        

        void setup(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
        	setupPlugin(_model, _sdf);
        }

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
        {
            setup(_model, _sdf);
            updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&NissanleafSimulationPlugin::Update, this));
            initRosNode();
        }

        void Update()
        {
            updateVehicle();
        }

    };
    GZ_REGISTER_MODEL_PLUGIN(NissanleafSimulationPlugin)
}


