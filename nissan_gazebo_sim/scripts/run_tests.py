import rospy

from std_msgs.msg import Float64
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, SetModelStateRequest, SetModelStateResponse
from gazebo_msgs.msg import ModelStates, ModelState

from geometry_msgs.msg import Pose

from rei_control_msgs.msg import SuspensionFeedback
from autoware_msgs.msg import ControlCommandStamped

import os



class SuspensionDataCollector(object):

    def __init__(self):
        self.pub_vehicle_cmd = rospy.Publisher("/ctrl_cmd", ControlCommandStamped, queue_size=10)
        self.pub_mass_vehicle = rospy.Publisher("/vehicle_mass", Float64, queue_size=10)
        self.sub_suspension = rospy.Subscriber("/suspension_feedback", SuspensionFeedback, self.cbSuspension, queue_size=10)
        self.sub_modelstates = rospy.Subscriber("/gazebo/model_states", ModelStates, self.cbModelStates, queue_size=10)
        self.vehicle_cmd = ControlCommandStamped()
        self.vehicle_cmd.cmd.linear_velocity = 0.0
        self.vehicle_cmd.header.frame_id = "base_link"
        self.msg_modelstate = None
        self.model_index = -1
        self.msg_mass = Float64()
        self.test_is_running = False
        self.robot_pose = Pose()
        self.set_model_service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)

    def runTests(self):
        mass = 1500
        low_mass = int(mass-mass*0.2)
        high_mass = int(mass+mass*0.2)
        inc_mass = int(mass*0.01)        
        self.vehicle_cmd.cmd.linear_velocity = 5.0
        
        p = Pose()
        p.orientation.w = 1.0
        import datetime
        start_time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M")
        newmodelstate = ModelState()
        os.makedirs("./data/"+start_time)
        newmodelstate.model_name = "nissanleaf"
        newmodelstate.pose.orientation.w = 1.0
        newmodelstate.pose.position.z = 0.282
        
        print("Starting at {0}".format(start_time))
        for m in range(low_mass, high_mass+inc_mass, inc_mass):
            s = self.reset_simulation()
            print("Setting mass: {0}".format(m))
            self.ofile = open("./data/{0}/{0}_{1}.csv".format(start_time, m), 'w')
            self.ofile.write("time;frontleft;frontright;rearleft;rearright\n")
            self.msg_mass.data = m
            self.vehicle_cmd.header.stamp = rospy.Time.now()
            
            self.pub_mass_vehicle.publish(self.msg_mass)
            self.set_model_service(newmodelstate)
            
            r = rospy.Rate(10)
            self.test_is_running = True
            while(self.test_is_running):
                self.pub_vehicle_cmd.publish(self.vehicle_cmd)
                r.sleep()
                if self.robot_pose.position.x > 52.0:
                    self.test_is_running = False
                if (rospy.is_shutdown()):
                    exit()
            self.ofile.close()

    

    def cbModelStates(self, data):
        if self.model_index == -1:
            for i,n in enumerate(data.name):
                if n=="nissanleaf":
                    self.model_index = i
        self.robot_pose = data.pose[self.model_index]
        
        

    def cbSuspension(self, data):
        self.susp_data = data.msg_data
        
        self.ofile.write("{0};{1};{2};{3};{4}\n".format(
            data.header.stamp.to_sec(), data.msg_data[0], data.msg_data[1], data.msg_data[2],data.msg_data[3]))
        
        

def main():
    rospy.init_node("suspension_data_collector")
    datacollector = SuspensionDataCollector()
    datacollector.runTests()


if __name__=="__main__":
    main()