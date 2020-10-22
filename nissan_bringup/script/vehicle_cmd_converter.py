#!/usr/bin/env python
# Temporary fix: converting ctrl_cmd to cmd_vehicle
# TODO
"""
/ctrl_cmd
rosmsg show autoware_msgs/ControlCommandStamped
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
autoware_msgs/ControlCommand cmd
  float64 linear_velocity
  float64 linear_acceleration
  float64 steering_angle

/cmd_vehicle
rosmsg show szelectricity_msgs/VehicleVelocityControlCommand
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
bool set_autonomous_mode
float64 ref_vehicle_speed
float64 ref_steer_angle
"""

import rospy
import autoware_msgs.msg as autw_msg
import szelectricity_msgs.msg as szelec_msg

pub_cmd = None
def ctrCallBack(msg):
    global pub_cmd
    cmd_msg = szelec_msg.VehicleVelocityControlCommand()
    cmd_msg.ref_vehicle_speed = msg.cmd.linear_velocity # km/h
    cmd_msg.ref_steer_angle = msg.cmd.steering_angle
    cmd_msg.set_autonomous_mode = True
    cmd_msg.header.stamp = rospy.Time.now()
    #print(cmd_msg)
    if pub_cmd is not None:
        pub_cmd.publish(cmd_msg)

def listener():
    global pub_cmd
    rospy.init_node("vehicle_cmd_converter", anonymous=True)
    rospy.Subscriber("/ctrl_cmd", autw_msg.ControlCommandStamped, ctrCallBack)
    pub_cmd = rospy.Publisher("/cmd_vehicle", szelec_msg.VehicleVelocityControlCommand, queue_size=10)
    rospy.spin()

if __name__ == '__main__':
    listener()
