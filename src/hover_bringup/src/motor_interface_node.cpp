#include "ros/ros.h"

#include "hover_bringup/motor_command.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_interface_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  auto motors = hover_bringup::MotorCommand();
  motors.init();
  while (ros::ok())
  {
    motors.sendJointCommands();
    motors.getJointState();
    motors.setLEDCmdVel();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}