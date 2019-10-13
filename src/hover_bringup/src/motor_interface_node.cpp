#include "ros/ros.h"
#include <wiringSerial.h>
#include <wiringPi.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_interface_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  // Setup serial connection
  int fd ;
  int count ;
  unsigned int nextTime ;

  if ((fd = serialOpen("/dev/ttyS0", 19200)) < 0)
  {
    ROS_ERROR_STREAM("Unable to open serial device.");
    return 1;
  }
  else
  {
    ROS_INFO_STREAM("Serial Port openend.");
  }

  if (wiringPiSetup () == -1)
  {
    ROS_ERROR_STREAM("Unable to start wiringPi.");
    return 1 ;
  }
  else
  {
    ROS_INFO_STREAM("Serial Communication Setup complete.");
  }

  while (ros::ok())
  {
    ros::spinOnce();
    while (serialDataAvail (fd))
    {
      ROS_INFO(" -> %3d", serialGetchar (fd)) ;
    }
    loop_rate.sleep();
  }
  return 0;
}