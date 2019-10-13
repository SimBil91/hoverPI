#include "ros/ros.h"
#include <wiringSerial.h>
#include <wiringPi.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motor_interface_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(100);
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
    int count = 0;
    std::vector<char> bytes_received;
    while (serialDataAvail (fd))
    {
      char received_byte = serialGetchar(fd);
      ROS_DEBUG("%d -> %3d", count, received_byte);
      bytes_received.push_back(received_byte);
      if (bytes_received.size() == 2 && bytes_received[0] == '/' && bytes_received[1] == '\n')
      {
        ROS_INFO("New Command requested by firmware.");
      }
      count++;
    }
    ROS_DEBUG("_____");
    loop_rate.sleep();
  }
  return 0;
}