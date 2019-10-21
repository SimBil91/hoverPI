#include "hover_bringup/motor_command.h"

namespace hover_bringup
{

MotorCommand::MotorCommand()
{

}

void MotorCommand::cmdVelCallback(const geometry_msgs::TwistConstPtr& vel)
{
  m_current_cmd_vel = *vel;   
}

void MotorCommand::initMotorSerial(void)
{
  ros::NodeHandle nh;

  m_cmd_vel_sub = nh.subscribe("cmd_vel", 1, &MotorCommand::cmdVelCallback, this);
  // Setup serial connection
  if ((fd = serialOpen("/dev/ttyAMA1", 19200)) < 0)
  {
    ROS_ERROR_STREAM("Unable to open serial device.");
    return;
  }
  else
  {
    ROS_INFO_STREAM("Serial Port openend.");
  }

  if (wiringPiSetup () == -1)
  {
    ROS_ERROR_STREAM("Unable to start wiringPi.");
    return;
  }
  else
  {
    ROS_INFO_STREAM("Serial Communication Setup complete.");
  }
}

void MotorCommand::setSpeed(int32_t speedM, int32_t speedS)
{
  speedValueM = CLAMP(speedM, -1000, 1000); 
  speedValueS = CLAMP(speedS, -1000, 1000); 
}

void MotorCommand::checkForRequest(void)
{
    int count = 0;
    std::vector<char> bytes_received;
    while (serialDataAvail(fd))
    {
      char received_byte = serialGetchar(fd);
      ROS_INFO("%d -> %3d", count, received_byte);
      bytes_received.push_back(received_byte);
      if (bytes_received.size() == 2 && bytes_received[0] == '/' && bytes_received[1] == '\n')
      {
        ROS_INFO("New Command requested by firmware. Sending velocity commands.");
        //sendAnswer();
      }
      count++;
    }
    ROS_INFO("%d Bytes_____", count);
}

void MotorCommand::sendAnswer(void)
{
  // Set Speed according to current CmdVel
  int32_t speedL, speedR;
  speedL = m_current_cmd_vel.linear.x * 300;
  speedR = speedL;
  setSpeed(speedL, speedR);
  
  int index = 0;
  uint8_t buffer[8];
  uint8_t byte1 = 0;
  uint8_t byte2 = 0;
  uint8_t byte3 = 0;
  uint8_t byte4 = 0;
  
  uint8_t sendByte = 0;
  sendByte |= (activateWeakening << 7);
  sendByte |= (beepsBackwards << 6);
  sendByte |= (mosfetOutSlave << 5);
  sendByte |= (lowerLEDSlave << 4);
  sendByte |= (upperLEDSlave << 3);
  sendByte |= (mosfetOutMaster << 2);
  sendByte |= (lowerLEDMaster << 1);
  sendByte |= (upperLEDMaster << 0);
  
  uint16_t speedValueM_Format = (uint16_t)(speedValueM);
  byte1 |= (speedValueM_Format >> 8) & 0xFF;
  byte2 |= speedValueM_Format & 0xFF;

  uint16_t speedValueS_Format = (uint16_t)(speedValueS);
  byte3 |= (speedValueS_Format >> 8) & 0xFF;
  byte4 |= speedValueS_Format & 0xFF;
  
  // Send answer
  buffer[index++] = '/';
  buffer[index++] = byte1;
  buffer[index++] = byte2;
  buffer[index++] = byte3;
  buffer[index++] = byte4;
  //buffer[index++] = sendByte;

  // Calculate CRC
  uint16_t crc = calcCRC(buffer, index);
  buffer[index++] = (crc >> 8) & 0xFF;
  buffer[index++] = crc & 0xFF;

  // Stop byte
  buffer[index++] = '\n';
  
  sendBuffer(buffer, index);
}

uint16_t MotorCommand::calcCRC(uint8_t *ptr, int count)
{
  uint16_t  crc;
  uint8_t i;
  crc = 0;
  while (--count >= 0)
  {
    crc = crc ^ (uint16_t) *ptr++ << 8;
    i = 8;
    do
    {
      if (crc & 0x8000)
      {
        crc = crc << 1 ^ 0x1021;
      }
      else
      {
        crc = crc << 1;
      }
    } while(--i);
  }
  return (crc);
}

void MotorCommand::sendBuffer(uint8_t buffer[], uint8_t length)
{
  uint8_t index = 0;
  auto written = write(fd, buffer, length);
  //ROS_INFO_STREAM(written);
}

void MotorCommand::sendDebug()
{

}

}