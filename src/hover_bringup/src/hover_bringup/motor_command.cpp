#include "hover_bringup/motor_command.h"

namespace hover_bringup
{

MotorCommand::MotorCommand()
{
  m_diff_drive = std::make_shared<diff_drive_controller::DiffDriveController>();
}

void MotorCommand::cmdVelCallback(const geometry_msgs::TwistConstPtr& vel)
{
  m_current_cmd_vel = *vel;   
}

std::vector<uint8_t> MotorCommand::getConfigCyclic()
{
  std::vector<uint8_t> output;
  uint8_t identifier = m_current_config_identifier;
  output.push_back(identifier);
  int16_t val = 0;
  // Fill in value based on identifier
  switch(identifier)
  {
    case Config_Identifier::PID_P:
      val = m_pid_p * 100;
    break;
    case Config_Identifier::PID_I:
      val = m_pid_i * 100;
    break;
    case Config_Identifier::PID_D:
      val = m_pid_d * 100;
    break;
  }
  uint8_t first_byte = ((val >> 8) & 0xFF);
  uint8_t second_byte = val & 0xFF;
  output.push_back(first_byte);
  output.push_back(second_byte);
  m_current_config_identifier++;
  if (m_current_config_identifier == Config_Identifier::NUM_ENTRIES)
  {
    // Wrap loop
    m_current_config_identifier = 0;
  }
  return output;
}

void MotorCommand::init(void)
{
  ros::NodeHandle nh;
  ros::NodeHandle nr("~");
  // Load Params
  nr.param<double>("wheel_separation", m_wheel_separation, 0.5);
  nr.param<double>("wheel_radius", m_wheel_radius, 0.05);
  nr.param<std::string>("left_wheel", m_left_wheel_name, "left_wheel");
  nr.param<std::string>("right_wheel", m_right_wheel_name, "right_wheel");
  nr.param<double>("right_wheel_ticks", m_right_wheel_ticks, 30.0);
  nr.param<double>("left_wheel_ticks", m_left_wheel_ticks, 30.0);

  // Prepare join state message
  m_js.name.push_back(m_left_wheel_name);
  m_js.name.push_back(m_right_wheel_name);
  m_js.position.resize(2);
  
  // Publisher
  m_joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  m_bat_current_pub = nh.advertise<std_msgs::Float32>("BMS/I", 10);
  m_bat_voltage_pub = nh.advertise<std_msgs::Float32>("BMS/U", 10);
  // Subscriber
  m_cmd_vel_sub = nh.subscribe("cmd_vel", 1, &MotorCommand::cmdVelCallback, this);

  // Create joint state handles
  hardware_interface::JointStateHandle state_handle_left_wheel(m_left_wheel_name, &m_joint_pos_l, &m_joint_vel_l, &m_joint_eff_l);
  hardware_interface::JointHandle handle_left_wheel(state_handle_left_wheel, &m_cmd_vel_l);
  m_hw.registerHandle(handle_left_wheel);
  hardware_interface::JointStateHandle state_handle_right_wheel(m_right_wheel_name, &m_joint_pos_r, &m_joint_vel_r, &m_joint_eff_r);
  hardware_interface::JointHandle handle_right_wheel(state_handle_right_wheel, &m_cmd_vel_r);
  m_hw.registerHandle(handle_right_wheel);
  m_diff_drive->init(&m_hw, nh, nr);

  // Setup serial connection
  if ((m_fd = serialOpen("/dev/ttyAMA1", 19200)) < 0)
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
  //m_dyn_reconfigure_server = std::make_shared<dynamic_reconfigure::Server<hover_bringup::MotorConfig>>();
  // Dynamic Reconfigure
  //*m_dyn_callback_type = boost::bind(&MotorCommand::dynReconfigureCallback, this, _1, _2);
  //m_dyn_reconfigure_server->setCallback(*m_dyn_callback_type);
}

void MotorCommand::dynReconfigureCallback(hover_bringup::MotorConfig &config, uint32_t level)
{   
    m_pid_p = config.PID_P;
    m_pid_i = config.PID_I;
    m_pid_d = config.PID_D;
}

void MotorCommand::setSpeed(int32_t speedM, int32_t speedS)
{
  speedValueM = CLAMP(speedM, -1000, 1000); 
  speedValueS = CLAMP(speedS, -1000, 1000); 
}

void MotorCommand::getJointState(void)
{
    const int num_bytes = 16;
    int count = 0;
    // Check Crc
    uint8_t buffer[num_bytes];
    while (serialDataAvail(m_fd) && count < num_bytes)
    {
      uint8_t received_byte = serialGetchar(m_fd);
      if (received_byte  == '/')
      {
        count = 0;
      }
      buffer[count] = received_byte;
      count++;
      if (count == num_bytes)
      {
        if (received_byte == '\n')
        {
          // Calc crc
          uint16_t crc = calcCRC(buffer, num_bytes - 3);
          if ( buffer[num_bytes - 3] == ((crc >> 8) & 0xFF) && buffer[num_bytes - 2] == (crc & 0xFF))
          {
            m_joint_pos_l = -(double)((int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4])) / m_left_wheel_ticks * 2 * M_PI;
            m_joint_pos_r = (double)((int32_t)((buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8]))  / m_right_wheel_ticks * 2 * M_PI;
            m_bat_current = (float)((int16_t)((buffer[9] << 8) | buffer[10])) / 100.0;
            m_bat_voltage = (float)((int16_t)((buffer[11] << 8) | buffer[12])) / 100.0;
            std_msgs::Float32 bat_info;
            bat_info.data = m_bat_current;
            m_bat_current_pub.publish(bat_info);
            bat_info.data = m_bat_voltage;
            m_bat_voltage_pub.publish(bat_info);
            // Publish and Update Joint State
            m_js.header.stamp = ros::Time::now();
            m_js.position[0] = m_joint_pos_l; 
            m_js.position[1] = m_joint_pos_r; 
            m_joint_states_pub.publish(m_js);
            ROS_DEBUG_STREAM("Got new Joint States. M: " << m_joint_pos_l << " rad. S: " << m_joint_pos_r << " rad");
            m_diff_drive->update(ros::Time::now(), ros::Duration(0));
          }
        }
        else
        {
          ROS_WARN_THROTTLE(1.0, "Received too many bytes");
        }
      }
    }
}

void MotorCommand::sendJointCommands(void)
{
  // Set Speed according to current CmdVel
  double v_l, v_r;
  int32_t speedL, speedR;
  speedL = m_left_wheel_ticks *  (m_current_cmd_vel.linear.x - m_wheel_separation / 2 * m_current_cmd_vel.angular.z) / m_wheel_radius / 2;
  speedR = m_right_wheel_ticks * (m_current_cmd_vel.linear.x + m_wheel_separation / 2 * m_current_cmd_vel.angular.z) / m_wheel_radius / 2;
  setSpeed(speedL, speedR);
  ROS_DEBUG_STREAM_THROTTLE(0.1, speedL << " " << speedR);
  int index = 0;
  uint8_t buffer[11];
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
  auto config = getConfigCyclic();
  buffer[index++] = config[0];
  buffer[index++] = config[1];
  buffer[index++] = config[2];

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
  auto written = write(m_fd, buffer, length);
}

}