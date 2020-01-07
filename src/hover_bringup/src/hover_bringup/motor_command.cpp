#include "hover_bringup/motor_command.h"

namespace hover_bringup
{

MotorCommand::MotorCommand()
{
  m_diff_drive = std::make_shared<diff_drive_controller::Odometry>(); // Rolling window size of 10 for velocity estimation
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
    case Config_Identifier::LED_L:
      val = m_led_l;
      break;
    case Config_Identifier::LED_R:
      val = m_led_r;
      break;
    case Config_Identifier::BACK_LED_L:
      val = m_back_led_l;
      break;
    case Config_Identifier::BACK_LED_R:
      val = m_back_led_r;
      break;
    case Config_Identifier::BUZZER:
      val = m_buzzer;
      break;
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

bool MotorCommand::setOutputCB(hover_bringup::SetOutputInt::Request  &req, hover_bringup::SetOutputIntResponse &res)
{
  if (req.output_name == "LED_L")
  {
    m_led_l = req.val;
  }
  else if (req.output_name == "BACK_LED_L")
  {
    m_back_led_l = req.val;
  }
  else if (req.output_name == "LED_R")
  {
    m_led_r = req.val;
  }
  else if (req.output_name == "BACK_LED_R")
  {
    m_back_led_r = req.val;
  }
  else if (req.output_name == "BUZZER")
  {
    m_buzzer = req.val;
  }
  else
  {
    res.success = false;
    return false;
  }
  res.success = true;
  return true;
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
  nr.param<std::string>("robot_frame", m_robot_frame, "base_footprint");

  // Prepare join state message
  m_js.name.push_back(m_left_wheel_name);
  m_js.name.push_back(m_right_wheel_name);
  m_js.position.resize(2);
  
  // Publisher
  m_joint_states_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
  m_left_motor_current_pub = nh.advertise<std_msgs::Float32>("BMS/left/I", 10);
  m_right_motor_current_pub = nh.advertise<std_msgs::Float32>("BMS/right/I", 10);
  m_bat_voltage_pub = nh.advertise<std_msgs::Float32>("BMS/U", 10);
  m_odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

  // Subscriber
  m_cmd_vel_sub = nh.subscribe("cmd_vel", 1, &MotorCommand::cmdVelCallback, this);
  m_diff_drive->init(ros::Time::now());
  m_diff_drive->setVelocityRollingWindowSize(10);
  m_diff_drive->setWheelParams(m_wheel_separation, m_wheel_radius, m_wheel_radius);

  // Service Server
  m_output_service = nh.advertiseService("set_output_int", &MotorCommand::setOutputCB, this);
  // Setup serial connection
  if ((m_fd = serialOpen("/dev/ttyAMA1", 38400)) < 0)
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
  m_dyn_reconfigure_server = std::make_shared<dynamic_reconfigure::Server<hover_bringup::MotorConfig>>();
  m_dyn_callback_type = boost::bind(&MotorCommand::dynReconfigureCallback, this, _1, _2);
  m_dyn_reconfigure_server->setCallback(m_dyn_callback_type);
}

void MotorCommand::dynReconfigureCallback(hover_bringup::MotorConfig &config, uint32_t level)
{   
    ROS_INFO("DYN_RECONFIGURE!");
    m_pid_p = config.PID_P;
    m_pid_i = config.PID_I;
    m_pid_d = config.PID_D;
}

void MotorCommand::getJointState(void)
{
    auto time = ros::Time::now();
    const int num_bytes = 15;
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
            float left_tick_speed, right_tick_speed;
            int32_t current_ticks_left = (int32_t)((buffer[1] << 24) | (buffer[2] << 16) | (buffer[3] << 8) | buffer[4]);
            int32_t current_ticks_right = (int32_t)((buffer[5] << 24) | (buffer[6] << 16) | (buffer[7] << 8) | buffer[8]);
            m_joint_pos_l = (double)(current_ticks_left) / m_left_wheel_ticks * 2 * M_PI;
            m_joint_pos_r = -(double)(current_ticks_right)  / m_right_wheel_ticks * 2 * M_PI;
            ROS_DEBUG_STREAM_THROTTLE(0.5, "Current Ticks: Left: " << current_ticks_left << " | Right: " << current_ticks_right);

            // Process additional Info 
            uint8_t identifier = buffer[9];
            int16_t value =  (buffer[10] << 8) | buffer[11];
            std_msgs::Float32 bat_info;
            switch(identifier)
            {
              case Additional_Info::BAT_U:
                m_bat_voltage = (float)value / 100;
                bat_info.data = m_bat_voltage;
                m_bat_voltage_pub.publish(bat_info);
                break;
              case Additional_Info::MOT_L_I:
                m_left_wheel_current = (float)value / 100;
                bat_info.data = m_left_wheel_current;
                m_left_motor_current_pub.publish(bat_info);
                break;
              case Additional_Info::MOT_R_I:
                m_right_wheel_current = (float)value / 100;
                bat_info.data = m_right_wheel_current;
                m_right_motor_current_pub.publish(bat_info);
                break;
              case Additional_Info::MOT_L_V:
                left_tick_speed = (float)value / 100;
                ROS_DEBUG_STREAM("Left Desired|Measured: " << m_speed_l << "|" << left_tick_speed);
                break;
              case Additional_Info::MOT_R_V:
                right_tick_speed = (float)value / 100;
                ROS_DEBUG_STREAM("Right Desired|Measured: " << m_speed_r << "|" << right_tick_speed);
                break;
              default:
                break;
            }
            // Publish and Update Joint State
            m_js.header.stamp = time;
            m_js.position[0] = m_joint_pos_l; 
            m_js.position[1] = m_joint_pos_r; 
            m_joint_states_pub.publish(m_js);
            m_diff_drive->update(m_joint_pos_l, m_joint_pos_r, time);
            // Compute and store orientation info
            const geometry_msgs::Quaternion orientation(
                  tf::createQuaternionMsgFromYaw(m_diff_drive->getHeading()));

            // Populate odom message and publish
            nav_msgs::Odometry odom_message;
            odom_message.header.frame_id = "odom";
            odom_message.child_frame_id = m_robot_frame;
            odom_message.header.stamp = time;
            odom_message.pose.pose.position.x = m_diff_drive->getX();
            odom_message.pose.pose.position.y = m_diff_drive->getY();
            odom_message.pose.pose.orientation = orientation;
            odom_message.twist.twist.linear.x  = m_diff_drive->getLinear();
            odom_message.twist.twist.angular.z = m_diff_drive->getAngular();
            m_odom_pub.publish(odom_message);
      
            // Publish tf /odom frame
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = m_robot_frame;
            odom_trans.transform.translation.x = m_diff_drive->getX();
            odom_trans.transform.translation.y = m_diff_drive->getY();
            odom_trans.transform.rotation = orientation;
            m_odom_broadcaster.sendTransform(odom_trans);
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
  m_speed_l = CLAMP((int32_t)(m_left_wheel_ticks *  (m_current_cmd_vel.linear.x - m_wheel_separation / 2 * m_current_cmd_vel.angular.z) / m_wheel_radius / 4 / M_PI), -1000, 1000); 
  m_speed_r = CLAMP((int32_t)(m_right_wheel_ticks * (m_current_cmd_vel.linear.x + m_wheel_separation / 2 * m_current_cmd_vel.angular.z) / m_wheel_radius / 4 / M_PI), -1000, 1000); 
  ROS_DEBUG_STREAM_THROTTLE(0.1, m_speed_l << " " << m_speed_r);
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
  
  byte1 |= (m_speed_l >> 8) & 0xFF;
  byte2 |= m_speed_l & 0xFF;

  byte3 |= (m_speed_r >> 8) & 0xFF;
  byte4 |= m_speed_r & 0xFF;
  
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