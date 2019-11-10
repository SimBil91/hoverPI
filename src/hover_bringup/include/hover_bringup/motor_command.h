#pragma once
#include <wiringSerial.h>
#include <wiringPi.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diff_drive_controller/diff_drive_controller.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>

#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAP(x, xMin, xMax, yMin, yMax) ((x - xMin) * (yMax - yMin) / (xMax - xMin) + yMin)

namespace hover_bringup
{
class MotorCommand
{
    public:
    MotorCommand();
    //----------------------------------------------------------------------------
    // Initializes the steering serial
    //----------------------------------------------------------------------------
    void init();

    //----------------------------------------------------------------------------
    // Sets the speed value
    //----------------------------------------------------------------------------
    void setSpeed(int32_t speedM, int32_t speedS);

    //----------------------------------------------------------------------------
    // Sets the steering value
    //----------------------------------------------------------------------------
    void setSteer(uint16_t data);

    //----------------------------------------------------------------------------
    // Sends answer to master device
    //----------------------------------------------------------------------------
    void sendJointCommands(void);

    //----------------------------------------------------------------------------
    // Sends debug infos
    //----------------------------------------------------------------------------
    void getJointState();

    void cmdVelCallback(const geometry_msgs::TwistConstPtr&  vel);

private:
    void sendBuffer(uint8_t buffer[], uint8_t length);
    uint16_t calcCRC(uint8_t *ptr, int count);

    int32_t speedValueM = 0;
    int32_t speedValueS = 0;
    double m_cmd_vel_l = 0;
    double m_cmd_vel_r = 0;
    double m_left_wheel_ticks;
    double m_right_wheel_ticks;

    // Joint State
    double m_joint_pos_l = 0;
    double m_joint_pos_r = 0;
    double m_joint_vel_l = 0;
    double m_joint_vel_r = 0;
    double m_joint_eff_l = 0;
    double m_joint_eff_r = 0;    
    // BMS INFO
    float m_bat_current = 0;
    float m_bat_voltage = 0;
    // LED stuff
    uint8_t upperLEDMaster = 0;
    uint8_t lowerLEDMaster = 0;
    uint8_t mosfetOutMaster = 0;
    uint8_t upperLEDSlave = 0;
    uint8_t lowerLEDSlave = 0;
    uint8_t mosfetOutSlave = 0;
    uint8_t beepsBackwards = 0;
    uint8_t activateWeakening = 0;
    int m_fd;
    // Subscriber
    ros::Subscriber m_cmd_vel_sub;
    // Publisher
    ros::Publisher m_joint_states_pub;
    ros::Publisher m_bat_current_pub;
    ros::Publisher m_bat_voltage_pub;

    sensor_msgs::JointState m_js;
    geometry_msgs::Twist m_current_cmd_vel;
    double m_wheel_radius;
    double m_wheel_separation;
    std::string m_left_wheel_name;
    std::string m_right_wheel_name;
    hardware_interface::VelocityJointInterface m_hw;
    std::shared_ptr<diff_drive_controller::DiffDriveController> m_diff_drive;
};

}