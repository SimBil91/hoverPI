#pragma once
#include <wiringSerial.h>
#include <wiringPi.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diff_drive_controller/diff_drive_controller.h>

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
    void initMotorSerial();

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
    void checkForRequest();

    void cmdVelCallback(const geometry_msgs::TwistConstPtr&  vel);

private:
    void sendBuffer(uint8_t buffer[], uint8_t length);
    uint16_t calcCRC(uint8_t *ptr, int count);

    int32_t speedValueM = 0;
    int32_t speedValueS = 0;
    uint8_t upperLEDMaster = 0;
    uint8_t lowerLEDMaster = 0;
    uint8_t mosfetOutMaster = 0;
    uint8_t upperLEDSlave = 0;
    uint8_t lowerLEDSlave = 0;
    uint8_t mosfetOutSlave = 0;
    uint8_t beepsBackwards = 0;
    uint8_t activateWeakening = 0;
    int fd;
    ros::Subscriber m_cmd_vel_sub;
    geometry_msgs::Twist m_current_cmd_vel;
    double m_wheel_radius;
    double m_wheel_separation;
    std::shared_ptr<diff_drive_controller::DiffDriveController> m_diff_drive;
};

}