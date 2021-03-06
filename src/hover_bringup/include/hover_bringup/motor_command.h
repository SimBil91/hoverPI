#pragma once
#include <wiringSerial.h>
#include <wiringPi.h>
#include <inttypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <diff_drive_controller/odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <dynamic_reconfigure/server.h>
#include <hover_bringup/MotorConfig.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <hover_bringup/SetOutputInts.h>

#define ABS(a) (((a) < 0.0) ? -(a) : (a))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define MAP(x, xMin, xMax, yMin, yMax) ((x - xMin) * (yMax - yMin) / (xMax - xMin) + yMin)

namespace hover_bringup
{
class MotorCommand
{
    public:
    enum Config_Identifier {PID_P, PID_I, PID_D, ALL_LEDS, ENABLE_MOTORS,BUZZER, NUM_ENTRIES, LED_L, LED_R, BACK_LED_L, BACK_LED_R, };
    enum Additional_Info {BAT_U, MOT_L_I, MOT_R_I, MOT_L_V, MOT_R_V};

    MotorCommand();
    //----------------------------------------------------------------------------
    // Initializes the steering serial
    //----------------------------------------------------------------------------
    void init();

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

    std::vector<uint8_t> getConfigCyclic();

    void dynReconfigureCallback(hover_bringup::MotorConfig &config, uint32_t level);
    
    void setLEDCmdVel();


private:
    void sendBuffer(uint8_t buffer[], uint8_t length);
    uint16_t calcCRC(uint8_t *ptr, int count);

    bool setOutputCB(hover_bringup::SetOutputInts::Request &req, hover_bringup::SetOutputIntsResponse &res);

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
    float m_left_wheel_current = 0;
    float m_right_wheel_current = 0;

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
    ros::Publisher m_left_motor_current_pub;
    ros::Publisher m_right_motor_current_pub;
    ros::Publisher m_bat_voltage_pub;
    ros::Publisher m_odom_pub;
    ros::Publisher m_button_pub;
    tf::TransformBroadcaster m_odom_broadcaster;

    sensor_msgs::JointState m_js;
    geometry_msgs::Twist m_current_cmd_vel;
    double m_wheel_radius;
    double m_wheel_separation;
    std::string m_left_wheel_name;
    std::string m_right_wheel_name;
    std::shared_ptr<diff_drive_controller::Odometry> m_diff_drive;
    
    // Config Vals
    int m_current_config_identifier = 0;
    float m_pid_p = 4;
    float m_pid_d = 0;
    float m_pid_i = 0;
    int16_t m_led_l = 0;
    ros::Time m_led_l_toggle_time;
    int16_t m_led_r = 0;
    ros::Time m_led_r_toggle_time;
    int16_t m_back_led_l = 0;
    int16_t m_back_led_r = 0;
    int16_t m_buzzer = 0;
    int32_t m_speed_l = 0;
    int32_t m_speed_r = 0;
    int16_t m_enable_motors = 1;
    int m_button_pin;
    ros::ServiceServer m_output_service;
    std::string m_robot_frame;
    // Dyn reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<hover_bringup::MotorConfig> > m_dyn_reconfigure_server;
    dynamic_reconfigure::Server<hover_bringup::MotorConfig>::CallbackType m_dyn_callback_type;
    };

}