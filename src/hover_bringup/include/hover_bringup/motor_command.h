#pragma once
#include <wiringSerial.h>
#include <wiringPi.h>
#include <inttypes.h>
#include <ros/ros.h>

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
    void setSpeed(uint16_t data, float factor);

    //----------------------------------------------------------------------------
    // Sets the steering value
    //----------------------------------------------------------------------------
    void setSteer(uint16_t data);

    //----------------------------------------------------------------------------
    // Sends answer to master device
    //----------------------------------------------------------------------------
    void sendAnswer(void);

    //----------------------------------------------------------------------------
    // Sends debug infos
    //----------------------------------------------------------------------------
    void sendDebug();

    void checkForRequest();

private:
    void sendBuffer(uint8_t buffer[], uint8_t length);
    uint16_t calcCRC(uint8_t *ptr, int count);

    int32_t speedValue = 300;
    int32_t steerValue = 0;
    uint8_t upperLEDMaster = 0;
    uint8_t lowerLEDMaster = 0;
    uint8_t mosfetOutMaster = 0;
    uint8_t upperLEDSlave = 0;
    uint8_t lowerLEDSlave = 0;
    uint8_t mosfetOutSlave = 0;
    uint8_t beepsBackwards = 0;
    uint8_t activateWeakening = 0;
    int fd;
};

}