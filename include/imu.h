#ifndef IMU_H
#define IMU_H

#include <cstdint>
#include "MPU6050_6Axis_MotionApps20.h"

class IMU
{
public:
    void setup(uint8_t pin);
    void update();

    float getYaw();
    float getPitch();
    float getRoll();

    int16_t getTemperature();

private:
    MPU6050 mpu;
    float euler[3]; // [psi, theta, phi]    Euler angle container
    float ypr[3];   // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
    int16_t temperature;

    // MPU control/status vars
    bool dmpReady = false; // Set true if DMP init was successful
};

#endif