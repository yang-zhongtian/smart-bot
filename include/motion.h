#ifndef MOTION_H
#define MOTION_H

#include <Esp32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "servo.h"

struct CartesianVector
{
    float x;
    float y;
    float z;
};

struct FramePayload
{
    uint32_t frameSize;
    uint8_t *frameBufPtr;
};

class MotionController
{
public:
    SemaphoreHandle_t obstacleTriggerSemaphore;
    bool motorMonitorEnabled = false;
    JointServo *servo[12];

    MotionController();
    void setup(const int pins[4][3], const int offsets[4][3]);
    void init();
    int getOffset(int index);
    void setOffset(int index, int offset);
    void setSite(int leg, float x, float y, float z);
    float getMoveSpeed();
    void setMoveSpeed(float speed);
    void setAutoAvoidance(bool enable);
    void autoAvoidanceWorker();
    void reset();
    void raiseHead();
    void lowerHead();
    void sit();
    void stand();
    void bodyInit();
    void turnLeft(unsigned int step);
    void turnRight(unsigned int step);
    void stepForward(unsigned int step);
    void stepBackward(unsigned int step);
    void servoCelebration();
    void servoServe();

private:
    CartesianVector siteNow[4];
    CartesianVector siteExpect[4];
    CartesianVector tempSpeed[4];
    float moveSpeed = 1.4;
    bool autoAvoidanceEnabled = false;
    const float pi = 3.1415926F;
    volatile int restCounter = 0;
    float alpha, beta, gamma;

    void watchReach(int leg);
    void watchAllReach();
    void cartesianToPolar(float x, float y, float z);
    void polarToServo(int legServoIndex);
    void updateCoordinate(float &current, float target, float speed);
};

#endif
