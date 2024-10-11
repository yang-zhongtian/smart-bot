#ifndef SERVO_H
#define SERVO_H

#include <Esp32Servo.h>

struct ServoPayload
{
    uint8_t servoIndex;
    uint8_t angle;
};

class JointServo
{
public:
    JointServo();
    JointServo(QueueHandle_t servoQueue, int index, int pin, int offset);
    void setup(int angle);
    void write(int angle);
    int getOffset();
    void setOffset(int offset);
    int getPostureOffset();
    void setPostureOffset(int posture_offset);
    void increasePostureOffset(int offset);

private:
    QueueHandle_t servoQueue;
    int index;
    int pin;
    int offset;
    volatile int posture_offset;
    Servo servo;
    int servoAngle;
};

#endif