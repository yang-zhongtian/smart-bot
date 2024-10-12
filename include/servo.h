#ifndef SERVO_H
#define SERVO_H

#include <Esp32Servo.h>

class JointServo
{
public:
    int servoAngle;
    JointServo(int index, int pin, int offset);
    void setup(int angle);
    void write(int angle);
    int getOffset();
    void setOffset(int offset);
    int getPostureOffset();
    void setPostureOffset(int posture_offset);
    void increasePostureOffset(int offset);

private:
    int index;
    int pin;
    int offset;
    volatile int posture_offset;
    Servo servo;
};

#endif