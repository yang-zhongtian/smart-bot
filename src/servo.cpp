#include "servo.h"
#include "constants.h"

JointServo::JointServo()
{
}

JointServo::JointServo(QueueHandle_t servoQueue, int index, int pin, int offset)
{
    this->servoQueue = servoQueue;
    this->index = index;
    this->pin = pin;
    this->offset = offset;
    this->posture_offset = 0;
}

void JointServo::setup(int angle)
{
    this->servo.attach(this->pin, 500, 2500);
    this->servo.write(angle + this->offset);
    delay(20);
}

void JointServo::write(int angle)
{
    this->servo.write(angle + this->offset);
    if (this->servoAngle == angle)
        return;
    ServoPayload data;
    data.servoIndex = this->index;
    data.angle = angle;
    xQueueSend(this->servoQueue, &data, 0);
    this->servoAngle = angle;
}

int JointServo::getOffset()
{
    return this->offset;
}

void JointServo::setOffset(int offset)
{
    this->offset = offset;
    this->servo.write(90 + offset);
}

int JointServo::getPostureOffset()
{
    return this->posture_offset;
}

void JointServo::setPostureOffset(int posture_offset)
{
    this->posture_offset = posture_offset;
}

void JointServo::increasePostureOffset(int offset)
{
    this->posture_offset += offset;
}