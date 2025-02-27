#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <WiFi.h>
#include "bltbridge.h"
#include "motion.h"

class TCPController
{
public:
    TCPController();
    void setup(MotionController &motionController, TaskHandle_t *task4Handle);
    void begin(const char *ssid, const char *password);
    void receive();
    void sendServoAngle();
    void sendTriggerObstacle();

private:
    WiFiServer *server;
    WiFiClient client;
    MotionController *motionController;
    BltBridge bltBridge;
    TaskHandle_t *task4Handle = NULL;
    SemaphoreHandle_t xSemaphore;

    void processReceivedData(const BltBridgeData &data);
};

#endif
