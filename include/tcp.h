#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <WiFi.h>
#include "bltbridge.h"
#include "motion.h"

class TCPController
{
public:
    TCPController();
    void setup(MotionController &motionController);
    void begin(const char *ssid, const char *password);
    void receive();

private:
    WiFiServer *server;
    WiFiClient client;
    MotionController *motionController;
    BltBridge bltBridge;

    void processReceivedData(const BltBridgeData &data);
    void sendResponse(const BltBridgeParams &response);
};

#endif
