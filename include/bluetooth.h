#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include <BluetoothSerial.h>
#include "bltbridge.h"
#include "motion.h"

class BluetoothSerialController
{
public:
    BluetoothSerialController();
    void setup(MotionController &motionController);
    void begin();
    void receive();

private:
    BluetoothSerial btSerial;
    MotionController *motionController;
    BltBridge bltBridge;

    void processReceivedData(const BltBridgeData &data);
    void sendResponse(const BltBridgeParams &response);
};

#endif
