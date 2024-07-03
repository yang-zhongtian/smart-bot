#include "bluetooth.h"

BluetoothSerialController::BluetoothSerialController()
{
}

void BluetoothSerialController::setup(MotionController &motionController)
{
    this->motionController = &motionController;
}

void BluetoothSerialController::begin()
{
    Serial.println("BluetoothSerialController::begin");
    btSerial.begin("ESP32-Bluetooth");
}

void BluetoothSerialController::receive()
{
    if (btSerial.available())
    {
        BltBridgeData receivedData;
        btSerial.readBytes(reinterpret_cast<char *>(&receivedData.opType), sizeof(receivedData.opType));
        btSerial.readBytes(reinterpret_cast<char *>(&receivedData.dCount), sizeof(receivedData.dCount));

        for (int i = 0; i < receivedData.dCount; i++)
        {

            btSerial.readBytes(reinterpret_cast<char *>(&receivedData.params[i].dtype), sizeof(receivedData.params[i].dtype));

            switch (receivedData.params[i].dtype)
            {
            case BLT_BRIDGE_DTYPE_INT:
                btSerial.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.intValue), sizeof(receivedData.params[i].data.intValue));
                break;
            case BLT_BRIDGE_DTYPE_FLOAT:
                btSerial.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.floatValue), sizeof(receivedData.params[i].data.floatValue));
                break;
            case BLT_BRIDGE_DTYPE_BOOL:
                btSerial.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.boolValue), sizeof(receivedData.params[i].data.boolValue));
                break;
            default:
                return;
            }
        }

        for (int i = 0; i < receivedData.dCount; i++)
        {
            switch (receivedData.params[i].dtype)
            {
            case BLT_BRIDGE_DTYPE_INT:
                Serial.printf("BluetoothSerialController::receive: %d\n", receivedData.params[i].data.intValue);
                break;
            case BLT_BRIDGE_DTYPE_FLOAT:
                Serial.printf("BluetoothSerialController::receive: %f\n", receivedData.params[i].data.floatValue);
                break;
            case BLT_BRIDGE_DTYPE_BOOL:
                Serial.printf("BluetoothSerialController::receive: %d\n", receivedData.params[i].data.boolValue);
                break;
            }
        }
        processReceivedData(receivedData);
    }
}

void BluetoothSerialController::processReceivedData(const BltBridgeData &data)
{
    int index, value;
    bool enable;
    BltBridgeParams response;

    bltBridge.setData(data);
    switch (data.opType)
    {
    case BLT_BRIDGE_OP_GET_OFFSET:
        index = bltBridge.getIntegerData(0);
        value = motionController->getOffset(index);
        response.dtype = BLT_BRIDGE_DTYPE_INT;
        response.data.intValue = value;
        sendResponse(response);
        break;
    case BLT_BRIDGE_OP_SET_OFFSET:
        index = bltBridge.getIntegerData(0);
        value = bltBridge.getIntegerData(1);
        Serial.printf("BluetoothSerialController::processReceivedData: index=%d, value=%d\n", index, value);
        motionController->setOffset(index, value);
        break;
    case BLT_BRIDGE_OP_SET_AUTO_AVOIDANCE:
        enable = bltBridge.getBoolData(0);
        motionController->setAutoAvoidance(enable);
        break;
    case BLT_BRIDGE_OP_FORWARD:
        value = bltBridge.getIntegerData(0);
        motionController->stepForward(value);
        break;
    case BLT_BRIDGE_OP_BACKWARD:
        value = bltBridge.getIntegerData(0);
        motionController->stepBackward(value);
        break;
    }
}

void BluetoothSerialController::sendResponse(const BltBridgeParams &response)
{
    btSerial.write(reinterpret_cast<const uint8_t *>(&response.dtype), sizeof(response.dtype));
    switch (response.dtype)
    {
    case BLT_BRIDGE_DTYPE_INT:
        btSerial.write(reinterpret_cast<const uint8_t *>(&response.data.intValue), sizeof(response.data.intValue));
        break;
    case BLT_BRIDGE_DTYPE_FLOAT:
        btSerial.write(reinterpret_cast<const uint8_t *>(&response.data.floatValue), sizeof(response.data.floatValue));
        break;
    case BLT_BRIDGE_DTYPE_BOOL:
        btSerial.write(reinterpret_cast<const uint8_t *>(&response.data.boolValue), sizeof(response.data.boolValue));
        break;
    }
}