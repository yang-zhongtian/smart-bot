#include "tcp.h"

TCPController::TCPController()
{
}

void TCPController::setup(MotionController &motionController)
{
    this->server = new WiFiServer(TCP_SERVER_PORT);
    this->motionController = &motionController;
}

void TCPController::begin(const char *ssid, const char *password)
{
    Serial.println("Setting up WiFi SoftAP...");
    WiFi.softAP(ssid, password);

    IPAddress IP = WiFi.softAPIP();
    Serial.print("SoftAP IP address: ");
    Serial.println(IP);

    server->begin();
}

void TCPController::receive()
{
    if (!client || !client.connected())
    {
        client = server->available();
    }

    if (client && client.connected() && client.available())
    {
        BltBridgeData receivedData;
        client.readBytes(reinterpret_cast<char *>(&receivedData.opType), sizeof(receivedData.opType));
        client.readBytes(reinterpret_cast<char *>(&receivedData.dCount), sizeof(receivedData.dCount));

        for (int i = 0; i < receivedData.dCount; i++)
        {

            client.readBytes(reinterpret_cast<char *>(&receivedData.params[i].dtype), sizeof(receivedData.params[i].dtype));

            switch (receivedData.params[i].dtype)
            {
            case BLT_BRIDGE_DTYPE_INT:
                client.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.intValue), sizeof(receivedData.params[i].data.intValue));
                break;
            case BLT_BRIDGE_DTYPE_FLOAT:
                client.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.floatValue), sizeof(receivedData.params[i].data.floatValue));
                break;
            case BLT_BRIDGE_DTYPE_BOOL:
                client.readBytes(reinterpret_cast<char *>(&receivedData.params[i].data.boolValue), sizeof(receivedData.params[i].data.boolValue));
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

void TCPController::processReceivedData(const BltBridgeData &data)
{
    int index, value;
    bool enable;
    FramePayload payload;
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
    case BLT_BRIDGE_OP_TAKE_PICTURE:
        payload = motionController->takePicture();
        response.dtype = BLT_BRIDGE_DTYPE_INT;
        response.data.intValue = payload.frameSize;
        sendResponse(response);
        client.write(payload.frameBufPtr, payload.frameSize);
    }
}

void TCPController::sendResponse(const BltBridgeParams &response)
{
    client.write(reinterpret_cast<const uint8_t *>(&response.dtype), sizeof(response.dtype));
    switch (response.dtype)
    {
    case BLT_BRIDGE_DTYPE_INT:
        client.write(reinterpret_cast<const uint8_t *>(&response.data.intValue), sizeof(response.data.intValue));
        break;
    case BLT_BRIDGE_DTYPE_FLOAT:
        client.write(reinterpret_cast<const uint8_t *>(&response.data.floatValue), sizeof(response.data.floatValue));
        break;
    case BLT_BRIDGE_DTYPE_BOOL:
        client.write(reinterpret_cast<const uint8_t *>(&response.data.boolValue), sizeof(response.data.boolValue));
        break;
    }
}