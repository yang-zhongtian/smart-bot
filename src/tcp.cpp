#include "tcp.h"

TCPController::TCPController()
{
}

void TCPController::setup(MotionController &motionController, TaskHandle_t *task4Handle)
{
    this->server = new WiFiServer(TCP_SERVER_PORT);
    this->motionController = &motionController;
    this->task4Handle = task4Handle;
}

void TCPController::begin(const char *ssid, const char *password)
{
    WiFi.mode(WIFI_STA);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    this->server->begin();
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

    bltBridge.setData(data);
    switch (data.opType)
    {
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
    case BLT_BRIDGE_OP_SET_MOTOR_MONITOR:
        enable = bltBridge.getBoolData(0);
        motionController->motorMonitorEnabled = enable;
        break;
    case BLT_BRIDGE_OP_FORWARD:
        value = bltBridge.getIntegerData(0);
        motionController->stepForward(value);
        break;
    case BLT_BRIDGE_OP_BACKWARD:
        value = bltBridge.getIntegerData(0);
        motionController->stepBackward(value);
        break;
    case BLT_BRIDGE_OP_CONTINUE:
        if (task4Handle)
            xTaskNotifyGive(*task4Handle);
        break;
    }
}

void TCPController::sendServoAngle()
{
    ServoPayload data;
    if (xQueueReceive(motionController->servoQueue, &data, portMAX_DELAY) != pdPASS)
    {
        vTaskDelay(20);
        return;
    }
    if (!motionController->motorMonitorEnabled)
        return;
    BltHostOpTypes opType = BLT_HOST_SERVO_ANGLE;
    client.write(reinterpret_cast<const uint8_t *>(&opType), sizeof(opType));
    client.write(reinterpret_cast<const uint8_t *>(&data.servoIndex), sizeof(data.servoIndex));
    client.write(reinterpret_cast<const uint8_t *>(&data.angle), sizeof(data.angle));
}

void TCPController::sendTriggerObstacle()
{
    BltHostOpTypes opType = BLT_HOST_TRIGGER_OBSTACLE;
    client.write(reinterpret_cast<const uint8_t *>(&opType), sizeof(opType));
}