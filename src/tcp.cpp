#include "tcp.h"

TCPController::TCPController()
{
}

void TCPController::setup(MotionController &motionController, TaskHandle_t *task4Handle)
{
    this->server = new WiFiServer(TCP_SERVER_PORT);
    this->motionController = &motionController;
    this->task4Handle = task4Handle;
    this->xSemaphore = xSemaphoreCreateMutex();
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

        // for (int i = 0; i < receivedData.dCount; i++)
        // {
        //     switch (receivedData.params[i].dtype)
        //     {
        //     case BLT_BRIDGE_DTYPE_INT:
        //         Serial.printf("BluetoothSerialController::receive: %d\n", receivedData.params[i].data.intValue);
        //         break;
        //     case BLT_BRIDGE_DTYPE_FLOAT:
        //         Serial.printf("BluetoothSerialController::receive: %f\n", receivedData.params[i].data.floatValue);
        //         break;
        //     case BLT_BRIDGE_DTYPE_BOOL:
        //         Serial.printf("BluetoothSerialController::receive: %d\n", receivedData.params[i].data.boolValue);
        //         break;
        //     }
        // }
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
    case BLT_BRIDGE_OP_TURN_LEFT:
        value = bltBridge.getIntegerData(0);
        motionController->turnLeft(value);
        break;
    case BLT_BRIDGE_OP_TURN_RIGHT:
        value = bltBridge.getIntegerData(0);
        motionController->turnRight(value);
        break;
    case BLT_BRIDGE_OP_STAND:
        motionController->reset();
        break;
    case BLT_BRIDGE_OP_CONTINUE:
        if (task4Handle)
            xTaskNotifyGive(*task4Handle);
        break;
    }
}

void TCPController::sendServoAngle()
{
    if (!motionController->motorMonitorEnabled)
        return;
    BltHostOpTypes opType = BLT_HOST_SERVO_ANGLE;
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
    {
        client.write(reinterpret_cast<const uint8_t *>(&opType), sizeof(opType));
        for (int i = 0; i < 12; i++)
        {
            client.write(reinterpret_cast<const uint8_t *>(&motionController->servo[i]->servoAngle), sizeof(int));
        }
        xSemaphoreGive(xSemaphore);
    }
}

void TCPController::sendTriggerObstacle()
{
    BltHostOpTypes opType = BLT_HOST_TRIGGER_OBSTACLE;
    if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
    {
        client.write(reinterpret_cast<const uint8_t *>(&opType), sizeof(opType));
        xSemaphoreGive(xSemaphore);
    }
}