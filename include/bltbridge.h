#ifndef BLTBRIDGE_H
#define BLTBRIDGE_H

#include <cstring>
#include "constants.h"

#define MAX_DATA_COUNT 10
#define MAX_DATA_SIZE 256

enum BltBridgeOpTypes : size_t
{
    BLT_BRIDGE_OP_SET_OFFSET,
    BLT_BRIDGE_OP_SET_AUTO_AVOIDANCE,
    BLT_BRIDGE_OP_SET_MOTOR_MONITOR,
    BLT_BRIDGE_OP_FORWARD,
    BLT_BRIDGE_OP_BACKWARD,
    BLT_BRIDGE_OP_CONTINUE,
};

enum BltHostOpTypes : size_t
{
    BLT_HOST_SERVO_ANGLE,
    BLT_HOST_TRIGGER_OBSTACLE,
};

enum BltBridgeDTypes : size_t
{
    BLT_BRIDGE_DTYPE_INT,
    BLT_BRIDGE_DTYPE_FLOAT,
    BLT_BRIDGE_DTYPE_BOOL,
};

struct BltBridgeParams
{
    BltBridgeDTypes dtype;

    union
    {
        int32_t intValue;
        float floatValue;
        bool boolValue;
        uint8_t bytesValue[MAX_DATA_SIZE];
    } data;
};

struct BltBridgeData
{
    BltBridgeOpTypes opType;
    std::size_t dCount;
    BltBridgeParams params[MAX_DATA_COUNT];
};

class BltBridge
{
public:
    BltBridge();
    void setData(const BltBridgeData data);
    int getIntegerData(int index);
    float getFloatData(int index);
    bool getBoolData(int index);

private:
    BltBridgeData data;
};

#endif
