#include "bltbridge.h"

BltBridge::BltBridge()
{
}

void BltBridge::setData(const BltBridgeData data)
{
    this->data = data;
}

int BltBridge::getIntegerData(int index)
{
    return data.params[index].data.intValue;
}

float BltBridge::getFloatData(int index)
{
    return data.params[index].data.floatValue;
}

bool BltBridge::getBoolData(int index)
{
    return data.params[index].data.boolValue;
}