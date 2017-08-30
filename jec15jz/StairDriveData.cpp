#include "StairDriveData.h"

StairDriveData::StairDriveData(int stairType, int termsValue, int forward, int tailValue, int target, ledcolor_t ledColor)
{
    this->stairType = stairType;
    this->termsValue = termsValue;
    this->forward = forward;
    this->tailValue = tailValue;
    this->target = target;
    this->ledColor = ledColor;
}

StairDriveData::~StairDriveData()
{

}

int StairDriveData::getStairType()
{
    return stairType;
}

int StairDriveData::getTermsValue()
{
    return termsValue;
}

int StairDriveData::getForward()
{
    return forward;
}

int StairDriveData::getTailValue()
{
    return tailValue;
}

int StairDriveData::getTarget()
{
    return target;
}

ledcolor_t StairDriveData::getLedColor()
{
    return ledColor;
}
