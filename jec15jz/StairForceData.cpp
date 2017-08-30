#include "StairForceData.h"

StairForceData::StairForceData(int stairType, int termsValue, int lPwm, int rPwm, int tailValue, ledcolor_t ledColor)
{
    this->stairType = stairType;
    this->termsValue = termsValue;
    this->lPwm = lPwm;
    this->rPwm = rPwm;
    this->tailValue = tailValue;
    this->ledColor = ledColor;
}

StairForceData::~StairForceData()
{

}

int StairForceData::getStairType()
{
    return stairType;
}

int StairForceData::getTermsValue()
{
    return termsValue;
}

int StairForceData::getLeftPwm()
{
    return lPwm;
}

int StairForceData::getRightPwm()
{
    return rPwm;
}

int StairForceData::getTailValue()
{
    return tailValue;
}

ledcolor_t StairForceData::getLedColor()
{
    return ledColor;
}
