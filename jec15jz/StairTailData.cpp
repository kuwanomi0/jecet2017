#include "StairTailData.h"

StairTailData::StairTailData(int tailType, int sAngle, int eAngle, int tPwm, int fPwm, int time, ledcolor_t ledColor)
{
    this->tailType = tailType;
    this->sAngle = sAngle;
    this->eAngle = eAngle;
    this->tPwm = tPwm;
    this->fPwm = fPwm;
    this->time = time;
    this->ledColor = ledColor;
}

StairTailData::~StairTailData()
{

}

int StairTailData::getTailType()
{
    return tailType;
}

int StairTailData::getStartAngle()
{
    return sAngle;
}

int StairTailData::getEndAngle()
{
    return eAngle;
}

int StairTailData::getTruePwm()
{
    return tPwm;
}

int StairTailData::getFalsePwm()
{
    return fPwm;
}

int StairTailData::getTime()
{
    return time;
}

ledcolor_t StairTailData::getLedColor()
{
    return ledColor;
}
