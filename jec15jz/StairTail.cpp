#include "StairTail.h"

StairTail::StairTail(int tailType, int sAngle, int eAngle, int tPwm, int fPwm, int time, ledcolor_t ledColor)
{
    rData = new StairTailData(tailType, sAngle, eAngle, tPwm, fPwm, time, ledColor);
}

StairTail::~StairTail()
{
    delete rData;
}


StairData* StairTail::getStairData()
{
    return rData;
}
