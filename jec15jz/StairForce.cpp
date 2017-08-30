#include "StairForce.h"

StairForce::StairForce(int stairType, int termsValue, int lPwm, int rPwm, int tailValue, ledcolor_t ledColor)
{
    sData = new StairForceData(stairType, termsValue, lPwm, rPwm,tailValue, ledColor);
}

StairForce::~StairForce()
{
    delete sData;
}


StairData* StairForce::getStairData()
{
    return sData;
}
