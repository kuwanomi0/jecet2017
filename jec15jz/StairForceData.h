#pragma once

#include <iostream>
#include "ev3api.h"
#include "Stair.h"
#include "StairData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairForceData : public StairData
{
public:
    StairForceData(int stairType, int termsValue, int lPwm, int rPwm, int tailValue, ledcolor_t ledColor);
    ~StairForceData();

    int getStairType();
    int getTermsValue();
    int getLeftPwm();
    int getRightPwm();
    int getTailValue();
    ledcolor_t getLedColor();

private:
    int stairType;
    int termsValue;
    int lPwm;
    int rPwm;
    int tailValue;
    ledcolor_t ledColor;
};
