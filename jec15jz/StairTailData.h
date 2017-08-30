#pragma once

#include <iostream>
#include "ev3api.h"
#include "StairData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairTailData : public StairData
{
public:
    StairTailData(int tailType, int sAngle, int eAngle, int tPwm, int fPwm, int time, ledcolor_t ledColor);
    ~StairTailData();

    int getTailType();
    int getStartAngle();
    int getEndAngle();
    int getTruePwm();
    int getFalsePwm();
    int getTime();
    ledcolor_t getLedColor();

private:
    int tailType;
    int sAngle;
    int eAngle;
    int tPwm;
    int fPwm;
    int time;
    ledcolor_t ledColor;
};
