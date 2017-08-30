#pragma once

#include <iostream>
#include "ev3api.h"
#include "Stair.h"
#include "StairData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairDriveData : public StairData
{
public:
    StairDriveData(int stairType, int termsValue, int forward, int tailValue, int target, ledcolor_t ledColor);
    ~StairDriveData();

    int getStairType();
    int getTermsValue();
    int getForward();
    int getTailValue();
    int getTarget();
    ledcolor_t getLedColor();

private:
    int stairType;
    int termsValue;
    int forward;
    int tailValue;
    int target;
    ledcolor_t ledColor;
};
