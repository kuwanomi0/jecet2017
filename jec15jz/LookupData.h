#pragma once

#include <iostream>
#include "ev3api.h"
#include "StairData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class LookupData : public StairData
{
public:
    LookupData(int target, float p, int endLine, int mode, double angle, ledcolor_t ledColor);
    ~LookupData();

    int getTarget();
    float getP();
    int getEndLine();
    int getMode();
    double getAngle();
    ledcolor_t getLedColor();
private:
    int target;
    float p;
    int endLine;
    int mode;
    double angle;
    ledcolor_t ledColor;
};
