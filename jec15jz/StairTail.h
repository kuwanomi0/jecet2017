#pragma once

#include <iostream>
#include "ev3api.h"
#include "Stair.h"
#include "StairTailData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairTail : public Stair
{
public:
    StairTail(int TailType, int sAngle, int eAngle, int tPwm, int fPwm, int time, ledcolor_t ledColor);
    ~StairTail();

    StairData* getStairData() override;
private:
    StairTailData* rData;
};
