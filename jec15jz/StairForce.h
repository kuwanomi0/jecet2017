#pragma once

#include <iostream>
#include "ev3api.h"
#include "Stair.h"
#include "StairForceData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairForce : public Stair
{
public:
    StairForce(int stairType, int termsValue, int lPwm, int rPwm, int tailValue, ledcolor_t ledColor);
    ~StairForce();

    StairData* getStairData() override;
private:
    StairForceData* sData;
};
