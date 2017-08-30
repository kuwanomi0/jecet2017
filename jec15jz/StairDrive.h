#pragma once

#include <iostream>
#include "ev3api.h"
#include "Stair.h"
#include "StairDriveData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class StairDrive : public Stair
{
public:
    StairDrive(int stairType, int termsValue, int forward, int tailValue, int target, ledcolor_t ledColor);
    ~StairDrive();

    StairData* getStairData() override;
private:
    StairDriveData* sData;
};
