#pragma once

#include <iostream>
#include "ev3api.h"
#include "EV3RT.h"
#include "Stair.h"
#include "LookupData.h"
#include "Constant.h"
#include "app.h"

using namespace ev3api;
using namespace std;

class Lookup : public Stair
{
public:
    Lookup(int target, float p, int endLine, int mode, double angle, ledcolor_t ledColor);
    ~Lookup();

    LookupData* getStairData();
private:
    LookupData* lData;
};
