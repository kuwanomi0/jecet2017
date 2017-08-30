#pragma once

#include <iostream>
#include "ev3api.h"
#include "StairData.h"
#include "Constant.h"
#include "app.h"

using namespace std;

typedef enum
{
    DISTANCE,
    ADISTANCE,
    TIME,
    COLOR_R1,
    COLOR_R2,
    COLOR_B1,
    COLOR_B2,
    L_KAITENKAKU,
    R_KAITENKAKU,
    ANGLE,
    LOOP
}STAIRTYPE;

typedef enum
{
    T_UP,
    T_DOWN
}STAIRTAILTYPE;

class Stair
{
public:
    Stair();
    virtual ~Stair();

    virtual StairData* getStairData() = 0;

    virtual void dummy() {}
};
