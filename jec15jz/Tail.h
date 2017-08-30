#pragma once

#include "ev3api.h"
#include "Constant.h"
#include "Motor.h"
#include "Clock.h"
#include "TouchSensor.h"
#include "ColorSensor.h"

using namespace std;
using namespace ev3api;

class Tail
{
public:
    Tail();
    ~Tail();

    void tail_control(int32_t angle);
    void tail(double angle);
    void initTail();
    void standTail(TouchSensor* mTouchSensor, ColorSensor* colorSensor, int32_t* bt_cmd );

private:
    float  data1;
    float  data2;
    float  dev;
    float  p;

    int target;
    rgb_raw_t colorRgb;
    Motor* mTailMotor;
    Clock* mClock;
};
