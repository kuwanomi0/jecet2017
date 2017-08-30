#pragma once

#include <iostream>
#include <string>
#include <typeinfo>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include "ev3api.h"
#include "Motor.h"
#include "Clock.h"
#include "ColorSensor.h"
#include "Constant.h"
#include <syslog.h>

using namespace ev3api;
using namespace std;

class Calculation
{
public:
    Calculation(ColorSensor* mColorSensor);
    ~Calculation();

    /** turn値を求めるのに必要なメソッド群 **/
    int directionAll(int32_t left, int32_t right);
    float leftDirection(float kp, float ki, float kd,  int target, rgb_raw_t& colorRgb);
    float rightDirection(float kp, float ki, float kd,  int target, rgb_raw_t& colorRgb);

    /** forward値を求めるのに使うメソッド群 **/
    int distance(int32_t rmCount, int32_t lmCount);
    float leftDistance(int32_t lmCount);
    float rightDistance(int32_t rmCount);
    int forwardCalculation(int v, int adistance, uint32_t time, int32_t rmCount, int32_t lmCount, double a);
    int forwardCalculation(int v, int vt, double a);
    int forwardSet(int v, int vt, double a);
    int speedDifference(int dt, int btime);

    int* getColorAverage(string fileName);
private:
    ColorSensor* mColorSensor;

    rgb_raw_t colorRgb;
    int8_t forward; //ロボの速度
    int8_t turn;
    int direction; //走った距離
    int adistance;
    int colorAry[3];
    int beforeTarget;
};
