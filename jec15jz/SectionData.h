#pragma once

#include <iostream>
#include "ev3api.h"
#include "Constant.h"
#include "app.h"

using namespace std;

class SectionData
{
public:
    SectionData(int v, int a, int target, float p, float i, float d, int endLine, ledcolor_t ledColor);
    ~SectionData();

    int getV();
    int getAccel();
    int getTarget();
    float getP();
    float getI();
    float getD();
    int getEndLine();
    ledcolor_t getLedColor();
private:
    int v;
    int a;
    int target;
    float p;
    float i;
    float d;
    int endLine;
    ledcolor_t ledColor;
};
