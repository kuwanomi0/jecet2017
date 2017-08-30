#pragma once

#include <iostream>
#include "ev3api.h"
#include "EV3RT.h"
#include "SectionData.h"
#include "Constant.h"
#include "app.h"

using namespace ev3api;
using namespace std;

class Section
{
public:
    Section(int v, int a, int target, float p, float i, float d, int endLine, ledcolor_t ledColor);
    ~Section();

    SectionData* getSectionData();
private:
    SectionData* sData;
};
