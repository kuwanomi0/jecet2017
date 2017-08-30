#include "LookupData.h"

LookupData::LookupData(int target, float p, int endLine, int mode, double angle, ledcolor_t ledColor)
{
    this->target = target;
    this->p = p;
    this->endLine = endLine;
    this->mode = mode;
    this->angle = angle;
    this->ledColor = ledColor;
}

LookupData::~LookupData()
{

}


int LookupData::getTarget()
{
    return target;
}

float LookupData::getP()
{
    return p;
}

int LookupData::getEndLine()
{
    return endLine;
}

int LookupData::getMode()
{
    return mode;
}

double LookupData::getAngle()
{
    return angle;
}

ledcolor_t LookupData::getLedColor()
{
    return ledColor;
}
