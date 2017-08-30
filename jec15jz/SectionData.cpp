#include "SectionData.h"

SectionData::SectionData(int v, int a, int target, float p,
                        float i, float d, int endLine, ledcolor_t ledColor)
{
    this->v = v;
    this->a = a;
    this->target = target;
    this->p = p;
    this->i = i;
    this->d = d;
    this->endLine = endLine;
    this->ledColor = ledColor;
}

SectionData::~SectionData()
{

}

int SectionData::getV()
{
    return v;
}

int SectionData::getAccel()
{
    return a;
}

int SectionData::getTarget()
{
    return target;
}

float SectionData::getP()
{
    return p;
}

float SectionData::getI()
{
    return i;
}

float SectionData::getD()
{
    return d;
}

int SectionData::getEndLine()
{
    return endLine;
}

ledcolor_t SectionData::getLedColor()
{
    return ledColor;
}
