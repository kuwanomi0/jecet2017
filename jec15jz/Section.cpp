#include "Section.h"

Section::Section(int v, int a, int target, float p, float i, float d, int endLine, ledcolor_t ledColor)
{
    sData = new SectionData(v, a, target, p, i, d, endLine, ledColor);
}

Section::~Section()
{
    delete sData;
}


SectionData* Section::getSectionData()
{
    return sData;
}
