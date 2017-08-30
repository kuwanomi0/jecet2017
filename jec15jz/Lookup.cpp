#include "Lookup.h"

Lookup::Lookup(int target, float p, int endLine, int mode, double angle, ledcolor_t ledColor)
{
    lData = new LookupData(target, p, endLine, mode, angle, ledColor);
}

Lookup::~Lookup()
{
    delete lData;
}

LookupData* Lookup::getStairData()
{
    return lData;
}
