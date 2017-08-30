#include "StairDrive.h"

StairDrive::StairDrive(int stairType, int termsValue, int forward, int tailValue, int target, ledcolor_t ledColor)
{
    sData = new StairDriveData(stairType, termsValue, forward, tailValue, target, ledColor);
}

StairDrive::~StairDrive()
{
    delete sData;
}


StairData* StairDrive::getStairData()
{
    return sData;
}
