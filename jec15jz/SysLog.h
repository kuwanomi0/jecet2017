#pragma once

#include <iostream>
#include "Log.h"
#include "ev3api.h"

using namespace std;

class SysLog : public Log
{
public:
    SysLog();
    ~SysLog() override;

    void logStart();
    void logEnd();

    void close() override;
    void writing(int value) override;
    void writing(int value1, int value2, int value3, int value4, int value5) override;
    void writing
         (
             int value1, int value2, int value3,
             int value4, int value5, int value6,
             int value7, int value8, int value9
         ) override;
    void writing(int* colors) override;
    void writing(string name, int value) override;
    void writing(rgb_raw_t rgbColor) override;
private:

};
