#pragma once

#include <iostream>
#include "ev3api.h"

using namespace std;

/* ログのスーパークラス */

class Log
{
public:
    Log(){}
    virtual ~Log(){}

    virtual void writing(int value) = 0;
    virtual void writing(int value1, int value2, int value3, int value4, int value5) = 0;
    virtual void writing(
                         int value1, int value2, int value3,
                         int value4, int value5, int value6,
                         int value7, int value8, int value9
                         ) = 0;
    virtual void writing(string name, int value) = 0;
    virtual void writing(rgb_raw_t colorRgb) = 0;
    virtual void writing(int* colors) = 0;
    virtual  void close() = 0;
};
