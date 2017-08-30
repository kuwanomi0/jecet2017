#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include "Log.h"
#include "ev3api.h"

using namespace std;

class LogData : public Log
{
private:
	ofstream writeFile;

public:
	LogData(string fileName);
	~LogData() override;

    void writing(string name, int value) override;
    void writing(rgb_raw_t colorRgb) override;
    void writing(int value) override;
    void writing(int value1, int value2, int value3, int value4, int value5) override;
    void writing
         (
             int value1, int value2, int value3,
             int value4, int value5, int value6,
             int value7, int value8, int value9
         ) override;
    void writing(int* colors) override;
	void close() override;

    void writingFile(string value);
    void writingFile(int8_t value);
    void writingFile(int r, int g, int b, int forward, int turn, int deg, int degSec, int distance, int rMotor, int lMotor);
};
