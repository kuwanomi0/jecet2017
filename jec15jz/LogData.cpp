#include "LogData.h"

LogData::LogData(string fileName)
{
	writeFile.open(fileName, ios::out);
}

LogData::~LogData()
{

}

void LogData::close()
{
	writeFile.close();
}
void LogData::writingFile(string value)
{
	writeFile << value << ",";
}

void LogData::writing(string name, int value)
{
	writeFile << value << endl;
}

void LogData::writingFile(int8_t value)
{
	writeFile << value << endl;
}
void LogData::writing(int value)
{
	writeFile << value;
}

/* 5個ずつ書き出し */
void LogData::writing(int value1, int value2, int value3, int value4, int value5)
{
	writeFile << "," << value1 << "," << value2 << "," << value3 << "," << value4 << "," << value5 << endl;
}

/* 9個書き出し */
void LogData::writing(int value1, int value2, int value3, int value4, int value5, int value6, int value7, int value8, int value9)
{
	writeFile << value1 << "," << value2 << "," << value3 << "," << value4 << "," << value5 << "," << value6 << "," << value7 << "," << value8 << "," << value9 << endl;
}

/* RGB値書き出し */
void LogData::writing(rgb_raw_t colorRgb)
{
	writeFile << colorRgb.r << "," << colorRgb.g << "," << colorRgb.b << ",";
}

void LogData::writing(int* colors)
{
	//ダミー
	for(unsigned int i = 0; i < sizeof(colors) / sizeof(colors[0]); i++)
    {
        syslog(LOG_NOTICE, "%d,", colors[i]);
    }
}
void LogData::writingFile(int r, int g, int b, int forward, int turn, int deg, int degSec, int distance, int lMotor, int rMotor)
{
	writeFile << r << "," << g << "," << b << "," << forward << ","
			  << turn << "," << deg << "," << degSec << "," << distance << "," << lMotor << "," << rMotor << endl;
}
