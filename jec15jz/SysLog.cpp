#include "SysLog.h"

SysLog::SysLog()
{

}

SysLog::~SysLog()
{

}

void SysLog::logStart()
{
    syslog(LOG_NOTICE, "start");
}

void SysLog::logEnd()
{
    syslog(LOG_NOTICE, "end");
}

void SysLog::close()
{
    syslog(LOG_NOTICE, "close");
}
void SysLog::writing(int value1, int value2, int value3, int value4, int value5)
{
    syslog
    (
       LOG_NOTICE, ",%d,%d,%d,%d,%d",
                    value1, value2, value3, value4,value5
    );
}

void SysLog::writing(int value1, int value2, int value3,int value4, int value5, int value6,int value7, int value8, int value9)
{
    syslog
    (
       LOG_NOTICE, "%d,%d,%d,%d,%d,%d,%d,%d,%d\r",
                    value1, value2, value3, value4,
                    value5, value6, value7, value8, value9
    );
}
void SysLog::writing(int value)
{
    syslog(LOG_NOTICE, "%d,", value);
}

void SysLog::writing(string name, int value)
{
    syslog(LOG_NOTICE, "%s,", name.c_str());
    syslog(LOG_NOTICE, "%d\r", value);
}

void SysLog::writing(rgb_raw_t rgbColor)
{
    syslog(LOG_NOTICE, "%d,%d,%d", rgbColor.r, rgbColor.g, rgbColor.b);
}

void SysLog::writing(int* colors)
{
    for(unsigned int i = 0; i < sizeof(colors) / sizeof(colors[0]); i++)
    {
        syslog(LOG_NOTICE, "%d,", colors[i]);
    }
}
