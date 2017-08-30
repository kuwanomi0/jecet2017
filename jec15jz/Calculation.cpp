#include "Calculation.h"

Calculation::Calculation(ColorSensor* mColorSensor)
{
    this->mColorSensor = mColorSensor;
    for(int i = 0; i < 3; i++)
    {
        colorAry[i] = 0;
    }
}

Calculation::~Calculation()
{

}

//*****************************************************************************
// 関数名 : directionAll
// 引数 :
// 返り値 : 無し
// 概要 :
//*****************************************************************************
int Calculation::directionAll(int32_t left, int32_t right)
{
	float leftdistance = 81.5 * PAI * left / 360;
	float rightdistance = 81.5 * PAI * right / 360;
	int result = (leftdistance + rightdistance) / 2;

	return result;
}

float Calculation::rightDirection(float kp, float ki, float kd,  int target, rgb_raw_t& colorRgb)
{
    static int before;
    static int now;
    static int integral;

    mColorSensor->getRawColor(colorRgb);

	before = now;
	now = target - colorRgb.r;
	if(before == now)
    {
		integral = 0;
    }

	integral += (now + before) / 2.0;

	float result = kp * (now + ki * integral + kd * (now - before));
	return result;
}

float Calculation::leftDirection(float kp, float ki, float kd, int target, rgb_raw_t& colorRgb)
{
	return -rightDirection(kp, ki, kd, target, colorRgb);
}

/**
 * forwardを求めるのに使うメソッド群
 **/
int Calculation::distance(int32_t rmCount, int32_t lmCount)
{
	float lDistance = leftDistance(lmCount);
	float rDistance = rightDistance(rmCount);
	int result = (int)(lDistance + rDistance) / 2;

	return result;
}

float Calculation::leftDistance(int32_t lmCount)
{
  int l = lmCount;
  float result = 81.5 * PAI * l / 360;
  return result;
}

float Calculation::rightDistance(int32_t rmCount)
{
  int r = rmCount;
  float result = 81.5 * PAI * r / 360;
  return result;
}

int Calculation::forwardCalculation(int v, int adistance, uint32_t time, int32_t rmCount, int32_t lmCount, double a)
{
    //syslog(LOG_NOTICE,"%d", v);
    int vt = 0;
    int result = 0;

    if(v > 130)
    {
        vt = speedDifference((distance(rmCount, lmCount) - adistance), time);
        //syslog(LOG_NOTICE, "%d", vt);
        result = 4 * v - (vt - v) * a;
        result = result / 10;
        if(result > 127)
        {
            result = 127;
        }
    }
    else
    {
        result = v;
    }
    return result;
}

int Calculation::forwardCalculation(int v, int vt, double a)
{
    int result = 0;

    if(v > 130)
    {
        result = 4 * v - (vt - v) * a;
        result = result / 10;
        if(result > 127)
        {
            result = 127;
        }
    }
    else
    {
        result = v;
    }
    return result;
}

int Calculation::forwardSet(int v, int vt, double a)
{
  return v;
}

int Calculation::speedDifference(int dt, int btime)
{
  int result = dt * 1000 / btime;
  return result;
}

/* 走行中のRGB値の各平均を計算してsyslogで出力 */
int* Calculation::getColorAverage(string fileName)
{
    ifstream file;
    string reading_line_buffer = "";
    const char delimiter = ',';
    int i = 0;
    int count = 0;
    file.open("testcsv.txt", ios::in);
    while(file && getline(file, reading_line_buffer, delimiter))
    {
        colorAry[i++] += atoi(reading_line_buffer.c_str());

        count++;
        //cout << count << " colorBuff : " << reading_line_buffer.c_str() << endl;
        if(i == 3)
        {
            i = 0;
        }
    }

	count /= 3;

	for(int i = 0; i < 3; i++)
    {
        colorAry[i] /= count;
    }

    syslog(LOG_NOTICE,"%d, %d, %d\r",colorAry[0],colorAry[1],colorAry[2]);
    return colorAry;

    /*
    syslog(LOG_NOTICE,"r : %d, g : %d, b : %d",
                           colorAry[0] / count,
                           colorAry[1] / count,
                           colorAry[2] / count
          );
    */
}
