#pragma once

#include "ev3api.h"
#include "ColorSensor.h"
#include "Clock.h"
#include "LogData.h"
#include <sstream>
#include <iostream>
#include <string>
#include <stdio.h>
#include <cstdio>

using namespace std;
using namespace ev3api;

//キャリブレーションに用いる各色の値
typedef struct
{
    int whiteR, whiteG, whiteB, white;
    int blackR, blackG, blackB, black;
    int grayR, grayG, grayB, gray;
}WBG;

//走行中の現在の位置を保存する値
typedef struct
{
    int runR, runG, runB;
}run_color;

class Calibration
{
public:
  Calibration(int32_t *bt_cmd);
  ~Calibration();

  void setColor();//指定されたColor値をセットする
  int getTarget();
  int getGray();
  int getScreanPosi();
  void getWbgColor(WBG* wbg);
  run_color getRunningColor();//走行中の色の値を取る
  ColorSensor* getColorSensor();
private:
  ColorSensor* colorSensor;
  Clock* clock;
  string colorName;
  rgb_raw_t colorRgb;
  WBG wbg;
  int rgbColors[3];
  int target;
  int gray;
  int screanPosi;
  int32_t *bt_cmd;
  int getRgbColor();
  void bt_cmdReset();
};
