#pragma once

#include <stdio.h>
#include <string>
#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "Stair.h"
#include "StairDrive.h"
#include "StairForce.h"
#include "StairTail.h"
#include "Lookup.h"
#include "LookupData.h"
#include "LogData.h"
#include "Calibration.h"
#include "Tail.h"
#include "Constant.h"
#include "Section.h"
#include "SectionData.h"
#include "SysLog.h"
#include "Calculation.h"

using namespace std;
using namespace ev3api;

class EV3RT
{
public:
    EV3RT(int32_t* bt_cmd);
    ~EV3RT();

    void driving(SectionData* sData);
    void stairDrive(StairDriveData* sData);
    void stairForce(StairForceData* sData);
    void underTail();
    void stairTail(StairTailData* rData);
    void lookupDrive(LookupData* lData);
    void stop();
    void ready();
    void forceStop();
    void setCourseDistance();

    /* ログに出力するに使用するメソッド群 */
    int8_t    getForward();             //フォワード値
    int8_t    getTurn();                //ターン値
    int       getV();                   //目標速度

    rgb_raw_t getColorRgb();            //R,G,Bで散った値
    uint8_t   getColorAmbient();        //環境光の値
    int8_t    getColorBrightness();     //反射光の値
    int16_t   getGyroAngle();           //ロボの角度
    int16_t   getGyroAngleVelocity();   //ロボのオフセット付の角度
    int16_t   getSonarDistance();       //距離
    int32_t   getLeftMotorCount();      //左モータの回転角
    int32_t   getRightMotorCount();     //右モータの回転角
    int       getLeftMotorPwm();        //左モータに渡す値
    int       getRightMotorPwm();       //右モータに渡す値
    int*      getColorAverage(string fileName);
private:
    /* オブジェクトへのポインタ定義 */
    TouchSensor*    mTouchSensor;       //タッチセンサー
    SonarSensor*    mSonarSensor;       //ソナーセンサー
    ColorSensor*    mColorSensor;       //カラーセンサー
    GyroSensor*     mGyroSensor;        //ジャイロセンサー
    Motor*          mLeftMotor;         //左タイヤモーター
    Motor*          mRightMotor;        //右タイヤモーター
    Motor*          mTailMotor;         //尻尾モーター
    Clock*          mClock;             //クロック1
    Clock*          mClock2;            //クロック2
    Clock*          mClock3;            //クロック3
    Tail*           tail;               //尻尾
    LogData*        wLogData;           //ファイルに書き込み
    SysLog*         sysLog;             //teraterm上に書き込み
    Calculation*    calculation;        //走行のための計算用クラス

    char      buf[100];
    int32_t*  bt_cmd;
    int       time;                      //走行時間
    rgb_raw_t colorRgb;
    int       sectionPlace;

    int8_t    forward;
    int8_t    turn;
    int       direction;
    int       pointDirection;
    float     pointRightDirection;
    int       adistance;
    int8_t    pwmL;
    int8_t    pwmR;
    int       target;
    int       v;

    int courseDistance;
    int addTarget;
    int stairAdistance;
    int kaitenkaku;
    bool b_stop;         //走行を強制終了
    void balancer_control(float forward, float turn);
    void stairDriving(StairDriveData* sData);
    void stairForcing(StairForceData* sData);
};
