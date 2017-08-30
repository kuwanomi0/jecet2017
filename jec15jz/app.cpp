/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 **/

//Rコース３コーナ調整 階段調整 階段後グレー検知 tyousei

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "Clock.h"
#include "Log.h"
#include "SysLog.h"
#include "LogData.h"
#include "Calibration.h"
#include "Tail.h"
#include "Constant.h"
#include "EV3RT.h"
#include <typeinfo>
#include <vector>

using namespace ev3api;
using namespace std;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = -1;      /* Bluetoothコマンド 1:リモートスタート */
static FILE      *bt = NULL;      /* Bluetoothファイルハンドル */

Log* wLogData;
EV3RT* ev3rt;

void main_task(intptr_t unused)
{
    ev3rt = new EV3RT(&bt_cmd);

    vector<Section*>    sectionsR;   //Rコース
    vector<Section*>    sections;   //Rコース
    vector<Section*>    sectionsL;   //Lコース
    vector<Stair*>      stairDrive;  //階段走行 (ライントレース)
    vector<Section*>    useSections; //実際に運転する走行区間
    vector<Stair*>      lookupGate;  //ルックアップゲート

    /**セクションクラスのコンストラクタ
     * new Section(目標速度, アクセル定数, ターゲット値, P, I, D, 走行区間, LEDカラー)
    ***/
    /** Rコース **/
    sectionsR.push_back(new Section( 90,  0, TARGET, KP, KI  , KD  , LINERS  , LED_RED  ));
    sectionsR.push_back(new Section( v1, a4, TARGET, KP, KI1 , KD1 , LINER1  , LED_GREEN));
    sectionsR.push_back(new Section( 80, a1, TARGET, KP, KI1 , KD1 , LINER1_D, LED_RED  ));
    //sections.push_back(new Section(v2, a2, TARGET, KP, KI2,  KD2,  LINER2_1  ));
    sectionsR.push_back(new Section( v2 , a2, TARGET, KP, KI2 , KD2 , LINER2_2, LED_GREEN));
    sectionsR.push_back(new Section( 100, a1, TARGET, KP, KI3 , KD3 , LINER3  , LED_RED  ));
    sectionsR.push_back(new Section( v3 , a3, 110   , KP, KI4 , KD4 , LINER4  , LED_GREEN));
    sectionsR.push_back(new Section( vR4, a1, 120   , KP, KI5 , KD5 , LINER5  , LED_RED  ));
    sectionsR.push_back(new Section( 127, a3, 120   , KP, KI6 , KD6 , LINER6  , LED_GREEN));
    sectionsR.push_back(new Section(  90, a1, TARGET, KP, KIc2, KDc2, LINER7  , LED_RED  ));
    sectionsR.push_back(new Section(  25,  0, TARGET, KP, KI  , KD  , 11270 , LED_GREEN));

    /** Lコース **/
    sectionsL.push_back(new Section(90,  0, TARGET, KP, KI    ,  KD     , LINELS     ,LED_RED  ));
    sectionsL.push_back(new Section(v1, a4, TARGET, KP, KIL1t  ,  KDL1t   , LINEL1     ,LED_GREEN));
    sectionsL.push_back(new Section(80, a1, TARGET, KP, KIL1  ,  KDL1   , LINEL1_D   ,LED_RED  ));
    //sectionsL.push_back(new Section(v2, a2, TARGET, KP, KI2,  KD2,  LINER2_1  ));
    //sectionsL.push_back(new Section(v2, a2, TARGET, KP, KI2,  KD2,  LINEL2_2  ));
    sectionsL.push_back(new Section(v6, a5, TARGET, KP, KIL2    , KDL2    , LINEL2   ,LED_GREEN));
    sectionsL.push_back(new Section(v5, a2, TARGET, KP, KIL2_D  , KDL2_D  , LINEL2_D ,LED_RED  ));
    //sectionsL.push_back(new Section(80, a5, TARGET, KP, KIR3_2,  KDR3_2,  LINEL2_DD  ));
    sectionsL.push_back(new Section( 80, a5, 85    , KP, KIL2_D_2,KDL2_D_2,LINEL2_D_2 ,LED_GREEN));
    sectionsL.push_back(new Section( v3, a2, TARGET, KP, KIL3    ,KDL3    ,LINEL3     ,LED_RED  ));
    sectionsL.push_back(new Section(120, a1, TARGET, KP, KIL4    ,KDL4    ,LINEL4     ,LED_GREEN));
    sectionsL.push_back(new Section( v2, a2, TARGET, KP, KIL4_2  ,KDL4_2  ,LINEL4_2   ,LED_RED  ));
    sectionsL.push_back(new Section( v4, a1, TARGET, KP, KIL5    ,KDL5    ,LINEL5     ,LED_GREEN));
    sectionsL.push_back(new Section(127, a3, TARGET, KP, KIL6    ,KDL6    ,LINEL6     ,LED_RED  ));
    sectionsL.push_back(new Section( 70, a1, TARGET, KP, KIc2    ,KDc2    ,LINEL7     ,LED_GREEN));
    sectionsL.push_back(new Section( 30,  1, TARGET, KP, KI      ,KD      ,10500    ,LED_RED  ));

    // ルックアップゲート前の直線開始地点から始める
    // ターゲット値はブルーでやってます。そっちの方が、黒と灰色に差が出るからです。
    // 走行モード
    lookupGate.push_back(new StairTail(T_DOWN, 82, 80, 20, 0, 300, LED_RED));
    lookupGate.push_back(new Lookup(25,  KP, 400,     4,  90, LED_GREEN));
    lookupGate.push_back(new Lookup(25,  KP, 999999,  0,  90, LED_GREEN));
    lookupGate.push_back(new Lookup(85,  KP, 80,      4,  90, LED_RED));
    lookupGate.push_back(new Lookup(0,   KP, 110,     1,  85, LED_GREEN));
    lookupGate.push_back(new Lookup(0,   KP, 140,     1,  80, LED_RED));
    lookupGate.push_back(new Lookup(0,   KP, 170,     1,  75, LED_GREEN));
    lookupGate.push_back(new Lookup(0,   KP, 200,     1,  70, LED_RED));
    lookupGate.push_back(new Lookup(20,  KP, 999999,  2,  65, LED_GREEN));
    lookupGate.push_back(new Lookup(20,  KP, 350,     4,  65, LED_RED));
    lookupGate.push_back(new Lookup(0,   KP, 999999,  3,  65, LED_GREEN));
    lookupGate.push_back(new Lookup(20,  KP, 999999,  2,  65, LED_RED));
    lookupGate.push_back(new Lookup(20,  KP, 330,     4,  65, LED_GREEN));
    lookupGate.push_back(new Lookup(0,   KP, 999999,  3,  65, LED_RED));
    lookupGate.push_back(new Lookup(20,  KP, 999999,  2,  65, LED_GREEN));
    lookupGate.push_back(new Lookup(20,  KP, 400,     4,  65, LED_RED));
    lookupGate.push_back(new Lookup(0,   KP, 370,     5,  70, LED_GREEN));
    lookupGate.push_back(new Lookup(0,   KP, 340,     5,  77, LED_RED));
    lookupGate.push_back(new Lookup(0,   KP, 310,     5,  84, LED_GREEN));
    lookupGate.push_back(new Lookup(0,   KP, 290,     5,  90, LED_RED));
    lookupGate.push_back(new Lookup(25,  KP, 360,     4,  90, LED_GREEN));
    lookupGate.push_back(new Lookup(25,  KP, 999999,  0,  90, LED_RED));
    lookupGate.push_back(new Lookup(85,  KP, 80,      4,  90, LED_GREEN));
    lookupGate.push_back(new Lookup(85,  KP, 999999,  6,  90, LED_RED));
    lookupGate.push_back(new Lookup(25,  KP, 250,     4,  91, LED_GREEN));
    lookupGate.push_back(new Lookup(25,  KP, 999999, 10,  91, LED_RED));

    /*階段走行*/
    /*階段走行に用いるクラスのコンストラクタ
      new StairDrive(ループを抜ける条件種類, 条件値, フォワード値, 尻尾の位置, ターゲット値, LEDの色)
      *ターゲット値が0の場合EV3RT.cpp内に宣言されてあるtarget値を使用
      new StairForce(ループを抜ける条件種類, 条件値, 左モータ値,   右モータ値, 尻尾の位置,  LEDの色)
      new StairTail(尻尾をあげるか下げるか, 最初の角度, 終了角度, 中の条件式がtrueの時に渡すPWM値,
                    中の条件式がfalseの時に渡すPWM値,tailメソッドを呼ぶ時間, LEDの色)
      条件種類
      DISTANCE     : 走行した距離
      ADISTANCE    : 走行した距離の差分
      TIME         : 時間
      ANGLE        : 角度
      L_KAITENKAKU : 左モータの回転数
      R_KAITENKAKU : 右モータの回転数
      COLOR_R1     : colorRgb.rの値が条件値(第二引数)より小さい場合
      COLOR_R2     : colorRgb.rの値が条件値(第二引数)より大きい場合
      COLOR_B1     : colorRgb.bの値が条件値(第二引数)より小さい場合
      COLOR_B2     : colorRgb.bの値が条件値(第二引数)より大きい場合
      LOOP         : 無限ループ
      T_UP         : StaitTail専用 尻尾を上げる場合
      T_DOWN       : StairTail専用 尻尾を下げる場合
    */
    stairDrive.push_back(new StairDrive(DISTANCE,  500,   30, TAIL_ANGLE_DRIVE, 130, LED_GREEN));
    stairDrive.push_back(new StairDrive(ANGLE,      50,  100, TAIL_ANGLE_DRIVE, 130, LED_RED  ));
    stairDrive.push_back(new StairDrive(TIME,      100,  -40, TAIL_ANGLE_DRIVE,  90, LED_GREEN));
    stairDrive.push_back(new StairDrive(ADISTANCE,  40,    5, TAIL_ANGLE_DRIVE,  90, LED_RED  ));
    stairDrive.push_back(new StairDrive(TIME,      800,   -2, TAIL_ANGLE_DRIVE,  90, LED_GREEN));
    stairDrive.push_back(new StairTail (T_DOWN, 82, 80, 20, 0, 300, LED_RED));
    stairDrive.push_back(new StairForce(TIME,            700, -10, -10, 80, LED_GREEN));
    stairDrive.push_back(new StairForce(TIME,            500,   0,   0, 80, LED_RED  ));
    stairDrive.push_back(new StairForce(L_KAITENKAKU,    500,  18, -18, 80, LED_GREEN));
    stairDrive.push_back(new StairForce(COLOR_R2,         15,  18, -18, 80, LED_RED  ));
    stairDrive.push_back(new StairForce(R_KAITENKAKU,     25, -18,  18, 80, LED_GREEN));
    stairDrive.push_back(new StairForce(TIME,            400, -10, -10, 80, LED_RED  ));
    stairDrive.push_back(new StairTail (T_UP, 80, 101, 0, 0, 100, LED_GREEN));
    stairDrive.push_back(new StairDrive(TIME,            3500,  -20, TAIL_ANGLE_DRIVE, 90, LED_GREEN));
    stairDrive.push_back(new StairDrive(COLOR_R2,          50,  105, TAIL_ANGLE_DRIVE, 90, LED_RED));
    stairDrive.push_back(new StairDrive(TIME,             100, -100, TAIL_ANGLE_DRIVE, 90, LED_GREEN));
    stairDrive.push_back(new StairDrive(TIME,            1000,   -5, TAIL_ANGLE_DRIVE, 90, LED_RED  ));
    stairDrive.push_back(new StairTail (T_DOWN, 82, 80, 20, 0, 300, LED_GREEN));
    stairDrive.push_back(new StairForce(COLOR_R2,         45,   -10, -10, 80, LED_RED  ));
    stairDrive.push_back(new StairForce(TIME,            200,    10,  10, 80, LED_GREEN));
    stairDrive.push_back(new StairForce(R_KAITENKAKU,    750,   -18,  18, 80, LED_RED  ));
    stairDrive.push_back(new StairForce(COLOR_R2,         25,   -18,  18, 80, LED_GREEN));
    stairDrive.push_back(new StairForce(R_KAITENKAKU,     50,   -18,  18, 80, LED_RED  ));
    stairDrive.push_back(new StairForce(TIME,            400,   -10, -10, 80, LED_GREEN));
    stairDrive.push_back(new StairTail (T_UP, 80, 102, 0, 0, 100, LED_RED));
    stairDrive.push_back(new StairDrive(TIME,      2000,  10, TAIL_ANGLE_DRIVE,  90, LED_GREEN));
    stairDrive.push_back(new StairDrive(ADISTANCE,  200,  80, TAIL_ANGLE_DRIVE,  90, LED_RED  ));
    stairDrive.push_back(new StairDrive(ADISTANCE,  100,  20, TAIL_ANGLE_DRIVE, 130, LED_GREEN));
    stairDrive.push_back(new StairDrive(ADISTANCE,  100,  30, TAIL_ANGLE_DRIVE,  40, LED_RED  ));
    stairDrive.push_back(new StairDrive(COLOR_B1 ,   90,  20, TAIL_ANGLE_DRIVE,  40, LED_GREEN));
    stairDrive.push_back(new StairDrive(COLOR_B2 ,   80,  10, TAIL_ANGLE_DRIVE,   0, LED_RED  ));
    stairDrive.push_back(new StairDrive(ADISTANCE,  200,  20, TAIL_ANGLE_DRIVE,   0, LED_GREEN));
    stairDrive.push_back(new StairTail (T_DOWN, 82, 80, 20, 0, 300, LED_RED));
    stairDrive.push_back(new StairForce(LOOP,         0,   0,  0, 80, LED_GREEN));



    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    /* スタート通知 */
    ev3_led_set_color(LED_GREEN);

    //尻尾お下ろして命令待ち
    ev3rt->ready();

    char buf[100];
    sprintf(buf,"btcmd : %d",(int)bt_cmd);
    ev3_lcd_draw_string(buf, 0, 10);

    /** 走行コース決め、1 = Rコース : 2 = Lコース **/
    switch(bt_cmd)
    {
    case 1:
        useSections = sectionsR;
        break;
    case 2:
        useSections = sectionsL;
        break;
    }
    /* スタート通知 */
    ev3_led_set_color(LED_GREEN);

    /* ロボの走行開始 */
    if(bt_cmd != -1)
    {
        //ログ書き込みタスク開始
        ev3_sta_cyc(LOG_CYC_TASK);

        //走行区間の数だけ回す
        for(auto itr = useSections.begin(); itr != useSections.end(); itr++)
        {
            ev3rt->driving( (*itr)->getSectionData() );
        }
    }

    if(bt_cmd == 1 || bt_cmd == 4) //ルックアップゲート
    {
        //難所までの走行距離を保存
        ev3rt->setCourseDistance();

        for(auto itr = lookupGate.begin(); itr != lookupGate.end(); itr++)
        {
            if(typeid(**itr) == typeid(Lookup))
            {
                syslog(LOG_NOTICE, "Lookup");
                ev3rt->lookupDrive( (LookupData*)(*itr)->getStairData() );
            }
            else if(typeid(**itr) == typeid(StairTail))
            {
                syslog(LOG_NOTICE, "stairTail");
                ev3rt->stairTail( (StairTailData*)(*itr)->getStairData());
            }
        }
    }
    else if (bt_cmd == 2 || bt_cmd == 5)//階段
    {
        //難所までの走行距離を保存
        ev3rt->setCourseDistance();

        for(auto itr = stairDrive.begin(); itr != stairDrive.end(); itr++)
        {
            //**itrの派生クラスがStairDriveなら
            if(typeid(**itr) == typeid(StairDrive))
            {
                syslog(LOG_NOTICE, "stairDrive");
                ev3rt->stairDrive( (StairDriveData*)(*itr)->getStairData() );
            }
            //**itrの派生クラスがStairForceなら
            else if(typeid(**itr) == typeid(StairForce))
            {
                syslog(LOG_NOTICE, "stairForce");
                ev3rt->stairForce( (StairForceData*)(*itr)->getStairData());
            }
            //**itrの派生クラスがStairTailなら
            else if(typeid(**itr) == typeid(StairTail))
            {
                syslog(LOG_NOTICE, "stairTail");
                ev3rt->stairTail( (StairTailData*)(*itr)->getStairData());
            }
        }
    }

    //ev3rt->emergencyStair();

    /*
    else if(bt_cmd == 1)
    {
        // ルックアップゲート前の直線開始地点から始める
        sections.push_back(new Section(20, 90, 30, KP, KI, KD, 50, LED_RED));
        sections.push_back(new Section(20, 85, 30, KP, KI, KD, 100, LED_RED));
        sections.push_back(new Section(20, 80, 30, KP, KI, KD, 150, LED_RED));
        sections.push_back(new Section(20, 75, 30, KP, KI, KD, 200, LED_RED));
        sections.push_back(new Section(20, 70, 30, KP, KI, KD, 250, LED_RED));
        sections.push_back(new Section(20, 65, 30, KP, KI, KD, 1200, LED_RED));
        // 1st回転
        sections.push_back(new Section(20, 65, 30, KP, KI, KD, 999999, LED_RED));
        // 戻る
        sections.push_back(new Section(20, 65, 30, KP, KI, KD, 1250, LED_RED));
        // 2nd回転
        sections.push_back(new Section(20, 65, 30, KP, KI, KD, 999999, LED_RED));
        // 再び進む
        sections.push_back(new Section(20, 65, 30, KP, KI, KD, 1500, LED_RED));
    }
    */
    syslog(LOG_NOTICE, "end");

    if(wLogData != NULL)
    {
        wLogData->close();
    }

    //走行中のRGB値の平均値を出力
    wLogData->writing(ev3rt->getColorAverage("testcsv.txt"));

    //モーターストップ
    ev3rt->stop();

    //ログ書き込みタスク終了
    ev3_stp_cyc(LOG_CYC_TASK);

    //ブルートゥースタスクを終了
    ter_tsk(BT_TASK);

    //開放
    fclose(bt);

    //デストラクタ呼び出し
    delete ev3rt;

    //タスクをすべて終了
    ext_tsk();
}

//*****************************************************************************
// 関数名 : calib_task
// 引数 : unused
// 返り値 : なし
// 概要 : ロボの走行中に4msecごとに現在の位置から色を取得する
//***********************************************************************
void log_cyc_task(intptr_t unused)
{
    act_tsk(LOG_TASK);
}

//*****************************************************************************
// 関数名 : test2_task
// 引数 : unused
// 返り値 :CRE_TSKで作れるかの確認、LEDの色をオレンジにする
//***********************************************************************
void log_task(intptr_t unused)
{
    if(wLogData != NULL)
    {
        //wLogData->writing(ev3rt->getForward());
        wLogData->writing(ev3rt->getColorRgb());
        //wLogData->writing((int)ev3rt->getColorAmbient());
        /*
        wLogData->writing((int)ev3rt->getColorBrightness());
        wLogData->writing((int)ev3rt->getGyroAngle());
        wLogData->writing((int)ev3rt->getGyroAngleVelocity());
        wLogData->writing((int)ev3rt->getSonarDistance());
        wLogData->writing((int)ev3rt->getLeftMotorCount());
        wLogData->writing((int)ev3rt->getRightMotorCount());
        wLogData->writing((int)ev3rt->getLeftMotorPwm());
        wLogData->writing((int)ev3rt->getRightMotorPwm());
        //wLogData->writing("forward", ev3rt->getForward());
        */

        /*
        wLogData->writing(ev3rt->getColorRgb());
        wLogData->writing
                  (
                    (int)ev3rt->getColorAmbient(),
                    (int)ev3rt->getColorBrightness(),
                    (int)ev3rt->getGyroAngle(),
                    (int)ev3rt->getGyroAngleVelocity(),
                    (int)ev3rt->getSonarDistance()
        );

        wLogData->writing
                  (
                    (int)ev3rt->getLeftMotorCount(),
                    (int)ev3rt->getLeftMotorPwm(),
                    (int)ev3rt->getRightMotorCount(),
                    (int)ev3rt->getRightMotorPwm(),
                    0
                  );
                  */
    }
}

//*****************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//*****************************************************************************
void bt_task(intptr_t unused)
{
    while(1)
    {
        char c = fgetc(bt); /* 受信 */
        switch(c)
        {
        /* Rコース */
        case '0':
        case 'R':
        case 'r':
        	bt_cmd = 1;
        	break;
        /* Lコース */
        case '1':
        case 'l':
        case 'L':
            bt_cmd = 2;
            break;
        /* 白の色取り */
        case 'w':
            bt_cmd = 3;
            break;
        /* 黒の色取り */
        case 'b':
            bt_cmd = 4;
            break;
        /* 灰色の色取り */
        case 'g':
            bt_cmd = 5;
            break;
        /* CSVファイルへのログ書き込み */
        case 'c':
            wLogData = (Log*) new LogData("testcsv.txt");
            break;
        /* syslogでteraterm上に出力 */
        case 's':
            wLogData = (Log*) new SysLog();
            break;
        case 'q':
            ev3rt->forceStop();
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}
