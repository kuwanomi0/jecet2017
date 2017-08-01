/**
 ******************************************************************************
 ** ファイル名 : app.cpp
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用C++サンプルプログラム
 **
 ** 注記 : sample_cpp (ライントレース/尻尾モータ/超音波センサ/リモートスタート)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "BalancerCpp.h"
#include "TouchSensor.h"
#include "SonarSensor.h"
#include "ColorSensor.h"
#include "GyroSensor.h"
#include "Motor.h"
#include "Clock.h"
#include "PID.h"
#include "Distance.h"

#include "Course.h"

using namespace ev3api;

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/* Bluetooth */
static int32_t   bt_cmd = 0;      /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;      /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* 走行に関するマクロ */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define RGB_WHITE           500  /* 白色のRGBセンサの合計 */
#define RGB_BLACK            20  /* 黒色のRGBセンサの合計 */
#define RGB_TARGET          240  /* 中央の境界線のRGBセンサ合計値 */
#define RGB_NULL              5  /* 何もないときのセンサの合計 */
#define KP_WALK         0.1200F  /* 走行用定数P TODO :1 この値は走行時には使われていない*/
#define KI_WALK         0.0000F  /* 走行用定数I TODO :1 この値は走行時には使われていない*/
#define KD_WALK         0.1000F  /* 走行用定数D TODO :1 この値は走行時には使われていない*/
#define FORWARD_K            75  /* ロボットの前進速度 TODO :1 この値は走行時には使われていない*/
#define PIDX                  1  /* PID倍率 */

/* 超音波センサーに関するマクロ */
#define SONAR_ALERT_DISTANCE 20  /* 超音波センサによる障害物検知距離[cm] */

/* 尻尾に関するマクロ */
#define TAIL_ANGLE_STAND_UP   92 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_ROKET     100 /* ロケットダッシュ時の角度[度] */
#define TAIL_ANGLE_DRIVE       3 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_STOP       75 /* 停止処理時の角度[度] */
#define KP_TAIL            1.00F /* 尻尾用定数P */
#define KI_TAIL            0.01F /* 尻尾用定数I */
#define KD_TAIL            0.15F /* 尻尾用定数D */
#define PWM_ABS_MAX           60 /* 完全停止用モータ制御PWM絶対最大値 */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);

/* オブジェクトへのポインタ定義 */
TouchSensor*    touchSensor;
SonarSensor*    sonarSensor;
ColorSensor*    colorSensor;
GyroSensor*     gyroSensor;
Motor*          leftMotor;
Motor*          rightMotor;
Motor*          tailMotor;
Clock*          clock;

/* インスタンスの生成 */
Balancer balancer;              // <1>
PID pid_walk(KP_WALK, KI_WALK, KD_WALK); /* 走行用のPIDインスタンス */
PID pid_tail(KP_TAIL, KI_TAIL, KD_TAIL); /* 尻尾用のPIDインスタンス */
Distance distance_way;

#define R_COURSE   15   /* Rコースの一番最初の値が格納されている番号、Lコースの配列の追加削除をした際にはRコースの先頭配列の番号を変える必要があります */  //TODO :2 コース関連 だいぶ改善されました

static Course gCourseL[]  {   //TODO :2 コース関連 だいぶ改善されました
    /* Lコース用配列 */
    { 0,     0,110,  0, 0.0300F, 0.00000F, 0.0050F },  // 直線 //TODO :2 コース関連 だいぶ改善されました
    { 1,  2225,110,-13, 0.0500F, 0.00000F, 0.0500F },  // 曲線 //TODO :2 コース関連 だいぶ改善されました
    { 2,  3850,110,  8, 0.0800F, 0.00000F, 0.0530F },  // 曲線 //TODO :2 コース関連 だいぶ改善されました
    { 3,  4554,110,  2, 0.0700F, 0.00000F, 0.0530F },   //TODO :2 コース関連 だいぶ改善されました
    { 4,  5209, 75,  0, 0.1000F, 0.00000F, 0.1030F },   //TODO :2 コース関連 だいぶ改善されました
    { 5,  6134, 75,  0, 0.0900F, 0.00000F, 0.0530F },   //TODO :2 コース関連 だいぶ改善されました
    { 6,  6674, 75,  0, 0.1200F, 0.00000F, 0.0930F },   //TODO :2 コース関連 だいぶ改善されました
    { 7,  7562, 75,  0, 0.1000F, 0.00000F, 0.1030F },   //TODO :2 コース関連 だいぶ改善されました
    { 8,  8612, 75,  0, 0.0600F, 0.00000F, 0.0330F },   //TODO :2 コース関連 だいぶ改善されました
    { 9,  9900, 75,  0, 0.0900F, 0.00000F, 0.0330F },   //TODO :2 コース関連 だいぶ改善されました
    {10, 10030, 75,  0, 0.0000F, 0.00000F, 0.0030F },   //TODO :2 コース関連 だいぶ改善されました
    {11, 10401, 75,  0, 0.1200F, 0.00000F, 0.1030F },   //TODO :2 コース関連 だいぶ改善されました
    {12, 11576, 75,  0, 0.0000F, 0.00000F, 0.0030F },   //TODO :2 コース関連 だいぶ改善されました
    {13, 11766, 75,  0, 0.1200F, 0.00000F, 0.1030F },   //TODO :2 コース関連 だいぶ改善されました
    {14, 99999, 75,  0, 0.0000F, 0.00000F, 0.00306F}    //TODO :2 コース関連 だいぶ改善されました
};

static Course gCourseLFullPID[] {
    { 0,     0,125,  0, 0.0600F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
    { 1,  2100, 85,  0, 0.1000F, 0.0000F, 0.0100F },   //TODO :2 コース関連 だいぶ改善されました
    { 2,  3927, 85,  0, 0.1000F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 3,  4754,100,  0, 0.0700F, 0.0000F, 0.0500F },   //TODO :2 コース関連 だいぶ改善されました
    { 4,  5209, 85,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 5,  6134,100,  0, 0.0800F, 0.0000F, 0.0500F },   //TODO :2 コース関連 だいぶ改善されました
    { 6,  6674, 85,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 7,  7562, 85,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 8,  8612,100,  0, 0.0600F, 0.0000F, 0.0300F },   //TODO :2 コース関連 だいぶ改善されました
    { 9,  9900, 30,  0, 0.0900F, 0.0000F, 0.0300F },   //TODO :2 コース関連 だいぶ改善されました
    {10, 10030, 30,  0, 0.0000F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
    {11, 10351, 30,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    {12, 11576, 30,  0, 0.0000F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
    {13, 11766, 50,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    {14, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
};

static Course gCourseR[]  {
    /* Rコース用配列 */
    { 0,     0,125,  0, 0.0600F, 0.0000F, 0.0100F },   //TODO :2 コース関連 だいぶ改善されました
    { 1,  2225, 80,  0, 0.1000F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 2,  5400, 80,  0, 0.1000F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 3,  6350, 80,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 4,  7250, 80,  0, 0.1000F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 5,  8900,100,  0, 0.0600F, 0.0000F, 0.0300F },   //TODO :2 コース関連 だいぶ改善されました
    { 6, 10198, 30,  0, 0.0900F, 0.0000F, 0.0300F },   //TODO :2 コース関連 だいぶ改善されました
    { 7, 10298, 30,  0, 0.0000F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
    { 8, 10570, 20,  0, 0.1000F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    { 9, 11950, 50,  0, 0.0000F, 0.0000F, 0.0000F },   //TODO :2 コース関連 だいぶ改善されました
    {10, 12167, 30,  0, 0.1200F, 0.0000F, 0.1000F },   //TODO :2 コース関連 だいぶ改善されました
    {11, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }   //TODO :2 コース関連 だいぶ改善されました
};   //TODO :2 コース関連 だいぶ改善されました

/* メインタスク */
void main_task(intptr_t unused)
{
    int8_t    forward;      /* 前後進命令 */
    int8_t    turn;         /* 旋回命令 */
    int8_t    pwm_L, pwm_R; /* 左右モータPWM出力 */
    rgb_raw_t rgb_level;    /* カラーセンサーから取得した値を格納する構造体 */
    int8_t course_number = 0; //TODO :2 強引な区間設定によって作られた変数です
    int count = 0;  //TODO :2 強引な区間設定によって作られた変数です
    int roket = 0;  //TODO :3 ロケットスタート用変数 タイマーの役割をしています
    int tail_i = 0; //TODO :4 おまけ コマンドでの終了する際のタイマー用変数
    int8_t forward_course = 50; //TODO :2 強引な区間設定によって作られた変数
    int8_t turn_course = 0; //TODO :2
    uint16_t rgb_total;
    Course* mCourse = NULL;

    /* 各オブジェクトを生成・初期化する */
    touchSensor = new TouchSensor(PORT_1);
    sonarSensor = new SonarSensor(PORT_2);
    colorSensor = new ColorSensor(PORT_3);
    gyroSensor  = new GyroSensor(PORT_4);
    leftMotor   = new Motor(PORT_C);
    rightMotor  = new Motor(PORT_B);
    tailMotor   = new Motor(PORT_A);
    clock       = new Clock();

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET 16JZ", 0, CALIB_FONT_HEIGHT*1);

    /* 尻尾モーターのリセット */
    tailMotor->reset();

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    double angle = (double)TAIL_ANGLE_STAND_UP;
    while(1)
    {
        tail_control(angle); /* 完全停止用角度に制御、調整も可 */

        /* Lコース */  //TODO :2 コース関連 だいぶ改善されました
        if (bt_cmd == 1 || touchSensor->isPressed())
        {
            mCourse = gCourseL;
            break; /* リモートスタート */
        }
        /* Rコース */  //TODO :2 コース関連 だいぶ改善されました
        if (bt_cmd == 2)
        {
            mCourse = gCourseR;//TODO :2 コース関連 だいぶ改善されました
            break; /* リモートスタート */
        }
        if (bt_cmd == 3)
        {
            mCourse = gCourseLFullPID;//TODO :2 コース関連 だいぶ改善されました
            break; /* リモートスタート */
        }

        // if (bt_cmd == '?')     // TODO :BAKA
        // {     // TODO :BAKA
            // break;     // TODO :BAKA
        // }     // TODO :BAKA

        // if (touchSensor->isPressed())
        // {
        //     break; /* タッチセンサが押された */
        // }

        // スタート前の尻尾調整
        if (ev3_button_is_pressed(DOWN_BUTTON) || bt_cmd == 'd') {
            angle -= 0.1;
            bt_cmd = 0; // コマンドリセット
        }
        if (ev3_button_is_pressed(UP_BUTTON) || bt_cmd == 'u') {
            angle += 0.1;
            bt_cmd = 0; // コマンドリセット
        }
        syslog(LOG_NOTICE, "DEBUG, angle（尻尾の角度ぉぉぉぉ） : %d\r", (int)angle);

        clock->sleep(10); /* 10msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    leftMotor->reset();
    rightMotor->reset();

    /* ジャイロセンサーリセット */
    gyroSensor->reset();
    balancer.init(GYRO_OFFSET); /* 倒立振子API初期化 */  // <1>

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /**
    * Main loop for the self-balance control algorithm
    */
    while(1)
    // while(bt_cmd != '?')   // TODO :BAKA   BAKAプログラム有効用条件式
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;
        int32_t distance_now; /*現在の走行距離を格納する変数 */

        /* バックボタンによる停止処理です */
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            break;
        }

        /* 参照しているコース配列が切り替わったことをLEDで知らせます */
        if((course_number % 2) == 1) {  /* 奇数配列参照時、赤 */
            ev3_led_set_color(LED_RED);
        }
        else {                          /* 偶数配列参照時、緑 */
            ev3_led_set_color(LED_GREEN);
        }

        /* 尻尾の制御 */
        if (bt_cmd == 6) {  // TODO :4 おまけ コマンド終了停止用の角度変更を回避するための分岐
        }
        else if(roket++ < 50)                                              //TODO :3 ロケットスタートと呼ぶにはまだ怪しい、改良必須
            tail_control(TAIL_ANGLE_ROKET); /* ロ8ット走行用角度に制御 */  //TODO :3 ロケットスタートと呼ぶにはまだ怪しい、改良必須
        else {
            tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
        }

        colorSensor->getRawColor(rgb_level); /* RGB取得 */
        rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b);

        /* 転倒時の停止処理 */
        if(rgb_total <= RGB_NULL) {
            break;
        }

        /* 現在の走行距離を取得 */
        distance_now = distance_way.distanceAll(leftMotor->getCount(), rightMotor->getCount());

        /* 区間変更を監視、行うプログラム */
        if (distance_now >= mCourse[count].getDis()) {      //TODO :2 もっといい書き方があると思います。
            course_number  = mCourse[count].getCourse_num();      //TODO :2 もっといい書き方があると思います。
            forward_course = mCourse[count].getForward();      //TODO :2 もっといい書き方があると思います。
            turn_course    = mCourse[count].getTurn();
            pid_walk.setPID(mCourse[count].getP() * PIDX, mCourse[count].getI() * PIDX, mCourse[count].getD() * PIDX);      //TODO :2 もっといい書き方があると思います。
            count++;      //TODO :2 もっといい書き方があると思います。
        }      //TODO :2 もっといい書き方があると思います。

        if (sonar_alert() == 1) {/* 障害物検知 */
            forward = turn = 0; /* 障害物を検知したら停止 */
            ev3_led_set_color(LED_RED);
        }
        else {
            if (bt_cmd == 7 || bt_cmd == 6) //TODO 4: おまけコマンド停止処理用
            {
                forward = -40; //TODO 4: おまけコマンド停止処理用
            }
            else {
                forward = forward_course; /* 前進命令 */
            }
            /* PID制御 */
            // turn =  pid_walk.calcControl(((RGB_BLACK + RGB_WHITE) / 2) - rgb_total);
            turn =  pid_walk.calcControl(RGB_TARGET - rgb_total) + turn_course;
        }

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        /* 倒立振子制御APIを呼び出し、倒立走行するための */
        /* 左右モータ出力値を得る */
        balancer.setCommand(forward, turn);   // <1>
        balancer.update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = balancer.getPwmRight();       // <3>
        pwm_R = balancer.getPwmLeft();        // <3>

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);

        /* ログを送信する処理　*/
        // syslog(LOG_NOTICE, "DEBUG, C:%2d, DIS:%5d, GYRO:%3d, R:%3d, G:%3d, B:%3d, T:%4d\r", course_number, distance_now, gyro, rgb_level.r, rgb_level.g, rgb_level.b, rgb_total);
        syslog(LOG_NOTICE, "D:%5d, G:%3d, T:%3d, L:%3d, R:%3d\r", distance_now, gyro, turn, pwm_L, pwm_R);
        // if (bt_cmd == 1)
        // {
        //     syslog(LOG_NOTICE, "DEBUG, DIS:%5d, GYRO:%3d, C:%2d, F:%3d\r", distance_now, gyro, course_number, forward);
        //     bt_cmd = 0;
        // }

        // TODO :4 おまけ
        if (bt_cmd == 6)
        {
            // if (tail_i++ < 400) {
            //     tail_control(TAIL_ANGLE_STOP);
            // }
            // else {
                break;
            // }
        }


        clock->sleep(4); /* 4msec周期起動 */
    }
    leftMotor->reset();
    rightMotor->reset();
    tailMotor->reset();

    /* ここからBAKAプログラム */
    if (bt_cmd == '?') {
        syslog(LOG_NOTICE, "music mode\r\r");
        ev3_speaker_set_volume (1);
        while (bt_cmd != '-') {
            syslog(LOG_NOTICE, "play music!!\r");
            if (bt_cmd == 'z') {
                ev3_speaker_play_tone (NOTE_C5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 's') {
                ev3_speaker_play_tone (NOTE_CS5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'x') {
                ev3_speaker_play_tone (NOTE_D5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'd') {
                ev3_speaker_play_tone (NOTE_DS5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'c') {
                ev3_speaker_play_tone (NOTE_E5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'v') {
                ev3_speaker_play_tone (NOTE_F5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'g') {
                ev3_speaker_play_tone (NOTE_FS5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'b') {
                ev3_speaker_play_tone (NOTE_G5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'h') {
                ev3_speaker_play_tone (NOTE_GS5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'n') {
                ev3_speaker_play_tone (NOTE_A5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'j') {
                ev3_speaker_play_tone (NOTE_AS5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'm') {
                ev3_speaker_play_tone (NOTE_B5, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == ',') {
                ev3_speaker_play_tone (NOTE_C6, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == 'l') {
                ev3_speaker_play_tone (NOTE_CS6, 1000);
                bt_cmd = 0;
            }
            if (bt_cmd == '.') {
                ev3_speaker_play_tone (NOTE_D6, 1000);
                bt_cmd = 0;
            }
            clock->sleep(50);
        }
    }
    /* ここまでBAKAプログラム */

    ter_tsk(BT_TASK);
    fclose(bt);

    ext_tsk();
}

//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int32_t sonar_alert(void)
{
    static uint32_t counter = 0;
    static int32_t alert = 0;

    int32_t distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = sonarSensor->getDistance();
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
static void tail_control(int32_t angle)
{
    int pwm = (int)pid_tail.calcControl(angle - tailMotor->getCount()); /* PID制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    tailMotor->setPWM(pwm);
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
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
        case 'l':                         // XXX :BAKA使用時にはコメントアウト
            bt_cmd = 1;
            break;
        case '2':
        case 'r':
            bt_cmd = 2;
            break;
        case '3':
            bt_cmd = 3;
            break;
        case '6':
            bt_cmd = 6;
            break;
        case '7':
            bt_cmd = 7;
            break;
        case 'u':   // 上
        case '[':
            bt_cmd = 'u';
            break;
        case 'd':   // 下                            // XXX :BAKA使用時にはコメントアウト
        case ']':
            bt_cmd = 'd';
            break;

        // case '?':                                       // TODO :BAKA
            // bt_cmd = '?';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case '-':                                       // TODO :BAKA
            // bt_cmd = '-';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'z':                                       // TODO :BAKA
            // bt_cmd = 'z';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 's':                                       // TODO :BAKA
            // bt_cmd = 's';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'x':                                       // TODO :BAKA
            // bt_cmd = 'x';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'd':                                       // TODO :BAKA
            // bt_cmd = 'd';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'c':                                       // TODO :BAKA
            // bt_cmd = 'c';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'v':                                       // TODO :BAKA
            // bt_cmd = 'v';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'g':                                       // TODO :BAKA
            // bt_cmd = 'g';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'b':                                       // TODO :BAKA
            // bt_cmd = 'b';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'h':                                       // TODO :BAKA
            // bt_cmd = 'h';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'n':                                       // TODO :BAKA
            // bt_cmd = 'n';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'j':                                       // TODO :BAKA
            // bt_cmd = 'j';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'm':                                       // TODO :BAKA
            // bt_cmd = 'm';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case ',':                                       // TODO :BAKA
            // bt_cmd = ',';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case 'l':                                       // TODO :BAKA
            // bt_cmd = 'l';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA
        // case '.':                                       // TODO :BAKA
            // bt_cmd = '.';                                       // TODO :BAKA
            // break;                                       // TODO :BAKA

        default:
            break;
        }
        if (!(bt_cmd == 'u' || bt_cmd == 'd')) {    // TODO uとdのときはエコーバックしないようにしたい。未完成
            fputc(c, bt); /* エコーバック */
        }
    }
}
