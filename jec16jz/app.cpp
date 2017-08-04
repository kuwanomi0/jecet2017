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
#define RGB_NULL              7  /* 何もないときのセンサの合計 */
#define PIDX                  1  /* PID倍率 */
#define FORWARD_X           0.95
#define KLP                 0.6  /* LPF用係数*/

/* 超音波センサーに関するマクロ */
#define SONAR_ALERT_DISTANCE 20  /* 超音波センサによる障害物検知距離[cm] */

/* 尻尾に関するマクロ */
#define TAIL_ANGLE_STAND_UP   96 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_ROKET     101 /* ロケットダッシュ時の角度[度] */
#define TAIL_ANGLE_DRIVE       3 /* バランス走行時の角度[度] */
#define TAIL_ANGLE_STOP       75 /* 停止処理時の角度[度] */
#define KP_TAIL            1.20F /* 尻尾用定数P */
#define KI_TAIL            0.01F /* 尻尾用定数I */
#define KD_TAIL             1.0F /* 尻尾用定数D */
#define PWM_ABS_MAX           60 /* 完全停止用モータ制御PWM絶対最大値 */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static int32_t sonar_alert(void);
static void tail_control(int32_t angle);
static void carHorn(void);
static void run_result(void);

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
PID pid_walk(      0,       0,       0); /* 走行用のPIDインスタンス */
PID pid_tail(KP_TAIL, KI_TAIL, KD_TAIL); /* 尻尾用のPIDインスタンス */
Distance distance_way;

/* Lコース */
static Course gCourseL[] {  // TODO 2: コース関連 だいぶ改善されました これで30.36secでた。
    { 0,     0,122,  0, 0.0500F, 0.0000F, 1.2000F }, //スタート
    { 1,  2000,112,  0, 0.1400F, 0.0002F, 2.1000F }, //大きく右
    { 2,  3927,115,  0, 0.1150F, 0.0002F, 1.7000F }, //左
    { 3,  4754,121,  0, 0.0700F, 0.0000F, 1.6000F }, //直
    { 4,  5209,115,  0, 0.1150F, 0.0002F, 1.8000F }, //左
    { 5,  6134,122,  0, 0.0800F, 0.0000F, 1.6000F }, //直
    { 6,  6674,115,  0, 0.1300F, 0.0002F, 2.0000F }, //左
    { 7,  7562,112,  0, 0.1400F, 0.0002F, 2.0000F }, //右
    { 8,  8800,122,  0, 0.0450F, 0.0000F, 1.6000F }, //直GOOLまで
    { 9, 10030,122,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    {10, 10351,100,  0, 0.1150F, 0.0002F, 1.5000F }, //左
    {11, 11576, 50,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    {12, 11766, 50,  0, 0.0500F, 0.0000F, 1.0000F }, //直
    {13, 11946,  5,  0, 0.0500F, 0.0000F, 1.0000F }, //直
    {14, 12200,100,  0, 0.1500F, 0.0000F, 1.3000F }, //直
    {15, 12450,  5,  0, 0.1500F, 0.0000F, 1.3000F }, //直
    {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
};

/* Rコース */
static Course gCourseR[]  {  //TODO :2 コース関連 だいぶ改善されました これで31.25secでた
    { 0,     0,122,  0, 0.0500F, 0.0000F, 1.0000F }, //スタート
    { 1,  2200,106,  0, 0.1200F, 0.0002F, 1.4900F }, //大きく右
    { 2,  3700,106,  0, 0.1000F, 0.0001F, 1.5100F }, //大きく右
    { 3,  5400,108,  0, 0.1200F, 0.0001F, 1.3000F }, //左やや直進
    { 4,  6350,105,  0, 0.1260F, 0.0002F, 1.4500F }, //強く左
    { 5,  7150,106,  0, 0.1100F, 0.0002F, 1.5000F }, //緩やかに大きく右
    { 6,  8750,122,  0, 0.0500F, 0.0000F, 1.0000F }, //直GOOLまで
    { 7, 10400,110,  2, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
    { 8, 10475, 10,  0, 0.0000F, 0.0000F, 0.0000F }, //直GOOLまで
    { 9, 10550,106,  0, 0.1200F, 0.0002F, 1.5000F }, //左
    {10, 11900, 70,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    {11, 12150, 50,  0, 0.1200F, 0.0002F, 0.6000F }, //直?
    {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
};
/* 階段用コースファイル */
static Course gCourseKaidan[] {  // TODO 2: コース関連 だいぶ改善されました これで30.36secでた。
    { 1,     0, 20,  0, 0.0000F, 0.0000F, 0.0000F }, //灰
    { 2,   140, 20,  0, 0.1150F, 0.0002F, 1.0000F }, //前
    { 3,   425, 20,  0, 0.1150F, 0.0002F, 1.0000F }, //初段
    { 4,   750,  5,  0, 0.1150F, 0.0002F, 1.0000F }, //二段
    { 5,   800,  5,  0, 0.2000F, 0.0002F, 2.0000F }, //左
    {99, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F }  //終わりのダミー
};

/* デフォルト */
static Course gCourse[] {
    { 0,     0, 50,  0, 0.1000F, 0.0001F, 1.0000F }, //スタート
    { 1, 99999,  1,  0, 0.0000F, 0.0000F, 0.0000F } //終わりのダミー
};

// サウンド
#define NOTE_C4 (261.63)
#define NOTE_B6 (1975.53)
#define MYSOUND_MANUAL_STOP (100)
#define VOLUME 50
#define TONE NOTE_C4
// ファンファーレ
// memfile_t memfile;
// ev3_memfile_load("fa01101.wav", &memfile);

//　タイム格納用
static int time[100];
static int lapTime_count = 0;

/* メインタスク */
void main_task(intptr_t unused)
{
    int8_t    forward;      /* 前後進命令 */
    int8_t    turn;         /* 旋回命令 */
    int8_t    pwm_L, pwm_R; /* 左右モータPWM出力 */
    rgb_raw_t rgb_level;    /* カラーセンサーから取得した値を格納する構造体 */
    int course_number = 0; //TODO :2 コース関連 だいぶ改善されました
    int count = 0;  //TODO :2 コース関連 だいぶ改善されました
    int roket = 0;  //TODO :3 ロケットスタート用変数 タイマーの役割をしています
    // int tail_i = 0; //TODO :4 おまけ コマンドでの終了する際のタイマー用変数
    int forward_course = 50; //TODO :2 コース関連 だいぶ改善されました
    int turn_course = 0; //TODO :2 コース関連 だいぶ改善されました
    uint16_t rgb_total = RGB_TARGET;
    uint16_t rgb_before;
    int gyro_flag = 0;
    int distance_kaidan;
    int tail_flags = 0;
    int kaiden = 1;
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
    /* 尻尾モーターのリセット */
    for(int i = 0; i < 300; i++){
        tailMotor->setPWM(-3);
        clock->wait(1);
    }
    tailMotor->reset();

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    double angle = (double)TAIL_ANGLE_STAND_UP;
    int rotation_flag = 0;
    int run_flag = 0;
    int ctlspeed;
    while(1)
    {
        tail_control(angle); /* 完全停止用角度に制御、調整も可 */

        /* Lコース */
        if (bt_cmd == 1)
        {
            mCourse = gCourseL;
            break; /* リモートスタート */
        }
        /* Rコース */
        if (bt_cmd == 2)
        {
            mCourse = gCourseR;
            break; /* リモートスタート */
        }
        /* 階段コース */
        if (bt_cmd == 3)
        {
            mCourse = gCourseKaidan;
            break; /* リモートスタート */
        }

        if (touchSensor->isPressed())
        {
            mCourse = gCourse;
            break; /* タッチセンサが押された */
        }

        // スタート前の尻尾調整
        if (ev3_button_is_pressed(DOWN_BUTTON) || bt_cmd == ']') {
            angle -= 0.1;
            bt_cmd = 0; // コマンドリセット
        }
        if (ev3_button_is_pressed(UP_BUTTON) || bt_cmd == '[') {
            angle += 0.1;
            bt_cmd = 0; // コマンドリセット
        }
        // syslog(LOG_NOTICE, "DEBUG, angle : %d\r", (int)angle);

        // 回転
        if (bt_cmd == 9 && rotation_flag == 0) {
            syslog(LOG_NOTICE, "DEBUG, 回転\r");
            leftMotor->setPWM(-18);
            rightMotor->setPWM(17);
            bt_cmd = 0;
            rotation_flag = 1;
        }
        if (bt_cmd == 9 && rotation_flag == 1) {
            syslog(LOG_NOTICE, "DEBUG, 回転停止\r");
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            bt_cmd = 0;
            rotation_flag = 0;
        }
        // ラジコン操作
        if (bt_cmd == 't' && run_flag == 0) {
            syslog(LOG_NOTICE, "DEBUG, 前進\r");
            leftMotor->setPWM(10);
            rightMotor->setPWM(10);
            bt_cmd = 0;
            run_flag = 1;
        }
        if (bt_cmd == 't' && run_flag == 1) {
            syslog(LOG_NOTICE, "DEBUG, 前進停止\r");
            for (int i = 50; i >= 0; i--) {
                leftMotor->setPWM(i);
                rightMotor->setPWM(i);
            }
            bt_cmd = 0;
            run_flag = 0;
        }
        // ラジコン操作2
        if (bt_cmd == 'w') {    // 前進
            ctlspeed = 10;  // ラジコンのスピード
            while (ctlspeed >= 0) {
                leftMotor->setPWM(ctlspeed);
                rightMotor->setPWM(ctlspeed);
                ctlspeed--;
                // 以下遅延処理
                clock->reset();
                while (clock->now() < 500) {
                    // 遅延
                }
            }
            bt_cmd = 0;
        }
        if (bt_cmd == 's') {    // 後進
            ctlspeed = -10;  // ラジコンのスピード
            while (ctlspeed <= 0) {
                leftMotor->setPWM(ctlspeed);
                rightMotor->setPWM(ctlspeed);
                ctlspeed++;
                // 以下遅延処理
                clock->reset();
                while (clock->now() < 500) {
                    // 遅延
                }
            }
            bt_cmd = 0;
        }
        if (bt_cmd == 'd') {
            turn = 10;
            clock->reset();
            while (clock->now() < 500) {
                // 何もしない
            }
            turn = 0;
            bt_cmd = 0;
        }
        if (bt_cmd == 'a') {
            turn = -10;
            clock->reset();
            while (clock->now() < 500) {
                // 何もしない
            }
            turn = 0;
            bt_cmd = 0;
        }

        // クラクション
        if (bt_cmd == ' ') {
            carHorn();
            bt_cmd = 0;
        }

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
    {
        int32_t motor_ang_l, motor_ang_r;
        int32_t gyro, volt;
        int32_t distance_now; /*現在の走行距離を格納する変数 */

        /* バックボタンによる停止処理です */
        if (ev3_button_is_pressed(BACK_BUTTON)) {
            run_result();
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
        else if(roket++ < 25)                                              //TODO :3 ロケットスタートと呼ぶにはまだ怪しい、改良必須
            tail_control(TAIL_ANGLE_ROKET); /* ロケット走行用角度に制御 */  //TODO :3 ロケットスタートと呼ぶにはまだ怪しい、改良必須
        else if(tail_flags == 1){
            tail_control(80); /* バランス走行用角度に制御 */
        }
        else {
            tail_control(TAIL_ANGLE_DRIVE); /* バランス走行用角度に制御 */
        }

        rgb_before = rgb_total; //LPF用前処理
        colorSensor->getRawColor(rgb_level); /* RGB取得 */
        rgb_total = (rgb_level.r + rgb_level.g + rgb_level.b)  * KLP + rgb_before * (1 - KLP); //LPF

        /* 転倒時の停止処理 */
        if(rgb_total <= RGB_NULL) {
            run_result();
            break;
        }

        /* 現在の走行距離を取得 */
        distance_now = distance_way.distanceAll(leftMotor->getCount(), rightMotor->getCount());

        /* 区間変更を監視、行うプログラム */
        if (distance_now >= mCourse[count].getDis()) {      //TODO :2 コース関連 だいぶ改善されました
            course_number  = mCourse[count].getCourse_num();      //TODO :2 コース関連 だいぶ改善されました
            forward_course = mCourse[count].getForward();      //TODO :2 コース関連 だいぶ改善されました
            turn_course    = mCourse[count].getTurn();
            pid_walk.setPID(mCourse[count].getP() * PIDX, mCourse[count].getI() * PIDX, mCourse[count].getD() * PIDX);      //TODO :2 コース関連 だいぶ改善されました
            count++;      //TODO :2 コース関連 だいぶ改善されました
        }      //TODO :2 コース関連 だいぶ改善されました

        if (sonar_alert() == 1) {/* 障害物検知 */
            forward = turn = 0; /* 障害物を検知したら停止 */
            ev3_led_set_color(LED_RED);
            syslog(LOG_NOTICE, "障害物検知\r");
            carHorn();
            for (int i = 0; i < TAIL_ANGLE_STOP; i++) {
                tail_control(i);
                clock->sleep(4);
            }

            // balancer.setCommand(forward, turn);   // <1>
            // balancer.update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
            // pwm_L = balancer.getPwmRight();       // <3>
            // pwm_R = balancer.getPwmLeft();        // <3>
            //
            // leftMotor->setPWM(pwm_L);
            // rightMotor->setPWM(pwm_R);
            leftMotor->setPWM(-10);
            rightMotor->setPWM(-10);
            clock->sleep(1000);
            leftMotor->setPWM(0);
            rightMotor->setPWM(0);
            while (1) {
                // syslog(LOG_NOTICE, "停止中？\r");
            }
        }
        else {
            if (bt_cmd == 7 || bt_cmd == 6) //TODO 4: おまけコマンド停止処理用
            {
                forward = -30; //TODO 4: おまけコマンド停止処理用
            }
            else {
                forward = forward_course * FORWARD_X; /* 前進命令 */
            }
            /* PID制御 */
            turn =  pid_walk.calcControl(RGB_TARGET - rgb_total) + turn_course;
        }

        /* 倒立振子制御API に渡すパラメータを取得する */
        motor_ang_l = leftMotor->getCount();
        motor_ang_r = rightMotor->getCount();
        gyro = gyroSensor->getAnglerVelocity();
        volt = ev3_battery_voltage_mV();

        // TODO :KAIDAN
        if ((gyro >= 100 || gyro_flag >= 1) && roket >= 45) {
            gyro_flag++;
            if(gyro_flag <= 750) {
                forward = -5;
            }
            else if (gyro_flag <= 1500) {
                forward = 0;
            }
            else if (gyro_flag <= 2250 && kaiden == 1) {
                forward = 0;
                turn = -26;
            }
            else if (gyro_flag <= 2000 && kaiden == 2) {
                forward = 0;
                turn = -26;
            }
            // else if (gyro_flag <= 2000) {
            //     forward = -5;
            //     turn = -turn;
            // }
            else if (gyro_flag <= 3000) {
                forward = 2;
            }
            else if (kaiden == 1) {
                gyro_flag = 0;
                kaiden = 2;
            }
            else {
                gyro_flag = 0;
            }
        }
        // if ((gyro >= 100 || gyro_flag >= 1) && roket >= 45) {
        //     syslog(LOG_NOTICE, "wwwGwwwYwwwRwwwOwww\r");
        //     if(gyro_flag >= 500 && distance_now <= distance_kaidan) {
        //
        //         syslog(LOG_NOTICE, "GYRO GYRO GYRO GYRO !!!\r");
        //         forward = 70;
        //     }
        //     else if (gyro_flag >= 500) {
        //         syslog(LOG_NOTICE, "ああああああああああああああああああああああああああああああああああああ\r");
        //         forward = 0;
        //         tail_flags = 1;
        //     }
        //     else {
        //         syslog(LOG_NOTICE, "FLAG !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r");
        //         gyro_flag++;
        //         forward = -20;
        //         distance_kaidan = distance_now + 150;
        //     }
        // }

        // if (gyro_flag >= 500) {
        //
        // }
        /* 倒立振子制御APIを呼び出し、倒立走行するための */
        /* 左右モータ出力値を得る */
        balancer.setCommand(forward, turn);   // <1>
        balancer.update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
        pwm_L = balancer.getPwmRight();       // <3>
        pwm_R = balancer.getPwmLeft();        // <3>

        leftMotor->setPWM(pwm_L);
        rightMotor->setPWM(pwm_R);


        /* ログを送信する処理　*/
        // syslog(LOG_NOTICE, "D:%5d, G:%3d\r", distance_now, gyro);
        syslog(LOG_NOTICE, "D:%5d, G:%3d, flag:%5d, RGB%3d\r", distance_now, gyro, gyro_flag, rgb_total);
        // if (bt_cmd == 1)
        // {
        //     syslog(LOG_NOTICE, "DEBUG, DIS:%5d, GYRO:%3d, C:%2d, F:%3d\r", distance_now, gyro, course_number, forward);
        //     bt_cmd = 0;
        // }

        // if (course_number % 2 != 0) {
        //     ev3_speaker_play_tone (100, 1000);
        // }
        // else {
        //     ev3_speaker_play_tone (400, 1000);
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

        if (bt_cmd == ' ') {
            carHorn();
            bt_cmd = 0;
        }

        clock->sleep(4); /* 4msec周期起動 */
    }
    leftMotor->reset();
    rightMotor->reset();
    tailMotor->reset();

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
        case 'l':
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
        case '9':
            bt_cmd = 9;
            break;
        case ' ':
            bt_cmd = ' ';
            break;
        case '[':   // 上
            bt_cmd = '[';
            break;
        case ']':   // 下
            bt_cmd = ']';
            break;
        case 't':
            bt_cmd = 't';
            break;
        case 'w':
            bt_cmd = 'w';
            break;
        case 's':
            bt_cmd = 's';
            break;
        case 'd':
            bt_cmd = 'd';
            break;
        case 'a':
            bt_cmd = 'a';
            break;
        case '\r':
            bt_cmd = '\r';
            break;
        default:
            break;
        }
        if (!(bt_cmd == '[' || bt_cmd == ']' || bt_cmd == ' ')) {    // TODO uとdのときはエコーバックしないようにしたい。未完成
            fputc(c, bt); /* エコーバック */
        }
        if (bt_cmd == 1 || bt_cmd == 2) {
            clock->reset();
        }
        else if (bt_cmd == '\r') {
            syslog(LOG_NOTICE, "DEBUG, TIME : //////////////////////////////\r");
            syslog(LOG_NOTICE, "DEBUG, TIME : %d.%03d s\r", clock->now() / 1000, clock->now() % 1000);
            syslog(LOG_NOTICE, "DEBUG, TIME : //////////////////////////////\r");
            time[lapTime_count++] = clock->now();
        }
    }
}

//*****************************************************************************
// 関数名 : carHorn
// 引数 : unused
// 返り値 : なし
// 概要 : クラクションを鳴らす
//*****************************************************************************
static void carHorn() {
    ev3_speaker_set_volume(VOLUME);
    //ev3_speaker_play_file(&memfile, MYSOUND_MANUAL_STOP);
    ev3_speaker_play_tone(TONE, MYSOUND_MANUAL_STOP);
}

//*****************************************************************************
// 関数名 : run_result
// 引数 : unused
// 返り値 : なし
// 概要 : 走行結果を表示する
//*****************************************************************************
static void run_result() {
    syslog(LOG_NOTICE, "DEBUG, TIME --------------------\r");
    for (int i = 0; i < lapTime_count; i++) {
        syslog(LOG_NOTICE, "DEBUG, TIME(%3d) : %d.%03d s\r",i + 1 , time[i] / 1000, time[i] % 1000);
    }
    syslog(LOG_NOTICE, "DEBUG, TIME --------------------\r");
}
