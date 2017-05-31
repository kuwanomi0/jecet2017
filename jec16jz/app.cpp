/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 2輪倒立振子ライントレースロボットのTOPPERS/HRP2用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "balancer.h"
#include "BalancerCpp.h"        // <1>
#include "PID.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#ifdef DEBUG
#define _debug(x) (x)
#else
#define _debug(x)
#endif

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor    = EV3_PORT_1,
    sonar_sensor    = EV3_PORT_2,
    color_sensor    = EV3_PORT_3,
    gyro_sensor     = EV3_PORT_4;

static const motor_port_t
    left_motor      = EV3_PORT_C,
    right_motor     = EV3_PORT_B,
    tail_motor      = EV3_PORT_A;

static int      bt_cmd = 0;     /* Bluetoothコマンド 1:リモートスタート */
static FILE     *bt = NULL;     /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* 走行に関するマクロ */
#define GYRO_OFFSET      0      /* ジャイロセンサオフセット値(角速度0[deg/sec]時) */
#define RGB_WHITE      630      /* 白色のRGBセンサ合計値 */
#define RGB_BLACK       30      /* 黒色のRGBセンサ合計値*/
#define RGB_TARGET     460      /* 中央の境界線のRGBセンサ合計値 */
#define RGB_NULL         5      /* 何もないときのRGBセンサ合計値 */
#define KP_WALK      0.11F      /* 走行用定数P */
#define KI_WALK      0.001F      /* 走行用定数I */
#define KD_WALK      0.0005F      /* 走行用定数D */

/* 超音波センサーに関するマクロ */
#define SONAR_ALERT_DISTANCE 20 /* 超音波センサによる障害物検知距離[cm] */

/* 尻尾に関するマクロ */
#define TAIL_ANGLE_STAND_UP   92 /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE       3 /* バランス走行時の角度[度] */
#define KP_TAIL            2.00F /* 尻尾用定数P */
#define KI_TAIL            0.01F /* 尻尾用定数I */
#define KD_TAIL            0.15F /* 尻尾用定数D */
#define PWM_ABS_MAX           60 /* 完全停止用モータ制御PWM絶対最大値 */

/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/* magic number*/)
#define CALIB_FONT_HEIGHT (8/* magic number*/)

/* 関数プロトタイプ宣言 */
static int sonar_alert(void);
static void tail_control(signed int angle);

/* インスタンスの生成 */
Balancer balancer;              // <1>
PID pid_walk(KP_WALK, KI_WALK, KD_WALK); /* 走行用のPIDインスタンス */
PID pid_tail(KP_TAIL, KI_TAIL, KD_TAIL); /* 尻尾用のPIDインスタンス */

/* メインタスク */
void main_task(intptr_t unused)
{
    signed char forward;      /* 前後進命令 */
    signed char turn;         /* 旋回命令 */
    signed char pwm_L, pwm_R; /* 左右モータPWM出力 */
    rgb_raw_t   rgb_level;    /* カラーセンサーから取得した値を格納する構造体 */

    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    ev3_lcd_draw_string("EV3way-ET JEC16JZ", 0, CALIB_FONT_HEIGHT*1);

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb_level);
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
    ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
    /* モーター出力ポートの設定 */
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);
    ev3_motor_config(tail_motor, LARGE_MOTOR);
    ev3_motor_reset_counts(tail_motor);

    /* Open Bluetooth file */
    bt = ev3_serial_open_file(EV3_SERIAL_BT);
    assert(bt != NULL);

    /* Bluetooth通信タスクの起動 */
    act_tsk(BT_TASK);

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    /* スタート待機 */
    while(1)
    {
        tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */

        if (bt_cmd == 1)
        {
            break; /* リモートスタート */
        }

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; /* タッチセンサが押された */
        }

        tslp_tsk(10); /* 10msecウェイト */
    }

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    /* ジャイロセンサーリセット */
    ev3_gyro_sensor_reset(gyro_sensor);
    balancer.init(GYRO_OFFSET);                // <1>

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /**
    * Main loop for the self-balance control algorithm
    */
    while (1) {
    int32_t motor_ang_l, motor_ang_r;
    int gyro, volt;

    if (ev3_button_is_pressed(BACK_BUTTON)) {
    break;
    }

    ev3_led_set_color(LED_RED);
    tail_control(TAIL_ANGLE_DRIVE); /* 尻尾を走行位置に制御 */

    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb_level); /* RGB取得 */

    /* 転倒時の停止処理 */
    if((rgb_level.r + rgb_level.g + rgb_level.b) <= RGB_NULL) {
    break;
    }

    if (sonar_alert() == 1) {/* 障害物検知 */
        forward = turn = 0; /* 障害物を検知したら停止 */
    }
    else {
        forward = 80;        /* ロボの速度 */
        /* PID制御 */
        //  turn =  pid_walk.calcControl(((RGB_BLACK + RGB_WHITE) / 2) - (rgb_level.r + rgb_level.g + rgb_level.b));
        turn =  pid_walk.calcControl(RGB_TARGET - (rgb_level.r + rgb_level.g + rgb_level.b));
    }

    /* 倒立振子制御API に渡すパラメータを取得する */
    motor_ang_l = ev3_motor_get_counts(left_motor);
    motor_ang_r = ev3_motor_get_counts(right_motor);
    gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
    volt = ev3_battery_voltage_mV();

    /* 倒立振子制御APIを呼び出し、倒立走行するための */
    /* 左右モータ出力値を得る */
    balancer.setCommand(forward, turn);   // <1>
    balancer.update(gyro, motor_ang_r, motor_ang_l, volt); // <2>
    pwm_L = balancer.getPwmRight();       // <3>
    pwm_R = balancer.getPwmLeft();        // <3>

    /* 出力の設定 */
    ev3_motor_set_power(left_motor, (int)pwm_L);
    ev3_motor_set_power(right_motor, (int)pwm_R);

    tslp_tsk(4); /* 4msec周期起動 */
    }

    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

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
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
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
static void tail_control(signed int angle)
{
    int pwm = (int)pid_tail.calcControl(angle - ev3_motor_get_counts(tail_motor)); /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
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
        uint8_t c = fgetc(bt); /* 受信 */
        switch(c)
        {
        case '1':
            bt_cmd = 1;
            break;
        default:
            break;
        }
        fputc(c, bt); /* エコーバック */
    }
}
