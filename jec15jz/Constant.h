/*編集項目*/
#define FORWARD 100  /*最高前進速度 127 最高後進速度 -127と考える*/

#define TARGET 100

#define KI1     0.025F
#define KD1     16.0F
#define KI2     0.069F
#define KD2     14.5F
#define KI2_2   0.068F
#define KD2_2   12.5F
#define KI3     0.063F
#define KD3     13.5F
#define KI4     0.107F
#define KD4     16.0F
//#define KI5     0.13F
//#define KD5     15.0F
#define KI5     0.075F
#define KD5     14.0F
#define KI6     0.04F
#define KD6     16.0F
//KI2とKD2をかえないと


#define KIL1     0.02F
#define KDL1     16.0F
#define KIL1t     0.01F
#define KDL1t     8.0F
//#define KIR2     0.1F　最初
//#define KDR2     10.0F　最初
#define KIL2     0.072F
#define KDL2     14.5F
#define KIL2_D     0.1F
#define KDL2_D    13.0F
#define KIL2_D_2     0.074F
#define KDL2_D_2     14.0F
#define KIL3     0.06F
//#define KIR3_3     0.07F
#define KDL3     16.0F
#define KIL3_2   0.074F
#define KDL3_2   14.5F
#define KIL4     0.05F
#define KDL4     15.0F
#define KIL4_2     0.065F
#define KDL4_2     14.0F
#define KIL5     0.065F
#define KDL5     14.0F
#define KIL6     0.05F
#define KDL6     16.0F

#define KP      0.33F/*比例係数 変数最後記述 F はfloat型を表す おそらくkp最適は0.33前後*/
#define KI      0.05F /*積分係数 */
#define KD      15.0F /*微分係数 */
//#define KI2     0.14F
//#define KD2     16.0F


#define KIc     0.14F                /*サンプル第３コーナー*/
#define KDc     17.5F
#define KIc2    0.14F                /*サンプル第4コーナー*/
#define KDc2    17.0F



#define LINERS   150
#define LINER1   2000
#define LINER1_D 2100
//#define LINER2_1   3300
#define LINER2_2   4680
//#define LINER2_2   6030
#define LINER3   6540
#define LINER4   7160
#define LINER5   8310
#define LINER6   10120
#define LINER7   10270


#define LINELS     200
#define LINEL1     1930
#define LINEL1_D   2030
#define LINEL2     3170
#define LINEL2_D   3260
#define LINEL2_DD  3360
#define LINEL2_D_2 3460
#define LINEL3     4720
#define LINEL4     6275
#define LINEL4_2   7350
#define LINEL5     8500
#define LINEL6     9955
#define LINEL7     10065

/*サンプルコースforward値算出*/
#define v1   500
#define v2   380
#define v3   290
#define v4   280
#define vR4  320
//#define v5   360
//#define v6   340
#define v5   290
#define v6   290
#define v7   200
#define a1   3
#define a2   1.2  //アクセル定数
#define a3   1.5
#define a4   4
#define a5   1.5




#define COURSE_DEFAULT 0;

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
#define GYRO_OFFSET           0  /* ジャイロセンサオフセット値(角速度0[deg/sec]時) +のとき前のめり*/
#define LIGHT_WHITE          40  /* 白色の光センサ値 */
#define LIGHT_BLACK           0  /* 黒色の光センサ値 */
#define SONAR_ALERT_DISTANCE 30  /* 超音波センサによる障害物検知距離[cm] */
#define TAIL_ANGLE_STAND_UP  96  /* 完全停止時の角度[度] */
#define TAIL_ANGLE_DRIVE      3  /* バランス走行時の角度[度] */
#define TAIL_ANGLE_START    100  /* スタートする時の尻尾の角度 */
#define P_GAIN             2.5F  /* 完全停止用モータ制御比例係数 */
#define PWM_ABS_MAX          80  /* 完全停止用モータ制御PWM絶対最大値 */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 hrp2/target/ev3.h BLUETOOTH_LOCAL_NAMEで設定 */
//#define PASS_KEY        "1234" /* パスキー    hrp2/target/ev3.h BLUETOOTH_PIN_CODEで設定 */
#define CMD_START         '1'    /* リモートスタートコマンド */
#define PAI 3.14F


/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6/*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8/*TODO: magic number*/)
