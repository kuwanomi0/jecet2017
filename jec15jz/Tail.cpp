#include "Tail.h"

Tail::Tail()
{
    data1 = 0;
    data2 = 0;
    dev = 0;
    p = 0;
    target = 0;
    mClock = new Clock();
    mTailMotor = new Motor(PORT_A);

}

Tail::~Tail()
{

}

//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モータ目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モータの角度制御
//*****************************************************************************
void Tail::tail_control(int32_t angle)
{
	data1 = dev;
	data2 = p;
	dev = TAIL_ANGLE_STAND_UP - mTailMotor->getCount();
	p = dev - data1;
    float pwm = (float)(angle - mTailMotor->getCount()) * P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    mTailMotor->setPWM(pwm);
}
void Tail::tail(double angle)
{
	data1 = dev;
	data2 = p;
	dev = angle - mTailMotor->getCount();
	p = dev - data1;
    //float pwm = (float)(angle - mTailMotor->getCount()) * P_GAIN; /* 比例制御 */
	float pwm = 2 * dev + 0.1 * (p - data2) + 10 * p;
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    mTailMotor->setPWM(pwm);
}

void Tail::initTail()
{
    /* 尻尾モーターのリセット */
    for(int i = 0; i < 300; i++){
  		mTailMotor->setPWM(-1);
  		mClock->wait(3);
    }
    mTailMotor->reset();
}

void Tail::standTail(TouchSensor* mTouchSensor, ColorSensor* colorSensor, int32_t* bt_cmd )
{
    double angle = TAIL_ANGLE_STAND_UP;
    /* スタート待機 */ /* 完全停止用角度に制御 */
    while(1)
    {
        tail(angle);

        if(ev3_button_is_pressed(BACK_BUTTON))
        {
            break;
        }

        colorSensor->getRawColor(colorRgb);
        target = (colorRgb.r + colorRgb.g + colorRgb.b) / 3;
        syslog(LOG_NOTICE, "r : %d, g : %d, b : %d, target : %d\r",colorRgb.r, colorRgb.g, colorRgb.b, target);
        if( target >= TARGET - 5 && target <= TARGET + 5)
        {
            ev3_speaker_play_tone(NOTE_D6, 1);
        }

        if (*bt_cmd != -1)
        {
            break; /* リモートスタート */
        }
        if (ev3_button_is_pressed(DOWN_BUTTON)) {
    		angle = angle - 0.1;
    	}
    	if (ev3_button_is_pressed(UP_BUTTON)) {
    		angle = angle + 0.1;
    	}
        mClock->sleep(10);
    }
}
