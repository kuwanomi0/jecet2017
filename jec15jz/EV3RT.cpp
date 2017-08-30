#include "EV3RT.h"

EV3RT::EV3RT(int32_t* bt_cmd)
{
    /* 各オブジェクトを生成・初期化する */
    mTouchSensor  = new TouchSensor(PORT_1);
    mColorSensor  = new ColorSensor(PORT_3);
    mSonarSensor  = new SonarSensor(PORT_2);
    mGyroSensor   = new GyroSensor(PORT_4);
    mLeftMotor    = new Motor(PORT_C);
    mRightMotor   = new Motor(PORT_B);
    tail          = new Tail();
    mClock        = new Clock();
    mClock2       = new Clock();
    mClock3       = new Clock();
    sysLog        = new SysLog();
    calculation   = new Calculation(mColorSensor);

    time         = 0;
    sectionPlace = 1;
    forward      = 0;
    turn         = 0;
    adistance    = 0;
    direction    = 0;
    v            = 0;
    pwmL         = 0;
    pwmR         = 0;
    b_stop       = false;
    kaitenkaku   = 0;
    pointDirection      = 0;
    pointRightDirection = 0;

    courseDistance = 0;
    addTarget      = 0;
    stairAdistance = 0;

    this->bt_cmd = bt_cmd;
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);
    mColorSensor->getRawColor(colorRgb);//LEDの初期カラーをRGBに
    colorRgb.b = 5;

    tail->initTail();
}

EV3RT::~EV3RT()
{
    delete mTouchSensor;
    delete mColorSensor;
    delete mSonarSensor;
    delete mGyroSensor;
    delete mLeftMotor;
    delete mRightMotor;
    delete tail;
    delete mClock;
    delete mClock2;
    delete mClock3;
    delete sysLog;
    delete calculation;
}

//ロボの運転
void EV3RT::driving(SectionData* sData)
{
    if(!b_stop)
    {
        this->v = sData->getV();

        /* 区間変更時ロボのLEDのカラーをセット */
        ev3_led_set_color(sData->getLedColor());

        /* 区間変更時、時間を0に */
        mClock3->reset();

      //区間変更時目標速度が0じゃない場合差分を使うので
      //目標速度を条件に使用している（応急処置）
        if(sData->getV() > 100)
        {
            //右モータの回転角と左モータの回転角
            adistance = calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount());//差分
        }
        else
        {
            adistance = 0;
        }
        //走行ループ
        while (direction < sData->getEndLine())
        {
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                sprintf(buf, "colorRgb.b : %d", colorRgb.b);
                ev3_lcd_draw_string(buf, 0, 0);
                break;
            }
            tail->tail_control(TAIL_ANGLE_DRIVE);

          //差分とおなじく目標速度を｢指定しない場合forwardはきめうちなので分岐（応急処置）
            forward = calculation->forwardCalculation
                      (
                          sData->getV(),           //目標速度(決めうちの場合は速度はここの値に入る)
                          adistance,               //差分
                          mClock3->now(),          //区間ごとの現在の走行時間
                          mRightMotor->getCount(), //右モータの回転角
                          mLeftMotor->getCount(),  //左モータの回転角
                          sData->getAccel()        //アクセル定数
                      );
            //sysLog->logWrite("forward", (int)forward);
            turn    = calculation->leftDirection
                      (
                          sData->getP(),           //PIDのP値
                          sData->getI(),           //PIDのI値
                          sData->getD(),           //PIDのD値
                          sData->getTarget(),      //ターゲット値
                          colorRgb
                      );
            balancer_control(forward, turn);
            mClock->sleep(4);
        }

        /*ロボ本体のスクリーン上に区間タイムを表示*/
        time += (int)mClock3->now();                   //この区間でかかった時間を加算
        sprintf(buf, "section : %d", time);
        ev3_lcd_draw_string(buf, 0, 10 + sectionPlace * 10);
        sectionPlace++;                     //次の区間へ
    }
}

//balance_controlを使う場合
void EV3RT::stairDrive(StairDriveData* sData)
{
    mClock2->reset();

    switch(sData->getStairType())
    {
    case DISTANCE://走行した距離で判定
        while(calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount()) < courseDistance + sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);
        }
        break;
    case ADISTANCE://走行した距離の差分で判定
        //courseDistance変数は難所に入る前にsetCourseDistanceメソッドが呼び出され、難所までの走行距離が保存されている
        adistance = calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount()) - courseDistance + sData->getTermsValue();
        while(calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount()) - courseDistance < adistance)
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);
        }
        break;
    case TIME://時間で指定
        while(mClock2->now() < (unsigned int)sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
    		stairDriving(sData);
    	}
        break;
    case COLOR_R1://colorRgb.rの値が条件値より小さい場合
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.r < sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        break;
    case COLOR_R2://colorRgb.rの値が条件値より大きい場合
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.r > sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        break;
    case COLOR_B1://colorRgb.bの値が条件値より小さい場合
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.b < sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        target = colorRgb.b + 10;
        break;
    case COLOR_B2://colorRgb.rの値が条件値より大きい場合
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.b > sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        target = 100;
        break;
    case L_KAITENKAKU:
        break;
    case R_KAITENKAKU:
        break;
    case ANGLE://角度で判定
        while(mGyroSensor->getAnglerVelocity() < sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //ライントレース、balance_controlを用いた階段走行
            stairDriving(sData);
        }
    default:
        break;
    }
}

//balance_controlを使わず直接タイヤに値を渡す場合
void EV3RT::stairForce(StairForceData* sData)
{
    mClock2->reset();

    switch(sData->getStairType())
    {
    case DISTANCE:
        break;
    case ADISTANCE:
        break;
    case TIME://時間で判定
        while(mClock2->now() < (unsigned int)sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);
        }
        break;
    case COLOR_R1:
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.r < sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        break;
    case COLOR_R2:
        mColorSensor->getRawColor(colorRgb);
        while (colorRgb.r > sData->getTermsValue())
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);

            mColorSensor->getRawColor(colorRgb);
        }
        break;
    case COLOR_B1:
        break;
    case COLOR_B2:
        break;
    case L_KAITENKAKU://左モータの回転数で判定
        kaitenkaku = mLeftMotor->getCount() + sData->getTermsValue();
        while(mLeftMotor->getCount() <= kaitenkaku)
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下ならｓ
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);
        }
        break;
    case R_KAITENKAKU://右モータの回転数で判定
        kaitenkaku = mRightMotor->getCount() + sData->getTermsValue();
        while(mRightMotor->getCount() <= kaitenkaku)
        {
            //バックボタンが押されるまたはcolorRgb.bの値が2以下なら
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);
        }
        break;
    case ANGLE:
        break;
    case LOOP://無限ループ
        while(1)
        {
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                break;
            }

            //タイヤに直接データを渡して走行
            stairForcing(sData);
        }
    default:
        break;
    }
}

//尻尾の上げ下げ
void EV3RT::stairTail(StairTailData* rData)
{
    switch(rData->getTailType())
    {
    case T_UP://角度を加算
        for(int angle = rData->getStartAngle(); angle <= rData->getEndAngle(); angle++)
        {
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                return;
            }
    		if(angle >= rData->getEndAngle()){
    			tail->tail(angle); /* バランス走行用角度に制御 */
    			forward = 0;
    			turn    = 0;
    			balancer_control(forward, turn);
    			//logData->writeData(mClock->now(), motor_ang_l, motor_ang_r, gyro, volt, direction, forward, turn, pwm_L, pwm_R);
    	    	mClock->sleep(4); /* 4msec周期起動 */
    		}else{
    			mLeftMotor->setPWM(rData->getFalsePwm());
    			mRightMotor->setPWM(rData->getFalsePwm());
    			mClock->reset();
    			while(mClock->now() <= (unsigned int)rData->getTime())
                {
    				tail->tail(angle);
    				mClock->sleep(4);
    			}
    		}
    	}
        break;
    case T_DOWN://角度を減算
        for(int angle = rData->getStartAngle(); angle >= rData->getEndAngle(); angle--)
        {
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2) //|| mGyroSensor->getAnglerVelocity() > 80 || mGyroSensor->getAnglerVelocity() < -80)
            {
                return;
            }
            if(angle >= rData->getStartAngle()){
                mLeftMotor->setPWM(rData->getTruePwm());
                mRightMotor->setPWM(rData->getTruePwm());
            }else{
                mLeftMotor->setPWM(rData->getFalsePwm());
                mRightMotor->setPWM(rData->getFalsePwm());
          }
          mClock->reset();
          while(mClock->now() <= (unsigned int)rData->getTime()){
            tail->tail(angle);
            mClock->sleep(4);
          }
      }
      break;
    }
}

//ルックアップゲートの走行
void EV3RT::lookupDrive(LookupData* lData)
{
    mClock3->reset();

    ev3_led_set_color(lData->getLedColor());

    if (lData->getMode() == 3)
    {
        pointRightDirection =
            calculation->rightDistance(mRightMotor->getCount());
    }

    // 5: 後ろ向きに走りながら尻尾をあげる
    if (lData->getMode() == 5)
    {
        // 走行ループ
        while (calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount()) - courseDistance > lData->getEndLine())
        {
            //mColorSensor->getRawColor(colorRgb);
            if (ev3_button_is_pressed(BACK_BUTTON) || colorRgb.b < 2)
            {
                break;
            }

            mLeftMotor->setPWM(-10);
            mRightMotor->setPWM(-10);

            tail->tail(lData->getAngle());

            // 距離について
            int32_t motor_ang_l = mLeftMotor->getCount();
            int32_t motor_ang_r = mRightMotor->getCount();
            direction = calculation->directionAll(motor_ang_l, motor_ang_r);
        }
    }
    else {
        //走行ループ
        while (calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount()) - courseDistance < lData->getEndLine())
        {
            // 1: 尻尾を下げながら走行する
            if (lData->getMode() == 1)
            {
                mLeftMotor->setPWM(10);
                mRightMotor->setPWM(10);
            }
            // 0: 灰色までライントレース
            // 2: ルックアップゲート20cm手前までライントレースする
            // 4: 指定距離をライントレースする
            // 6: 灰色上を黒色になるまでライントレース
            else if (lData->getMode() == 0 || lData->getMode() == 2 || lData->getMode() == 4 || lData->getMode() == 6)
            {
                mColorSensor->getRawColor(colorRgb);

                if (lData->getMode() == 0 && colorRgb.b > 40)
                {
                    courseDistance = direction;
                    break;
                }

                if (lData->getMode() == 6 && colorRgb.b < 50)
                {
                    courseDistance = direction;
                    break;
                }

                if (lData->getMode() == 2 && mSonarSensor->getDistance() < 20)
                {
                    courseDistance = direction;
                    break;
                }

                int pwmL = 10 + (colorRgb.b - lData->getTarget()) * lData->getP();
                int pwmR = 10 + (lData->getTarget() - colorRgb.b) * lData->getP();
                mLeftMotor->setPWM(pwmL);
                mRightMotor->setPWM(pwmR);
            }
            // 3: 反対向きになる
            else if (lData->getMode() == 3)
            {
                mLeftMotor->setPWM(-10);
                mRightMotor->setPWM(10);

                if (calculation->rightDistance(mRightMotor->getCount()) - pointRightDirection > 300.0)
                {
                    courseDistance = direction;
                    break;
                }
            }

            // 10: 停止する
            else if (lData->getMode() == 10)
            {
                mLeftMotor->setPWM(0);
                mRightMotor->setPWM(0);
            }

            tail->tail(lData->getAngle());

            // 距離について
            int32_t motor_ang_l = mLeftMotor->getCount();
            int32_t motor_ang_r = mRightMotor->getCount();
            direction = calculation->directionAll(motor_ang_l, motor_ang_r);
        }
    }
}
//ライントレース、balance_controlを使う場合
void EV3RT::stairDriving(StairDriveData* sData)
{
    //tail_control呼び出し
    tail->tail_control(sData->getTailValue()); /* バランス走行用角度に制御 */

    //forwadをセット
    forward = sData->getForward();

    //階段走行データとして渡されたsData内のtarget値が0の場合
    //EV3RT内で宣言されているtarget値を使用(grayjudgeの時のtarget値が入る)（無理やり
    if(sData->getTarget() == 0)
    {
        turn = calculation->leftDirection
        (
            KP,
            KI,
            KD,
            target,
            colorRgb
        );  //線の左をライントレース
    }
    else //値があるならそれを使う
    {
        turn = calculation->leftDirection
        (
            KP,
            KI,
            KD,
            sData->getTarget(),
            colorRgb
        );  //線の左をライントレース
    }

    balancer_control(forward, turn);

    mClock->sleep(4); /* 4msec周期起動 */
}

//balance_controlを使わず直接タイヤに値を渡す場合
void EV3RT::stairForcing(StairForceData* sData)
{
    //LEDの色をセット
    ev3_led_set_color(sData->getLedColor());

    //左右モータにセット
    mLeftMotor->setPWM(sData->getLeftPwm());
    mRightMotor->setPWM(sData->getRightPwm());

    //tailメソッド呼び出し
    tail->tail(sData->getTailValue());
}

//ロボの停止
void EV3RT::stop()
{
    mLeftMotor->reset();
    mRightMotor->reset();
}

//走行準備
void EV3RT::ready()
{
    //尻尾で機体を立たせ、待機
    tail->standTail(mTouchSensor, mColorSensor, bt_cmd);

    /* 走行モーターエンコーダーリセット */
    mLeftMotor->reset();
    mRightMotor->reset();

    balance_init(); /* 倒立振子API初期化 */

    //mGyroSensor->reset(); /* ジャイロセンサーリセット */

    //tail->tail(angle);
    /* 尻尾の支えがなくなり、自立する */
    /* 要修正 */

    for(double angle = TAIL_ANGLE_STAND_UP; angle <= TAIL_ANGLE_START; angle++)
    {
        //角度がスタート角度(99)に達したらスタート
        if(angle >= TAIL_ANGLE_START)
        {
            tail->tail(TAIL_ANGLE_STAND_UP); // バランス走行用角度に制御
            break;
        }
        else
        {
            mLeftMotor->setPWM(0);
			mRightMotor->setPWM(0);
            mClock->reset();
            while(mClock->now() <= 100)
            {
                tail->tail(angle);
                mClock->sleep(4);
            }
            /*
            if(angle >= TAIL_ANGLE_START - 1)
            {
                //mGyroSensor->reset();
                //mClock->sleep(10);
            }
            */
            if(angle == TAIL_ANGLE_STAND_UP)
            {
                mGyroSensor->reset();
                mClock->sleep(10);
            }
        }
    }

}

//drivingメソッドを実行させなくする
void EV3RT::forceStop()
{
    b_stop = true;
}

//難所までの走行距離を保存
void EV3RT::setCourseDistance()
{
    courseDistance = calculation->distance(mRightMotor->getCount(), mLeftMotor->getCount());
}
//*****************************************************************************
// 関数名 : balancer_control
// 引数 :
// 返り値 : 無し
// 概要 :
//*****************************************************************************
void EV3RT::balancer_control(float forward, float turn)
{
  /* オフセット付き角位置取得  */
  int32_t motor_ang_l = mLeftMotor->getCount();
  int32_t motor_ang_r = mRightMotor->getCount();

  /* オフセット付き角速度取得 */
  int32_t gyro = mGyroSensor->getAnglerVelocity();

  /* バッテリの電圧を取得する */
  int32_t volt = ev3_battery_voltage_mV();

  direction = calculation->directionAll(motor_ang_l, motor_ang_r);

  /* 倒立振子制御APIを呼び出し、倒立走行するための */
  /* 左右モータ出力値を得る */
  balance_control
  (
    (float)forward,
    (float)turn,
    (float)gyro,
    (float)GYRO_OFFSET,
    (float)motor_ang_l,
    (float)motor_ang_r,
    (float)volt,
    (int8_t *)&pwmL,
    (int8_t *)&pwmR
  );


  mLeftMotor->setPWM(pwmL);
  mRightMotor->setPWM(pwmR);
}

int8_t EV3RT::getForward()
{
    return forward;
}

int8_t EV3RT::getTurn()
{
    return turn;
}

int EV3RT::getV()
{
    return v;
}

rgb_raw_t EV3RT::getColorRgb()
{
    return colorRgb;
}

uint8_t EV3RT::getColorAmbient()
{
    return mColorSensor->getAmbient();
}

int8_t EV3RT::getColorBrightness()
{
    return mColorSensor->getBrightness();
}
int16_t EV3RT::getGyroAngle()
{
    return mGyroSensor->getAngle();
}

int16_t EV3RT::getGyroAngleVelocity()
{
    return mGyroSensor->getAnglerVelocity();
}

int16_t EV3RT::getSonarDistance()
{
    return mSonarSensor->getDistance();
}

int32_t EV3RT::getLeftMotorCount()
{
    return mLeftMotor->getCount();
}

int32_t EV3RT::getRightMotorCount()
{
    return mRightMotor->getCount();
}

int EV3RT::getLeftMotorPwm()
{
    return (int)pwmL;
}

int EV3RT::getRightMotorPwm()
{
    return (int)pwmR;
}

int* EV3RT::getColorAverage(string fileName)
{
    return calculation->getColorAverage(fileName);
}
