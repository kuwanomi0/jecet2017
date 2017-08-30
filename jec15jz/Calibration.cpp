#include "Calibration.h"

Calibration::Calibration(int32_t *bt_cmd)
{
  colorSensor = new ColorSensor(PORT_3);
  clock       = new Clock();
  screanPosi = 1;
  target = 0;
  this->bt_cmd = bt_cmd;
  for(int i = 0; i < 3; i++)
  {
      rgbColors[i] = 0;
  }
  colorSensor->getRawColor(colorRgb);
}

Calibration::~Calibration()
{

}
//*****************************************************************************
// 関数名 : setColor
// 引数   :無し
// 返り値 : 無し
// 概要  : 押されたボタンによってどの色をとったか教える
//*****************************************************************************
void Calibration::setColor()
{
  while(1) //消してやってみる？
  {
    if(ev3_button_is_pressed(LEFT_BUTTON) || *bt_cmd == 3 )
    {
      colorName = "white";
      wbg.white = getRgbColor();
      target += wbg.white;
      bt_cmdReset();
      break;
    }
    else if(ev3_button_is_pressed(UP_BUTTON) || *bt_cmd == 4)
    {
      colorName = "black";
      wbg.black = getRgbColor();
      target += wbg.black;
      bt_cmdReset();
      break;
    }
    else if(ev3_button_is_pressed(RIGHT_BUTTON) || *bt_cmd == 5)
    {
      colorName = "gray";
      wbg.gray = getRgbColor();
      gray = wbg.gray;
      bt_cmdReset();
      break;
    }
    clock->sleep(10);
  }
}
//*****************************************************************************
// 関数名  : setRgbColor()
// 引数   : 無し
// 返り値 : 色の合計値
// 概要  :RGBで取った各値をEV３上画面に出力し、合計値/100の値を返す
//*****************************************************************************
int Calibration::getRgbColor()
{
  //static char buff[1024];
 stringstream val;
 const char* pchar;
 int sum = 0;
 ostringstream formatter;

 for(int i = 0; i < 100; i++)
 {
     colorSensor->getRawColor(colorRgb);
     rgbColors[0] += colorRgb.r;
     rgbColors[1] += colorRgb.g;
     rgbColors[2] += colorRgb.b;
     clock->sleep(4);
 }

 //取った値の合計値と各100回取った値を100で割る
 for(int i = 0; i < 3; i++)
 {
     sum += rgbColors[i];
     rgbColors[i] /= 100;
 }

 //ログ用に構造体に値を保存
 if(colorName == "white")
 {
     wbg.whiteR = rgbColors[0];
     wbg.whiteG = rgbColors[1];
     wbg.whiteB = rgbColors[2];
 }
 else if(colorName == "black")
 {
     wbg.blackR = rgbColors[0];
     wbg.blackG = rgbColors[1];
     wbg.blackB = rgbColors[2];
 }
 else if(colorName == "gray")
 {
     wbg.grayR = rgbColors[0];
     wbg.grayG = rgbColors[1];
     wbg.grayB = rgbColors[2];
 }

 sum /= 100;

    //格変数を文字列型に変換
 val << colorName << "R : " << rgbColors[0] << " "
     << colorName << "G : " << rgbColors[1];
     //char*型に変換
 pchar = val.str().c_str();
     //ev3上の画面に出力
 ev3_lcd_draw_string(pchar, 0, 10 * getScreanPosi());
 screanPosi += 1;
 val.str(""); // バッファをクリアする。
 // ストリームの状態をクリアする。この行がないと意図通りに動作しない
 val.clear(stringstream::goodbit);

//格変数を文字列型に変換
 val << colorName << "B : " << rgbColors[2]
     << " " << "sum : " << sum;
  //char*型に変換
 pchar = val.str().c_str();
  //ev3上の画面に出力
 ev3_lcd_draw_string(pchar, 0, 10 * getScreanPosi());
 //画面上の出力高さを変える
 screanPosi += 1;

 return sum;
}

//*****************************************************************************
// 関数名  : runningGetColor()
// 引数   :無し
// 返り値 : 現在の位置をカラーセンサーのRGBで取った値を保存した構造体
// 概要  :走行中のRGBのカラーセンサーから取る値を保存
//*****************************************************************************
run_color Calibration::getRunningColor()
{
    run_color runColor;
    //平均を取るため１００回取得
    for(int i = 0; i < 100; i++)
    {
        colorSensor->getRawColor(colorRgb);
        rgbColors[0] += colorRgb.r;
        rgbColors[1] += colorRgb.g;
        rgbColors[2] += colorRgb.b;
        //clock->sleep(4);
    }
    //取った値の合計値と各100回取った値を100で割る
    for(int i = 0; i < 3; i++)
    {
        rgbColors[i] /= 100;
    }
    runColor.runR = rgbColors[0];
    runColor.runG = rgbColors[1];
    runColor.runB = rgbColors[2];

    return runColor;
}
//*****************************************************************************
// 関数名 : getTarget
// 引数   : 無し
// 返り値 : ターゲット値
// 概要  :ターゲット値を返す
//*****************************************************************************
int Calibration::getTarget()
{
  return target;
}

//*****************************************************************************
// 関数名 : getGray
// 引数   : 無し
// 返り値 : Grayの値
// 概要   : Gray値を返す
//*****************************************************************************
int Calibration::getGray()
{
  return gray;
}

//*****************************************************************************
// 関数名 : getColorSensor
// 引数   : 無し
// 返り値 : 生成されたColorSensorのオブジェクトを返す
// 概要   : ColorSensorのオブジェクトを返す
//*****************************************************************************
ColorSensor* Calibration::getColorSensor()
{
  return colorSensor;
}

//*****************************************************************************
// 関数名 : getScreanPosi
// 引数   : 無し
// 返り値 : Grayの値
// 概要   : Gray値を返す
//*****************************************************************************
int Calibration::getScreanPosi()
{
  return screanPosi;
}

//*****************************************************************************
// 関数名 : getWbgColor
// 引数   : 無し
// 返り値 : WBGの構造体の値
// 概要   : whiteR,blackG, grayBなどの値を持った構造体を返す
//*****************************************************************************
void Calibration::getWbgColor(WBG* wbg)
{
    wbg->whiteR = this->wbg.whiteR;
    wbg->whiteG = this->wbg.whiteG;
    wbg->whiteB = this->wbg.whiteB;
    wbg->white  = this->wbg.white;

    wbg->blackR = this->wbg.blackR;
    wbg->blackG = this->wbg.blackG;
    wbg->blackB = this->wbg.blackB;
    wbg->black  = this->wbg.black;

    wbg->grayR = this->wbg.grayR;
    wbg->grayG = this->wbg.grayG;
    wbg->grayB = this->wbg.grayB;
    wbg->gray  = this->wbg.gray;
}
//*****************************************************************************
// 関数名 : bt_cmdReset()
// 引数   : 無し
// 返り値 : 無し
// 概要   : ブルートゥース接続でTeratermから送られてくる値をリセットする
//*****************************************************************************
void Calibration::bt_cmdReset()
{
    *bt_cmd = -1;
}
