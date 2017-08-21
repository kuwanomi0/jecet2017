## 尻尾を下ろしてライントレース

```cpp

mColorSensor  = new ColorSensor(PORT_3);
mColorSensor->getRawColor(colorRgb);
pwmL = 7 + (sData->getTarget() - colorRgb.r) * sData->getP();
pwmR = 7 + (colorRgb.r - sData->getTarget()) * sData->getP();
mLeftMotor->setPWM(pwmL);
mRightMotor->setPWM(pwmR);

```
