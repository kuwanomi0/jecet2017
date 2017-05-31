#include "PID.h"

#define DELTA_T 0.004 /* 周期 4msec*/
#define LIMIT   100.0 /* 最大最小の制限 */

/* PID制御計算メソッド */
int PID::calcControl(int now_value) {
    float p_control, i_control, d_control, total;

    diff[0]   = diff[1];
    diff[1]   = now_value;
    integral += (diff[1] + diff[0]) / 2.0 * DELTA_T;

    p_control = kp * diff[1];
    i_control = ki * integral;
    d_control = kd * (diff[1] - diff[0]) / DELTA_T;

    total = p_control + i_control + d_control;

    if (total > LIMIT) {
        total = LIMIT;
    }
    else if (total < -LIMIT) {
        total = -LIMIT;
    }

    return static_cast<int>(p_control + i_control + d_control);
}
