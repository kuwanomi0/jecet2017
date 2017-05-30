#ifndef PID_H
#define PID_H


class PID {

private:
    float m_P_coefficient; /*比例定数*/
    float m_I_coefficient; /*積分定数*/
    float m_D_coefficient; /*微分定数*/

    int m_d_now;
    int m_d_pre;
    int m_i_list[13];
    int m_num;

public:
    PID(float p_value,float i_value ,float d_value);
    float calcControllValue(int now_value);
};
#endif
