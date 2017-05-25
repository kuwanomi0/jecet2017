#include "PID.h"

/* PIDクラスのコンストラクタ */
PID::PID(float p_value,float i_value ,float d_value) {

	m_P_coefficient = p_value; /*比例定数*/
	m_I_coefficient = i_value; /*積分定数*/
	m_D_coefficient = d_value; /*微分定数*/
	m_d_now = 0;
	m_d_pre = 0;
	m_num = 0;
	for(int i=0; i<13; i++) {
		m_i_list[i]=0;
	}

}

/* PID制御計算メソッド */
float PID::calcControllValue(int now_value) {
	float p_control = 0;
	float i_control = 0;
	float d_control = 0;
	float pid_total = 0;
	int total_i = 0;

	//P制御計算
	p_control = now_value * m_P_coefficient;

	// return p_control;

	//D制御計算
	m_d_pre = m_d_now;
	m_d_now = now_value;
	d_control = (m_d_pre - m_d_now) * m_D_coefficient;

	//I制御計算
	if(m_num>=13) {
		m_num = 0;
	}
	m_i_list[m_num] = now_value;
	for(int i=0; i<13; i++) {
		total_i += m_i_list[i];
	}
	m_num ++;
	i_control = total_i * m_I_coefficient;

	//PID制御計算
	// pid_total = static_cast<int>(p_control + i_control + d_control);
	pid_total = (p_control + i_control + d_control);

	return pid_total;
}
