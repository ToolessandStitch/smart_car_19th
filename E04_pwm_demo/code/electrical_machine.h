#ifndef _electrical_machine_h_
#define _electrical_machine_h_

/******ͷ�ļ�������******/
#include "zf_common_typedef.h"

/******ȫ�ֱ���������******/
extern long front_left_counts,front_right_counts,down_left_counts,down_right_counts;

/******����������******/
 void overall_init();
 void pwm_Init();//���PWM��ʼ��
 void DIR_IO_Init(void);//�����ת�����ʼ��
 void encoder_init(void);//��������ʼ��
 void encoder_get();//�������ĳ�ʼ��
 void sevor_init(void);//�����ʼ��
 void sevor_control(int C30,int D0,int D1);//������ƺ���
 void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2);//ǰ��
	 




#endif