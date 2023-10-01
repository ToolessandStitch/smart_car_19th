#ifndef _electrical_machine_h_
#define _electrical_machine_h_

#include "zf_common_typedef.h"

extern long front_left_counts,front_right_counts,down_left_counts,down_right_counts;

extern void pwm_Init();//���PWM��ʼ��
extern void DIR_IO_Init(void);//�����ת�����ʼ��
extern void encoder_init(void);//��������ʼ��
extern void encoder_get();
extern void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2);//ǰ��
extern void sevor_init(void);//�����ʼ��
extern void sevor_control();




#endif