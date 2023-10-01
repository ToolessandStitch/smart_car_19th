#ifndef _electrical_machine_h_
#define _electrical_machine_h_

#include "zf_common_typedef.h"

extern long front_left_counts,front_right_counts,down_left_counts,down_right_counts;

extern void pwm_Init();//电机PWM初始化
extern void DIR_IO_Init(void);//电机旋转方向初始化
extern void encoder_init(void);//编码器初始化
extern void encoder_get();
extern void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2);//前进
extern void sevor_init(void);//舵机初始化
extern void sevor_control();




#endif