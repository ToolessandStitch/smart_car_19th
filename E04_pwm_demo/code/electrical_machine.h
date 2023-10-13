#ifndef _electrical_machine_h_
#define _electrical_machine_h_

/******头文件的声明******/
#include "zf_common_typedef.h"

/******全局变量的声明******/
extern long front_left_counts,front_right_counts,down_left_counts,down_right_counts;

/******函数的声明******/
 void overall_init();
 void pwm_Init();//电机PWM初始化
 void DIR_IO_Init(void);//电机旋转方向初始化
 void encoder_init(void);//编码器初始化
 void encoder_get();//编码器的初始化
 void sevor_init(void);//舵机初始化
 void sevor_control(int C30,int D0,int D1);//舵机控制函数
 void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2);//前进
	 




#endif