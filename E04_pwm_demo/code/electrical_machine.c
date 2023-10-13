/********************************************************************************************************************
* 电机舵机初始化以及基本控制算法
* 该文件包含算法：
* 文件名称    electrical_machine.c
* 创建时间：2023-10-13
* 作者：长了牙的无牙仔
* 备注：暂无
********************************************************************************************************************/

/******头文件的声明******/
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "electrical_machine.h"
#include "efficient.h"
#include "pid.h"
#include "isr.h"
/******全局变量的定义******/
long front_left_counts=0,front_right_counts=0,down_left_counts=0,down_right_counts=0;

/******具体函数******/
void overall_init()
{
		key_init(KEY_1);//按键的初始化
		key_init(KEY_2);//按键的初始化
		key_init(KEY_3);//按键的初始化
		key_init(KEY_4);//按键的初始化
		ips200_init(IPS200_TYPE_SPI);//ips200显示屏的初始化
		pwm_Init();//电机PWM初始化
		DIR_IO_Init();//电机旋转io方向初始化
		encoder_init();//正交编码器初始化
	  sevor_init();//舵机初始化
		mt9v03x_init();//摄像头的初始化
		imu963ra_init();//姿态传感器imu963ra的初始化
		wireless_uart_init();//无线传感器的初始化
	  /*对pid的初始化*/
		PID_Data_Init(&lfw);
		PID_Data_Init(&rfw);
		PID_Data_Init(&ldw);
		PID_Data_Init(&rdw);
		PID_Data_Init(&speed_Front_left);
		PID_Data_Init(&speed_Front_right);
		PID_Data_Init(&speed_down_left);
		PID_Data_Init(&speed_down_right);
		PID_Param_Init(&lfw,20000,1000);
		PID_Param_Init(&rfw,20000,1000);
		PID_Param_Init(&ldw,20000,1000);
		PID_Param_Init(&rdw,20000,1000);
		PID_Param_Init(&speed_Front_left,20000,20000);
		PID_Param_Init(&speed_Front_right,20000,20000);
		PID_Param_Init(&speed_down_left,20000,20000);
		PID_Param_Init(&speed_down_right,20000,20000);
}
void pwm_Init()//电机PWM初始化
{
		//调用pwm_init函数对对前后两组drv8701电机驱动对应的pwm输出口进行初始化
    pwm_init(PWM2_MODULE0_CHB_C7, 20000, 0);
    pwm_init(PWM2_MODULE1_CHB_C9 , 20000, 0);
    pwm_init(PWM2_MODULE2_CHB_C11, 20000, 0);
    pwm_init(PWM2_MODULE3_CHB_D3 , 20000, 0);
}
void DIR_IO_Init(void)//电机旋转io方向初始化
{
	//调用gpio_init函数对前后两组drv8701电机驱动对应的方向io进行初始化
	gpio_init(C10 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(D2 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C6 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C8 , GPO, 0, GPO_PUSH_PULL);

}
void encoder_init(void)//正交编码器初始化
{
		//正交编码器初始化
	  encoder_quad_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);
    encoder_quad_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER2_ENCODER2,QTIMER2_ENCODER2_CH1_C5,QTIMER2_ENCODER2_CH2_C25);
    encoder_quad_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);
		//清除编码器对应的寄存器
		encoder_clear_count(QTIMER1_ENCODER1);
		encoder_clear_count(QTIMER2_ENCODER1);
		encoder_clear_count(QTIMER1_ENCODER2);
		encoder_clear_count(QTIMER2_ENCODER2);
}
void sevor_init(void)//舵机初始化
{
		//调用pwm_init函数对io口进行PWM类型的初始化
		pwm_init(PWM4_MODULE2_CHA_C30, 50, 0);
		pwm_init(PWM1_MODULE3_CHA_D0 , 50, 0);
		pwm_init(PWM1_MODULE3_CHB_D1 , 50, 0);
}
void sevor_control(int C30,int D0,int D1)//舵机控制建议函数
{
		//舵机控制函数调用pwm_set_duty函数，对舵机的三个接口设置输出pwm波的高低电平的占比
		pwm_set_duty(PWM4_MODULE2_CHA_C30,C30);
		pwm_set_duty(PWM1_MODULE3_CHA_D0, D0) ;
		pwm_set_duty(PWM1_MODULE3_CHB_D1, D1) ;
}
/*只在中断中调用这个函数*/
void encoder_get()//编码器数据获取函数
{
	//先将存取四个轮的转速的变量清0
	front_left_counts=0;
	front_right_counts=0;
	down_left_counts=0;
	down_right_counts=0;
	//调用encoder_get_count函数将获取到四个编码器的值存放到四个变量里
	front_left_counts=encoder_get_count(QTIMER1_ENCODER1);
	front_right_counts=encoder_get_count(QTIMER2_ENCODER1);
	down_left_counts=encoder_get_count(QTIMER2_ENCODER2);
	down_right_counts=encoder_get_count(QTIMER1_ENCODER2);
	//调用encoder_clear_count函数清空四个编码器对应的寄存器
	encoder_clear_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER2_ENCODER2);
}
void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//电机控制函数
{
                           
       	if(duty_l1>0)
					{
						gpio_set_level(D2, 0);
						pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);
					}
				else
					{
						gpio_set_level(D2, 1);
						pwm_set_duty(PWM2_MODULE3_CHB_D3, -duty_l1);
					}
				if(duty_l2>0)
					{
						gpio_set_level(C6, 0);
						pwm_set_duty(PWM2_MODULE0_CHB_C7, duty_l2);
					}
				else
					{
						gpio_set_level(C6, 1);
						pwm_set_duty(PWM2_MODULE0_CHB_C7, -duty_l2);
					}
				if(duty_r2>0)
						{
							gpio_set_level(C8, 0);
							pwm_set_duty(PWM2_MODULE1_CHB_C9, duty_r2);
						}
				else
						{
							gpio_set_level(C8, 1);
							pwm_set_duty(PWM2_MODULE1_CHB_C9, -duty_r2);
						}
	      if(duty_r1>0)
					{
						gpio_set_level(C10, 0);
						pwm_set_duty(PWM2_MODULE2_CHB_C11, duty_r1);
	        }
			 if (duty_r1<0)
					{
						gpio_set_level(C10, 1);
						pwm_set_duty(PWM2_MODULE2_CHB_C11, -duty_r1);
	        }
   
}

//勿碰！！！！！
//勿碰！！！！！
//勿碰！！！！！
//勿碰！！！！！
//void ZUOPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//左平移电机控制函数
//{
//				gpio_set_level(D13, 1);
//        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
//				gpio_set_level(D12, 0);
//        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
//				gpio_set_level(D1, 0);
//        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);
//        gpio_set_level(D0, 1);
//        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);



//}
//void YOUPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//右平移电机控制函数
//{
//				gpio_set_level(D13, 0);
//        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
//				gpio_set_level(D12, 1);
//        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
//				gpio_set_level(D1, 1);
//        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);
//        gpio_set_level(D0, 0);
//        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

//}
//void YOUXIEQIAN_motor_control(int duty_l1,int duty_r2)
//{
//				
//        gpio_set_level(D12, 1);
//        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
//				gpio_set_level(D1, 1);
//        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);

//}
//void ZUOXIEQIAN_motor_control(int duty_l2,int duty_r1)
//{
//				gpio_set_level(D13, 1);
//        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
//				gpio_set_level(D0, 1);
//        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

//}
//void YOUXIEHOU_motor_control(int duty_r1,int duty_l2)
//{
//				gpio_set_level(D13, 0);
//        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
//				gpio_set_level(D0, 0);
//        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

//}
//void ZUOXIEHOU_motor_control(int duty_l1,int duty_r2)
//{
//				gpio_set_level(D12, 0);
//        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
//				gpio_set_level(D1, 0);
//        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);

//}
//勿碰！！！！！
//勿碰！！！！！
//勿碰！！！！！
//勿碰！！！！！


