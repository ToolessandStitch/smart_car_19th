#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "electrical_machine.h"
#include "efficient.h"
long front_left_counts=0,front_right_counts=0,down_left_counts=0,down_right_counts=0;

void pwm_Init()//电机PWM初始化
{
    pwm_init(PWM2_MODULE0_CHB_C7, 20000, 0);
    pwm_init(PWM2_MODULE1_CHB_C9 , 20000, 0);
    pwm_init(PWM2_MODULE2_CHB_C11, 20000, 0);
    pwm_init(PWM2_MODULE3_CHB_D3 , 20000, 0);
}
void DIR_IO_Init(void)//电机旋转方向初始化
{
	gpio_init(C10 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(D2 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C6 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C8 , GPO, 0, GPO_PUSH_PULL);

}
void encoder_init(void)//编码器初始化
{
	  encoder_quad_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);
    encoder_quad_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER2_ENCODER2,QTIMER2_ENCODER2_CH1_C5,QTIMER2_ENCODER2_CH2_C25);
    encoder_quad_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);
	
		encoder_clear_count(QTIMER1_ENCODER1);
		encoder_clear_count(QTIMER2_ENCODER1);
		encoder_clear_count(QTIMER1_ENCODER2);
		encoder_clear_count(QTIMER2_ENCODER2);
}

void sevor_init(void)//舵机初始化
{
		pwm_init(PWM4_MODULE2_CHA_C30, 50, 0);
		pwm_init(PWM1_MODULE3_CHA_D0 , 50, 0);
		pwm_init(PWM1_MODULE3_CHB_D1 , 50, 0);
}


void sevor_control()
{
		pwm_set_duty(PWM4_MODULE2_CHA_C30,90);
		pwm_set_duty(PWM1_MODULE3_CHA_D0,90);
		pwm_set_duty(PWM1_MODULE3_CHB_D1,50);
}
	

void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//??
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


void ZUOPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)
{
				gpio_set_level(D13, 1);
        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
				gpio_set_level(D12, 0);
        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
				gpio_set_level(D1, 0);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);
        gpio_set_level(D0, 1);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);



}
void YOUPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)
{
				gpio_set_level(D13, 0);
        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
				gpio_set_level(D12, 1);
        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
				gpio_set_level(D1, 1);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);
        gpio_set_level(D0, 0);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

}
void YOUXIEQIAN_motor_control(int duty_l1,int duty_r2)
{
				
        gpio_set_level(D12, 1);
        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
				gpio_set_level(D1, 1);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);

}
void ZUOXIEQIAN_motor_control(int duty_l2,int duty_r1)
{
				gpio_set_level(D13, 1);
        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
				gpio_set_level(D0, 1);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

}
void YOUXIEHOU_motor_control(int duty_r1,int duty_l2)
{
				gpio_set_level(D13, 0);
        pwm_set_duty(PWM1_MODULE1_CHB_D15, duty_l2);
				gpio_set_level(D0, 0);
        pwm_set_duty(PWM2_MODULE3_CHA_D2, duty_r1);

}
void ZUOXIEHOU_motor_control(int duty_l1,int duty_r2)
{
				gpio_set_level(D12, 0);
        pwm_set_duty(PWM1_MODULE1_CHA_D14, duty_r2);
				gpio_set_level(D1, 0);
        pwm_set_duty(PWM2_MODULE3_CHB_D3, duty_l1);

}


void encoder_get()
{
	front_left_counts=0;
	front_right_counts=0;
	down_left_counts=0;
	down_right_counts=0;
	
	front_left_counts=encoder_get_count(QTIMER1_ENCODER1);
	front_right_counts=encoder_get_count(QTIMER2_ENCODER1);
	down_left_counts=encoder_get_count(QTIMER2_ENCODER2);
	down_right_counts=encoder_get_count(QTIMER1_ENCODER2);
	
	
	encoder_clear_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER2_ENCODER2);
}

