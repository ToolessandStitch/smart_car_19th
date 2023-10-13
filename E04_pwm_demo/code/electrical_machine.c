/********************************************************************************************************************
* ��������ʼ���Լ����������㷨
* ���ļ������㷨��
* �ļ�����    electrical_machine.c
* ����ʱ�䣺2023-10-13
* ���ߣ���������������
* ��ע������
********************************************************************************************************************/

/******ͷ�ļ�������******/
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "electrical_machine.h"
#include "efficient.h"
#include "pid.h"
#include "isr.h"
/******ȫ�ֱ����Ķ���******/
long front_left_counts=0,front_right_counts=0,down_left_counts=0,down_right_counts=0;

/******���庯��******/
void overall_init()
{
		key_init(KEY_1);//�����ĳ�ʼ��
		key_init(KEY_2);//�����ĳ�ʼ��
		key_init(KEY_3);//�����ĳ�ʼ��
		key_init(KEY_4);//�����ĳ�ʼ��
		ips200_init(IPS200_TYPE_SPI);//ips200��ʾ���ĳ�ʼ��
		pwm_Init();//���PWM��ʼ��
		DIR_IO_Init();//�����תio�����ʼ��
		encoder_init();//������������ʼ��
	  sevor_init();//�����ʼ��
		mt9v03x_init();//����ͷ�ĳ�ʼ��
		imu963ra_init();//��̬������imu963ra�ĳ�ʼ��
		wireless_uart_init();//���ߴ������ĳ�ʼ��
	  /*��pid�ĳ�ʼ��*/
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
void pwm_Init()//���PWM��ʼ��
{
		//����pwm_init�����Զ�ǰ������drv8701���������Ӧ��pwm����ڽ��г�ʼ��
    pwm_init(PWM2_MODULE0_CHB_C7, 20000, 0);
    pwm_init(PWM2_MODULE1_CHB_C9 , 20000, 0);
    pwm_init(PWM2_MODULE2_CHB_C11, 20000, 0);
    pwm_init(PWM2_MODULE3_CHB_D3 , 20000, 0);
}
void DIR_IO_Init(void)//�����תio�����ʼ��
{
	//����gpio_init������ǰ������drv8701���������Ӧ�ķ���io���г�ʼ��
	gpio_init(C10 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(D2 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C6 , GPO, 0, GPO_PUSH_PULL);
	gpio_init(C8 , GPO, 0, GPO_PUSH_PULL);

}
void encoder_init(void)//������������ʼ��
{
		//������������ʼ��
	  encoder_quad_init(QTIMER1_ENCODER1,QTIMER1_ENCODER1_CH1_C0,QTIMER1_ENCODER1_CH2_C1);
    encoder_quad_init(QTIMER2_ENCODER1,QTIMER2_ENCODER1_CH1_C3,QTIMER2_ENCODER1_CH2_C4);
    encoder_quad_init(QTIMER2_ENCODER2,QTIMER2_ENCODER2_CH1_C5,QTIMER2_ENCODER2_CH2_C25);
    encoder_quad_init(QTIMER1_ENCODER2,QTIMER1_ENCODER2_CH1_C2,QTIMER1_ENCODER2_CH2_C24);
		//�����������Ӧ�ļĴ���
		encoder_clear_count(QTIMER1_ENCODER1);
		encoder_clear_count(QTIMER2_ENCODER1);
		encoder_clear_count(QTIMER1_ENCODER2);
		encoder_clear_count(QTIMER2_ENCODER2);
}
void sevor_init(void)//�����ʼ��
{
		//����pwm_init������io�ڽ���PWM���͵ĳ�ʼ��
		pwm_init(PWM4_MODULE2_CHA_C30, 50, 0);
		pwm_init(PWM1_MODULE3_CHA_D0 , 50, 0);
		pwm_init(PWM1_MODULE3_CHB_D1 , 50, 0);
}
void sevor_control(int C30,int D0,int D1)//������ƽ��麯��
{
		//������ƺ�������pwm_set_duty�������Զ���������ӿ��������pwm���ĸߵ͵�ƽ��ռ��
		pwm_set_duty(PWM4_MODULE2_CHA_C30,C30);
		pwm_set_duty(PWM1_MODULE3_CHA_D0, D0) ;
		pwm_set_duty(PWM1_MODULE3_CHB_D1, D1) ;
}
/*ֻ���ж��е����������*/
void encoder_get()//���������ݻ�ȡ����
{
	//�Ƚ���ȡ�ĸ��ֵ�ת�ٵı�����0
	front_left_counts=0;
	front_right_counts=0;
	down_left_counts=0;
	down_right_counts=0;
	//����encoder_get_count��������ȡ���ĸ���������ֵ��ŵ��ĸ�������
	front_left_counts=encoder_get_count(QTIMER1_ENCODER1);
	front_right_counts=encoder_get_count(QTIMER2_ENCODER1);
	down_left_counts=encoder_get_count(QTIMER2_ENCODER2);
	down_right_counts=encoder_get_count(QTIMER1_ENCODER2);
	//����encoder_clear_count��������ĸ���������Ӧ�ļĴ���
	encoder_clear_count(QTIMER1_ENCODER1);
	encoder_clear_count(QTIMER2_ENCODER1);
	encoder_clear_count(QTIMER1_ENCODER2);
	encoder_clear_count(QTIMER2_ENCODER2);
}
void motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//������ƺ���
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

//��������������
//��������������
//��������������
//��������������
//void ZUOPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//��ƽ�Ƶ�����ƺ���
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
//void YOUPINGYI_motor_control(int duty_l1, int duty_r1,int duty_l2, int duty_r2)//��ƽ�Ƶ�����ƺ���
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
//��������������
//��������������
//��������������
//��������������


