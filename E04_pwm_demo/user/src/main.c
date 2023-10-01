

#include "zf_common_headfile.h"
#include "electrical_machine.h"
#include "mt9v03x.h"
#include "efficient.h"
#include "isr.h"
#include "pid.h"
#include "ps2.h"
#define IPS200_TYPE     (IPS200_TYPE_SPI)                                 // 双排排针 并口两寸屏 这里宏定义填写 IPS200_TYPE_PARALLEL8
                                                                          // 单排排针 SPI 两寸屏 这里宏定义填写 IPS200_TYPE_SPI
// **************************** 代码区域 ****************************
int main (void)
{
    clock_init(SYSTEM_CLOCK_600M);                                              // 初始化芯片时钟 工作频率为 600MHz
    debug_init();                                                               // 初始化默认 debug uart
    // 此处编写用户代码 例如外设初始化代码等
		key_init(KEY_1);
		key_init(KEY_2);
		key_init(KEY_3);
		key_init(KEY_4);
		uint8 PS2_KEY = 1, X1=0,Y1=0,X2=0,Y2=0; 
		imu963ra_init();
		ips200_init(IPS200_TYPE);
	
    pwm_Init();
		DIR_IO_Init();
		encoder_init();
		sevor_init();
		PID_Init();	
		
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
		
		system_delay_ms(300);           //等待主板其他外设上电完成

		pit_ms_init(PIT_CH0, 10);
		pit_ms_init(PIT_CH1, 10);
		pit_ms_init(PIT_CH2, 10);
		wireless_uart_init();
		PS2_Init();

    while(1)
    {
				PS2_KEY = PS2_DataKey();	 //手柄按键捕获处理
				//获取模拟值
			if(PS2_KEY == PSB_L1 || PS2_KEY == PSB_R1)
			{
				X1 = PS2_AnologData(PSS_LX);
				Y1 = PS2_AnologData(PSS_LY);
				X2 = PS2_AnologData(PSS_RX);
				Y2 = PS2_AnologData(PSS_RY);
			}
			printf("%d\r\n",X1);
				sevor_control();
				ips200_show_int(10,10,1,1);
				SetSpeed_front_left=400;
//				key_scanner();
//				key_1=key_get_state(KEY_1);
//				key_2=key_get_state(KEY_2);
//				key_3=key_get_state(KEY_3);
//				key_4=key_get_state(KEY_4);
//				key_5=key_get_state(KEY_5);
//				key_6=key_get_state(KEY_6);
			my_camera(1);
			TwoThreshold();
			
    }
}
// **************************** 代码区域 ****************************
