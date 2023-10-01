/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "electrical_machine.h"
#include "pid.h"
#include "math.h"
#define delta_T      0.005f  //5ms����һ��
int i=0;
float A_XY=0;
float x=0.82,y=0.01,z=12;
int16 imu963ra_gyro_z_last;
long front_left_road=0,front_right_road=0,down_left_road=0,down_right_road=0;
int SetSpeed_front_left,SetSpeed_front_right,SetSpeed_down_left,SetSpeed_down_right;
int key_0,key_1,key_2,key_3,key_4,key_5,key_6;
_pid speed_Front_left,speed_Front_right,speed_down_left,speed_down_right;
float PWM_front_left,PWM_front_right,PWM_down_left,PWM_down_right;
float speed_front_left1,speed_front_right1,speed_down_left1,speed_down_right1;
_pid lfw,rfw,ldw,rdw;
float turn;
void CSI_IRQHandler(void)
{
    CSI_DriverIRQHandler();     // ����SDK�Դ����жϺ��� ���������������������õĻص�����
    __DSB();                    // ����ͬ������
}

void IMU_get_count(void)
	{
				imu963ra_get_gyro();
//				imu963ra_gyro_z=x*imu963ra_gyro_z_last+y*(imu963ra_gyro_z+z);
//				imu963ra_gyro_z_last=imu963ra_gyro_z;
					
	}
int flag=0;
void PIT_IRQHandler(void)
{
    if(pit_flag_get(PIT_CH0))
    {		
				if(flag<40)
				{
					flag++;
					encoder_clear_count(QTIMER1_ENCODER1);
					encoder_clear_count(QTIMER2_ENCODER1);
					encoder_clear_count(QTIMER1_ENCODER2);
					encoder_clear_count(QTIMER2_ENCODER2);	
				}
				
				front_left_counts=encoder_get_count(QTIMER1_ENCODER1);
				front_right_counts=-encoder_get_count(QTIMER2_ENCODER1);
				down_left_counts=encoder_get_count(QTIMER2_ENCODER2);
				down_right_counts=encoder_get_count(QTIMER1_ENCODER2);
				

				front_left_road+=front_left_counts;
				front_right_road+=front_right_counts;
				down_left_road+=down_left_counts;
				down_right_road-=down_right_counts;
			
				encoder_clear_count(QTIMER1_ENCODER1);
				encoder_clear_count(QTIMER2_ENCODER1);
				encoder_clear_count(QTIMER1_ENCODER2);
				encoder_clear_count(QTIMER2_ENCODER2);	
				
				IMU_get_count();
				turn=((imu963ra_gyro_z+5)/16.4)*delta_T;
				if(fabs(turn)<0.04)
				{
					turn=0;
				}
				A_XY=2*turn+A_XY;
				A_XY=A_XY;
        pit_flag_clear(PIT_CH0);
    }
    
    if(pit_flag_get(PIT_CH1))
    {
			//23,0.18,0
				speed_front_left1=PID_Calc(&lfw,4,0,1.4,(float)5000,(float)front_left_road);
				PWM_front_left=PID_Calc(&speed_Front_left,3,0,0.9,(float)speed_front_left1,(float)front_left_counts);

				speed_front_right1=PID_Calc(&rfw,3,0,1.0,(float)5000,(float)front_right_road);
				PWM_front_right=PID_Calc(&speed_Front_right,4,0,0.5,(float)speed_front_right1,(float)front_right_counts);
			
				speed_down_left1=PID_Calc(&ldw,3.1,0,0.99,(float)5000,(float)down_left_road);
				PWM_down_left=PID_Calc(&speed_down_left,3.8,0,0.54,(float)speed_down_left1,(float)down_left_counts);
			
				speed_down_right1=PID_Calc(&rdw,3.3,0,1.45,(float)5000,(float)down_right_road);
				PWM_down_right=PID_Calc(&speed_down_right,3,0,0.9,(float)speed_down_right1,(float)down_right_counts);
			
				motor_control(PWM_down_left,PWM_down_right,PWM_front_right,PWM_front_left);
			
        pit_flag_clear(PIT_CH1);
    }
    
    if(pit_flag_get(PIT_CH2))
    {

        pit_flag_clear(PIT_CH2);
    }
    
    if(pit_flag_get(PIT_CH3))
    {
        pit_flag_clear(PIT_CH3);
    }

    __DSB();
}

void LPUART1_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART1))
    {
        // �����ж�
    #if DEBUG_UART_USE_INTERRUPT                        // ������� debug �����ж�
        debug_interrupr_handler();                      // ���� debug ���ڽ��մ����� ���ݻᱻ debug ���λ�������ȡ
    #endif                                              // ����޸��� DEBUG_UART_INDEX ����δ�����Ҫ�ŵ���Ӧ�Ĵ����ж�ȥ
    }
        
    LPUART_ClearStatusFlags(LPUART1, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART2_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART2))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART2, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART3_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART3))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART3, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART4_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART4))
    {
        // �����ж� 
        flexio_camera_uart_handler();
        
        gps_uart_callback();
    }
        
    LPUART_ClearStatusFlags(LPUART4, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART5_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART5))
    {
        // �����ж�
        camera_uart_handler();
    }
        
    LPUART_ClearStatusFlags(LPUART5, kLPUART_RxOverrunFlag);    // ������ɾ��
}

void LPUART6_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART6))
    {
        // �����ж�
        
    }
        
    LPUART_ClearStatusFlags(LPUART6, kLPUART_RxOverrunFlag);    // ������ɾ��
}


void LPUART8_IRQHandler(void)
{
    if(kLPUART_RxDataRegFullFlag & LPUART_GetStatusFlags(LPUART8))
    {
        // �����ж�
        wireless_module_uart_handler();
        
    }
        
    LPUART_ClearStatusFlags(LPUART8, kLPUART_RxOverrunFlag);    // ������ɾ��
}


void GPIO1_Combined_0_15_IRQHandler(void)
{
    if(exti_flag_get(B0))
    {
        exti_flag_clear(B0);// ����жϱ�־λ
    }
    
}


void GPIO1_Combined_16_31_IRQHandler(void)
{
    wireless_module_spi_handler();
    if(exti_flag_get(B16))
    {
        exti_flag_clear(B16); // ����жϱ�־λ
    }

    
}

void GPIO2_Combined_0_15_IRQHandler(void)
{
    flexio_camera_vsync_handler();
    
    if(exti_flag_get(C0))
    {
        exti_flag_clear(C0);// ����жϱ�־λ
    }

}


void GPIO2_Combined_16_31_IRQHandler(void)
{
    // -----------------* ToF INT �����ж� Ԥ���жϴ����� *-----------------
    tof_module_exti_handler();
    // -----------------* ToF INT �����ж� Ԥ���жϴ����� *-----------------
    
    if(exti_flag_get(C16))
    {
        exti_flag_clear(C16); // ����жϱ�־λ
    }
    
}




void GPIO3_Combined_0_15_IRQHandler(void)
{

    if(exti_flag_get(D4))
    {
        exti_flag_clear(D4);// ����жϱ�־λ
    }
}









/*
�жϺ������ƣ��������ö�Ӧ���ܵ��жϺ���
Sample usage:��ǰ���������ڶ�ʱ���ж�
void PIT_IRQHandler(void)
{
    //��������־λ
    __DSB();
}
�ǵý����жϺ������־λ
CTI0_ERROR_IRQHandler
CTI1_ERROR_IRQHandler
CORE_IRQHandler
FLEXRAM_IRQHandler
KPP_IRQHandler
TSC_DIG_IRQHandler
GPR_IRQ_IRQHandler
LCDIF_IRQHandler
CSI_IRQHandler
PXP_IRQHandler
WDOG2_IRQHandler
SNVS_HP_WRAPPER_IRQHandler
SNVS_HP_WRAPPER_TZ_IRQHandler
SNVS_LP_WRAPPER_IRQHandler
CSU_IRQHandler
DCP_IRQHandler
DCP_VMI_IRQHandler
Reserved68_IRQHandler
TRNG_IRQHandler
SJC_IRQHandler
BEE_IRQHandler
PMU_EVENT_IRQHandler
Reserved78_IRQHandler
TEMP_LOW_HIGH_IRQHandler
TEMP_PANIC_IRQHandler
USB_PHY1_IRQHandler
USB_PHY2_IRQHandler
ADC1_IRQHandler
ADC2_IRQHandler
DCDC_IRQHandler
Reserved86_IRQHandler
Reserved87_IRQHandler
GPIO1_INT0_IRQHandler
GPIO1_INT1_IRQHandler
GPIO1_INT2_IRQHandler
GPIO1_INT3_IRQHandler
GPIO1_INT4_IRQHandler
GPIO1_INT5_IRQHandler
GPIO1_INT6_IRQHandler
GPIO1_INT7_IRQHandler
GPIO1_Combined_0_15_IRQHandler
GPIO1_Combined_16_31_IRQHandler
GPIO2_Combined_0_15_IRQHandler
GPIO2_Combined_16_31_IRQHandler
GPIO3_Combined_0_15_IRQHandler
GPIO3_Combined_16_31_IRQHandler
GPIO4_Combined_0_15_IRQHandler
GPIO4_Combined_16_31_IRQHandler
GPIO5_Combined_0_15_IRQHandler
GPIO5_Combined_16_31_IRQHandler
WDOG1_IRQHandler
RTWDOG_IRQHandler
EWM_IRQHandler
CCM_1_IRQHandler
CCM_2_IRQHandler
GPC_IRQHandler
SRC_IRQHandler
Reserved115_IRQHandler
GPT1_IRQHandler
GPT2_IRQHandler
PWM1_0_IRQHandler
PWM1_1_IRQHandler
PWM1_2_IRQHandler
PWM1_3_IRQHandler
PWM1_FAULT_IRQHandler
SEMC_IRQHandler
USB_OTG2_IRQHandler
USB_OTG1_IRQHandler
XBAR1_IRQ_0_1_IRQHandler
XBAR1_IRQ_2_3_IRQHandler
ADC_ETC_IRQ0_IRQHandler
ADC_ETC_IRQ1_IRQHandler
ADC_ETC_IRQ2_IRQHandler
ADC_ETC_ERROR_IRQ_IRQHandler
PIT_IRQHandler
ACMP1_IRQHandler
ACMP2_IRQHandler
ACMP3_IRQHandler
ACMP4_IRQHandler
Reserved143_IRQHandler
Reserved144_IRQHandler
ENC1_IRQHandler
ENC2_IRQHandler
ENC3_IRQHandler
ENC4_IRQHandler
TMR1_IRQHandler
TMR2_IRQHandler
TMR3_IRQHandler
TMR4_IRQHandler
PWM2_0_IRQHandler
PWM2_1_IRQHandler
PWM2_2_IRQHandler
PWM2_3_IRQHandler
PWM2_FAULT_IRQHandler
PWM3_0_IRQHandler
PWM3_1_IRQHandler
PWM3_2_IRQHandler
PWM3_3_IRQHandler
PWM3_FAULT_IRQHandler
PWM4_0_IRQHandler
PWM4_1_IRQHandler
PWM4_2_IRQHandler
PWM4_3_IRQHandler
PWM4_FAULT_IRQHandler
Reserved171_IRQHandler
GPIO6_7_8_9_IRQHandler*/



