#ifndef _ps2_h_
#define _ps2_h_

#include "zf_common_typedef.h"

#define DI   gpio_get_level(D12)           //PB12  ����

#define DO_H gpio_set_level(D13,GPIO_HIGH)       //����λ��
#define DO_L gpio_set_level(D13,GPIO_LOW)       //����λ��

#define CS_H gpio_set_level(D14,GPIO_HIGH)       //CS����
#define CS_L gpio_set_level(D14,GPIO_LOW)      //CS����

#define CLC_H gpio_set_level(D15,GPIO_HIGH)      //ʱ������
#define CLC_L gpio_set_level(D15,GPIO_LOW)      //ʱ������



//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16



//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8



extern uint8 Data[9];
extern uint16 MASK[16];
extern uint16 Handkey;

extern void PS2_Init(void);
extern uint8 PS2_DataKey(void);		  //��ֵ��ȡ
extern uint8 PS2_AnologData(uint8 button); //�õ�һ��ҡ�˵�ģ����
extern void PS2_ClearData(void);	  //������ݻ�����


#endif