#ifndef _ps2_h_
#define _ps2_h_

#include "zf_common_typedef.h"

#define DI   gpio_get_level(D12)           //PB12  输入

#define DO_H gpio_set_level(D13,GPIO_HIGH)       //命令位高
#define DO_L gpio_set_level(D13,GPIO_LOW)       //命令位低

#define CS_H gpio_set_level(D14,GPIO_HIGH)       //CS拉高
#define CS_L gpio_set_level(D14,GPIO_LOW)      //CS拉低

#define CLC_H gpio_set_level(D15,GPIO_HIGH)      //时钟拉高
#define CLC_L gpio_set_level(D15,GPIO_LOW)      //时钟拉低



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
#define PSS_RX 5                //右摇杆X轴数据
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8



extern uint8 Data[9];
extern uint16 MASK[16];
extern uint16 Handkey;

extern void PS2_Init(void);
extern uint8 PS2_DataKey(void);		  //键值读取
extern uint8 PS2_AnologData(uint8 button); //得到一个摇杆的模拟量
extern void PS2_ClearData(void);	  //清除数据缓冲区


#endif