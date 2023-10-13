/********************************************************************************************************************
* 遥控器手柄通讯
* 该文件包含算法：
* 通讯
* 文件名称    ps2.c
* 创建时间：2023-10-13
* 作者：长了牙的无牙仔
* 备注：1.此文件仅用于用遥控器控制小车的情况
				2.无法用此文件实现车机电脑的互联
********************************************************************************************************************/

/******头文件的声明******/
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "ps2.h"
/******全局变量的定义******/
uint16 Handkey;
uint8 Comd[2]={0x01,0x42};	//开始命令。请求数据
uint8 scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//{0x01,0x42,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};	// 类型读取
uint8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //数据存储数组
uint16 MASK[]={
    PSB_SELECT,
    PSB_L3,
    PSB_R3 ,
    PSB_START,
    PSB_PAD_UP,
    PSB_PAD_RIGHT,
    PSB_PAD_DOWN,
    PSB_PAD_LEFT,
    PSB_L2,
    PSB_R2,
    PSB_L1,
    PSB_R1 ,
    PSB_GREEN,
    PSB_RED,
    PSB_BLUE,
    PSB_PINK
	};	//按键值与按键明

/******具体函数******/
void PS2_Init(void)//按键初始化
{
	gpio_init(D13,GPO,0,GPO_PUSH_PULL);
	gpio_init(D14,GPO,0,GPO_PUSH_PULL);
	gpio_init(D15,GPO,0,GPO_PUSH_PULL);
	gpio_init(D12,GPO,0,FAST_GPI_PULL_DOWN);
	DO_H;
	CLC_H;
	CS_H;
}
uint8 PS2_ReadData(uint8 command)//读取手柄数据
{

	uint8 i,j=1;
	uint8 res=0; 
    for(i=0; i<=7; i++)          
    {
		if(command&0x01)
			DO_H;
		else
			DO_L;
		command = command >> 1;
		system_delay_us(10);
		CLC_L;
		system_delay_us(10);
		if(DI) 
			res = res + j;
		j = j << 1; 
		CLC_H;
		system_delay_us(10);	 
    }
    DO_H;
	system_delay_us(50);
    return res;	
}
unsigned char PS2_DataKey()//对读出来的 PS2 的数据进行处理，按下为 0， 未按下为 1
{
	uint8 index = 0, i = 0;

	PS2_ClearData();
	CS_L;
	for(i=0;i<9;i++)	//更新扫描按键
	{
		Data[i] = PS2_ReadData(scan[i]);	
	} 
	CS_H;
	Handkey=(Data[4]<<8)|Data[3];     //这是16个按键  按下为0， 未按下为1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
			return index+1;
	}
	return 0;          //没有任何按键按下
}
uint8 PS2_AnologData(uint8 button)//得到一个摇杆的模拟量	 范围0~256
{
	return Data[button];
}
void PS2_ClearData()//清除数据缓冲区
{
	uint8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
