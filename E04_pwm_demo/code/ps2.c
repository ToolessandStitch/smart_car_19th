/********************************************************************************************************************
* 遥控器手柄通讯
*
* 该文件包含算法：
* 通讯
*
* 文件名称          mt9v03x.c 
* 创建时间：2023-9-14
* 作者：长了牙的无牙仔

* 备注：1. Canny算子是一整套图像处理的集合，其中包含了高斯模糊、Sobel算子（Scharr算子）、非极大值抑制、双边缘检测与链接
*       2. readme里会提供详细讲解相关算法的文章链接
********************************************************************************************************************/




#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "ps2.h"
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
void PS2_Init(void)
{
	gpio_init(D13,GPO,0,GPO_PUSH_PULL);
	gpio_init(D14,GPO,0,GPO_PUSH_PULL);
	gpio_init(D15,GPO,0,GPO_PUSH_PULL);
	
	gpio_init(D12,GPO,0,FAST_GPI_PULL_DOWN);
	DO_H;
	CLC_H;
	CS_H;

}

//读取手柄数据
uint8 PS2_ReadData(uint8 command)
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

//对读出来的 PS2 的数据进行处理
//按下为 0， 未按下为 1
unsigned char PS2_DataKey()
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

//得到一个摇杆的模拟量	 范围0~256
uint8 PS2_AnologData(uint8 button)
{
	return Data[button];
}

//清除数据缓冲区
void PS2_ClearData()
{
	uint8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
