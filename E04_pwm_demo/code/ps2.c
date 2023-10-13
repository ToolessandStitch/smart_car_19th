/********************************************************************************************************************
* ң�����ֱ�ͨѶ
* ���ļ������㷨��
* ͨѶ
* �ļ�����    ps2.c
* ����ʱ�䣺2023-10-13
* ���ߣ���������������
* ��ע��1.���ļ���������ң��������С�������
				2.�޷��ô��ļ�ʵ�ֳ������ԵĻ���
********************************************************************************************************************/

/******ͷ�ļ�������******/
#include "zf_common_debug.h"
#include "zf_driver_gpio.h"
#include "zf_common_headfile.h"
#include "fsl_gpio.h"
#include "zf_driver_encoder.h"
#include "ps2.h"
/******ȫ�ֱ����Ķ���******/
uint16 Handkey;
uint8 Comd[2]={0x01,0x42};	//��ʼ�����������
uint8 scan[9]={0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};//{0x01,0x42,0x00,0x5A,0x5A,0x5A,0x5A,0x5A,0x5A};	// ���Ͷ�ȡ
uint8 Data[9]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //���ݴ洢����
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
	};	//����ֵ�밴����

/******���庯��******/
void PS2_Init(void)//������ʼ��
{
	gpio_init(D13,GPO,0,GPO_PUSH_PULL);
	gpio_init(D14,GPO,0,GPO_PUSH_PULL);
	gpio_init(D15,GPO,0,GPO_PUSH_PULL);
	gpio_init(D12,GPO,0,FAST_GPI_PULL_DOWN);
	DO_H;
	CLC_H;
	CS_H;
}
uint8 PS2_ReadData(uint8 command)//��ȡ�ֱ�����
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
unsigned char PS2_DataKey()//�Զ������� PS2 �����ݽ��д�������Ϊ 0�� δ����Ϊ 1
{
	uint8 index = 0, i = 0;

	PS2_ClearData();
	CS_L;
	for(i=0;i<9;i++)	//����ɨ�谴��
	{
		Data[i] = PS2_ReadData(scan[i]);	
	} 
	CS_H;
	Handkey=(Data[4]<<8)|Data[3];     //����16������  ����Ϊ0�� δ����Ϊ1
	for(index=0;index<16;index++)
	{	    
		if((Handkey&(1<<(MASK[index]-1)))==0)
			return index+1;
	}
	return 0;          //û���κΰ�������
}
uint8 PS2_AnologData(uint8 button)//�õ�һ��ҡ�˵�ģ����	 ��Χ0~256
{
	return Data[button];
}
void PS2_ClearData()//������ݻ�����
{
	uint8 a;
	for(a=0;a<9;a++)
		Data[a]=0x00;
}
