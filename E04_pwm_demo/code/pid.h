#ifndef _PID_h
#define _PID_h
/******ͷ�ļ�������******/
#include "zf_common_headfile.h"

/******�ֲ����������*******/
#define PID_ANGLE_IMAX		2000  //��������
#define PID_ANGLE_VMAX		5000  //�������

/******PID�ṹ�������******/
typedef struct _PID{
	float Kp;                 //����
	float Ki;                 //����
	float Kd;                 //΢��
	float SetSpeed;						//�趨ֵ
	float AcutualSpeed;				//ʵ��ֵ
	float err;								//ƫ��ֵ
	float err_last;						//��һ��ƫ��ֵ
	float I_Max;							//��������
	float output,V_Max;							//������������
	float voltage;						//�����ѹֵ������ִ�����ı�����
	float interal;						//�������ֵ
}_pid;
/******����������******/																
void PID_Data_Init(_pid *pid);//PID���ݳ�ֵ��
void PID_Param_Init(_pid *p,float IM,float VM);//PID����Ƕ��
float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed);//PID������

#endif


