/********************************************************************************************************************
* �Զ�����ԭ����㷨
* ���ļ������㷨��
* ͨѶ
* �ļ�����    pid.c
* ����ʱ�䣺2023-10-13
* ���ߣ���������������
* ��ע��1.���ļ������ڿ���С����Ҫ�Զ�����ԭ������
********************************************************************************************************************/

/******ͷ�ļ�������******/
#include "pid.h"
#include "zf_common_headfile.h"
/******���庯��******/
void PID_Data_Init(_pid *pid)	//PID ����ʼֵ��
{	
	pid->SetSpeed			=		0.0;		
	pid->AcutualSpeed	=		0.0;
	pid->err					=		0.0;
	pid->err_last			=		0.0;
	pid->interal			=		0.0;
	pid->voltage			=		0.0;
}
void PID_Param_Init(_pid *p,float IM,float VM)  //PID����Ƕ��
{
	p->I_Max	=		IM;
	p->V_Max	=		VM;
}
float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed)							//PID�㷨	
{
	pid->Kp                  =     kp;
	pid->Ki                  =     ki;
	pid->Kd                  =     kd;
	pid->AcutualSpeed		=			feedback_speed;   //����ʵ���ٶ�
	pid->SetSpeed				=			speed;            //�趨�ٶ�
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //��ֵ
	
	if(pid->interal > pid->I_Max)                          //�������ֵ���ڻ�������
		 pid->interal	=		pid->I_Max;                  //��������ֵ���趨�����ֵ
	else if(pid->interal < -pid->I_Max)                    //�������ֵС�ڸ��Ļ�������
			pid->interal	=		-pid->I_Max;                 //���Ļ�������ֵ���趨�����ֵ
		
	pid->interal	+=	pid->err;                              //�������ֵ�ۼӲ�ֵ
	pid->voltage	=		pid->Kp * pid->err + pid->Ki * pid->interal + pid->Kd * (pid->err - pid->err_last);//���㶨���ѹֵ������ִ�����ı�����
	pid->err_last	=		pid->err;                              //��һ��ƫ��ֵ
		
 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//�����ѹֵ�����������
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	return pid->voltage;                                          //���ض����ѹֵ
}




