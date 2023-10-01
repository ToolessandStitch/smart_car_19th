#include "pid.h"
#include "zf_common_headfile.h"
_pid L1_PID={'\0'};
_pid R1_PID;
_pid L2_PID;
_pid R2_PID;
_pid stPID_Distance={'\0'};
_pid stPID_Distance1={'\0'};
void PID_Init(void)//PID ʼֵ��
{
				PID_Param_Init(&L1_PID,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
        PID_Data_Init(&L1_PID);
        PID_Param_Init(&R1_PID,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
        PID_Data_Init(&R1_PID);
        PID_Param_Init(&L2_PID,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
        PID_Data_Init(&L2_PID);
        PID_Param_Init(&R2_PID,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
        PID_Data_Init(&R2_PID);
	
				PID_Param_Init(&stPID_Distance,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
				PID_Data_Init(&stPID_Distance);
				PID_Param_Init(&stPID_Distance1,PID_ANGLE_IMAX,PID_ANGLE_VMAX);
				PID_Data_Init(&stPID_Distance1);
}

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
//	p->Kp			=		kp;
//	p->Ki			=		ki;
//	p->Kd			=		kd;
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
float Location_pid(_pid *pid,float kp,float ki,float kd,float target,float feedback_target){

pid->Kp                  =     kp;
	pid->Ki                  =     ki;
	pid->Kd                  =     kd;
	pid->AcutualSpeed		=			feedback_target;   //����ʵ�ʾ���
	pid->SetSpeed				=			target;            //�趨����
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
//float pid_nest_Cala(_pid *pid,float kp_position,float ki_position,float kd_position,float kp_speed,float ki_speed,float kd_speed)
//{
//	Location_pid(pid,kp_position,ki_position,kd_position);
//}


