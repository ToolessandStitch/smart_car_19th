#include "pid.h"
#include "zf_common_headfile.h"
_pid L1_PID={'\0'};
_pid R1_PID;
_pid L2_PID;
_pid R2_PID;
_pid stPID_Distance={'\0'};
_pid stPID_Distance1={'\0'};
void PID_Init(void)//PID 始值化
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

void PID_Data_Init(_pid *pid)	//PID 数据始值化
{	
	pid->SetSpeed			=		0.0;		
	pid->AcutualSpeed	=		0.0;
	pid->err					=		0.0;
	pid->err_last			=		0.0;
	pid->interal			=		0.0;
	pid->voltage			=		0.0;
}


void PID_Param_Init(_pid *p,float IM,float VM)  //PID参数嵌入
{
//	p->Kp			=		kp;
//	p->Ki			=		ki;
//	p->Kd			=		kd;
	p->I_Max	=		IM;
	p->V_Max	=		VM;
}

float PID_Calc(_pid *pid,float kp,float ki,float kd,float speed,float feedback_speed)							//PID算法	
{
	pid->Kp                  =     kp;
	pid->Ki                  =     ki;
	pid->Kd                  =     kd;
	pid->AcutualSpeed		=			feedback_speed;   //反馈实际速度
	pid->SetSpeed				=			speed;            //设定速度
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
		 pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
			pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值
		
	pid->interal	+=	pid->err;                              //定义积分值累加差值
	pid->voltage	=		pid->Kp * pid->err + pid->Ki * pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	return pid->voltage;                                          //返回定义电压值

}
float Location_pid(_pid *pid,float kp,float ki,float kd,float target,float feedback_target){

pid->Kp                  =     kp;
	pid->Ki                  =     ki;
	pid->Kd                  =     kd;
	pid->AcutualSpeed		=			feedback_target;   //反馈实际距离
	pid->SetSpeed				=			target;            //设定距离
	pid->err						=			pid->SetSpeed		-		pid->AcutualSpeed;  //差值
	
	if(pid->interal > pid->I_Max)                          //定义积分值大于积分上限
		 pid->interal	=		pid->I_Max;                  //积分上限值赋予定义积分值
	else if(pid->interal < -pid->I_Max)                    //定义积分值小于负的积分上限
			pid->interal	=		-pid->I_Max;                 //负的积分上限值赋予定义积分值
		
	pid->interal	+=	pid->err;                              //定义积分值累加差值
	pid->voltage	=		pid->Kp * pid->err + pid->Ki * pid->interal + pid->Kd * (pid->err - pid->err_last);//计算定义电压值（控制执行器的变量）
	pid->err_last	=		pid->err;                              //上一个偏差值
		
 if(pid->voltage > pid->V_Max) 		pid->voltage 	= pid->V_Max;//定义电压值大于输出上限
	else if(pid->voltage < -pid->V_Max) 	pid->voltage	=	-pid->V_Max;	
		
	return pid->voltage;                                          //返回定义电压值
}	
//float pid_nest_Cala(_pid *pid,float kp_position,float ki_position,float kd_position,float kp_speed,float ki_speed,float kd_speed)
//{
//	Location_pid(pid,kp_position,ki_position,kd_position);
//}


