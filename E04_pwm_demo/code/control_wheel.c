#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "electrical_machine.h"
#include "efficient.h"
#include "pid.h"
#include "math.h"
#include "control_wheel.h"
//转向环
_pid turn_a,turn_b;
float turn_speed;
float speed_fl,speed_fr,speed_bl,speed_br; 
void motor_angle(float angle_turn)
{
		PID_Data_Init(&turn_a);
		PID_Data_Init(&turn_a);
		PID_Param_Init(&turn_a,20000,1000);
		PID_Data_Init(&turn_b);
		PID_Data_Init(&turn_b);
		PID_Param_Init(&turn_b,20000,1000);
		turn_speed=PID_Calc(&turn_a,0.78,0,0.03,angle_turn,angle_pro);
		motor_control_pid(-turn_speed,turn_speed,-turn_speed,turn_speed);
		printf("left  %f   %f\r\n",angle_pro,turn_speed);
//	printf("%d  %d  %d  %d  \r\n",front_left_counts,front_right_counts,down_left_counts,down_right_counts);
}

void motor_control_pid(int FL,int FR,int BL,int BR)
{
				PWM_front_left=PID_Calc(&speed_Front_left,30,0.15,0,(float)FL,(float)front_left_counts);


				PWM_front_right=PID_Calc(&speed_Front_right,30,0.2,0,(float)FR,(float)front_right_counts);
			
				PWM_down_left=PID_Calc(&speed_down_left,30,0.4,0,(float)BL,(float)down_left_counts);

				PWM_down_right=PID_Calc(&speed_down_right,35,0.3,0.01,(float)BR,(float)down_right_counts);
			
				motor_control(PWM_down_left,PWM_down_right,PWM_front_right,PWM_front_left);
	
}
void motor_control_road(int FL,int FR,int BL,int BR)
{
				//23,0.18,0
				speed_front_left1=PID_Calc(&lfw,4,0,1.4,(float)5000,(float)FL);
				PWM_front_left=PID_Calc(&speed_Front_left,3,0,0.9,(float)speed_front_left1,(float)front_left_counts);

				speed_front_right1=PID_Calc(&rfw,3,0,1.0,(float)5000,(float)FR);
				PWM_front_right=PID_Calc(&speed_Front_right,4,0,0.5,(float)speed_front_right1,(float)front_right_counts);
			
				speed_down_left1=PID_Calc(&ldw,3.1,0,0.99,(float)5000,(float)BL);
				PWM_down_left=PID_Calc(&speed_down_left,3.8,0,0.54,(float)speed_down_left1,(float)down_left_counts);
			
				speed_down_right1=PID_Calc(&rdw,3.3,0,1.45,(float)5000,(float)BR);
				PWM_down_right=PID_Calc(&speed_down_right,3,0,0.9,(float)speed_down_right1,(float)down_right_counts);
			
				motor_control(PWM_down_left,PWM_down_right,PWM_front_right,PWM_front_left);

}
/*磁力计相关数据详解
y轴朝向为正南或正北为0
x轴朝向为正东或正西为0
z轴表示车体的俯仰程度
										北y=0
					x<0				|      y<0
					y>0				|			 x<0
										|
										|
										|
	x=0西---------------------------东x=0
										|
					x>0				|			x>0
					y>0				|			y<0
										|
										|
										南y=0
*/