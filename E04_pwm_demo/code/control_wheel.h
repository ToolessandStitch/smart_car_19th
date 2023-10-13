#ifndef _CONTROL_WHEEL_H
#define _CONTROL_WHEEL_H

typedef struct _motor
{
	void (*motor_angle)(float angle_turn);
	void (*motor_control_pid)(int FL,int FR,int BL,int BR);
	void (*motor_control_road)(int FL,int FR,int BL,int BR);

}_motor;


void motor_angle(float angle_turn);
void motor_control_pid(int FL,int FR,int BL,int BR);
void motor_control_road(int FL,int FR,int BL,int BR);
#endif