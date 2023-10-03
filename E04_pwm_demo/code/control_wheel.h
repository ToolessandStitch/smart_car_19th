#ifndef _CONTROL_WHEEL_H
#define _CONTROL_WHEEL_H

extern void motor_angle(float angle_turn);
extern void motor_control_pid(int FL,int FR,int BL,int BR);
extern void motor_control_road(int FL,int FR,int BL,int BR);
#endif