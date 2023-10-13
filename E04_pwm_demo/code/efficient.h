#ifndef _efficient_h_
#define _efficient_h_

#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>


typedef struct {
    float Q_angle;     // �����ǽǶȲ�������Э����
    float Q_bias;      // ������ƫ�ò�������Э����
    float R_measure;   // ���ٶȼƲ�������Э����
    float angle;       // ���ƵĽǶ�
    float bias;        // ���Ƶ�ƫ��
    float P[2][2];     // �������Э�������
} KalmanFilter;



void swap(uint8 arr[], int index_i, int index_j);
uint8 partition_Rowe(uint8 arr[], int low, int high);
void quick_sort(uint8 arr[], int low, int high);
uint8 Atan2(float y,float x);
float InvSqrt(float x);
void IPS_Draw(uint8 j,uint8 i);
float angle_calc(float angle_m, float gyro_m);
float angle_x_time();
float angle_y_time();
float angle_z_time();
float imu963ra_acc_x_time();
float imu963ra_acc_y_time();
float imu963ra_acc_z_time();
extern float angle_pro_last;
extern int16 imu963ra_acc_x_last,imu963ra_acc_y_last,imu963ra_acc_z_last;
void kalmanFilterInit(KalmanFilter* filter, float Q_angle, float Q_bias, float R_measure);
float kalmanFilterUpdate(KalmanFilter* filter, float gyroData, float accelData, float dt);
void adjustFilterParameters(KalmanFilter* filter) ;
void beiyong_angle();
float angle_pro_time();
extern float Q_angle, Q_bias, R_measure,alpha;
#endif