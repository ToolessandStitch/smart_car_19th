/*********************************************************************************************************************
* RT1064DVL6A Opensourec Library ����RT1064DVL6A ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
* 
* ���ļ��� RT1064DVL6A ��Դ���һ����
* 
* RT1064DVL6A ��Դ�� ��������
* �����Ը��������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
* 
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
* 
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
* 
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ���֤Э�� �����������Ϊ���İ汾
* �������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ���֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
* 
* �ļ�����          isr
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 8.32.4 or MDK 5.33
* ����ƽ̨          RT1064DVL6A
* ��������          https://seekfree.taobao.com/
* 
* �޸ļ�¼
* ����              ����                ��ע
* 2022-09-21        SeekFree            first version
********************************************************************************************************************/
 


#ifndef _isr_h
#define _isr_h
#include "pid.h"
#include "efficient.h"
extern int i;
extern KalmanFilter filter;
extern long front_left_road,front_right_road,down_left_road,down_right_road;
extern _pid speed_Front_left,speed_Front_right,speed_down_left,speed_down_right;
extern int SetSpeed_front_left,SetSpeed_front_right,SetSpeed_down_left,SetSpeed_down_right;
extern float PWM_front_left,PWM_front_right,PWM_down_left,PWM_down_right;
extern int key_0,key_1,key_2,key_3,key_4,key_5,key_6;
extern _pid lfw,rfw,ldw,rdw;
extern float A_XY;
extern float x,y,z;
extern float turn,angle_pro,imu963ra_acc_y_pro,imu963ra_gyro_y_pro,angle_pro_max;
extern int16 imu963ra_gyro_z_last,imu963ra_gyro_y_last,imu963ra_gyro_x_last;
extern float speed_front_left1,speed_front_right1,speed_down_left1,speed_down_right1;
#endif
