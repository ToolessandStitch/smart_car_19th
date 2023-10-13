#ifndef _EIGHT_NEIGHBORHOOD_H
#define _EIGHT_NEIGHBORHOOD_H
#include "mt9v03x.h"
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "electrical_machine.h"
#include "efficient.h"
#include "pid.h"
#include "math.h"
#include "control_wheel.h"

//��ɫ����  ��Ϊ������������ɫ���Ĳ�������ֱ�ӷ�����
#define uesr_RED     0XF800    //��ɫ
#define uesr_GREEN   0X07E0    //��ɫ
#define uesr_BLUE    0X001F    //��ɫ




//�궨��
#define image_h	120//ͼ��߶�
#define image_w	188//ͼ����

#define white_pixel	255
#define black_pixel	0

#define bin_jump_num	1//�����ĵ���
#define border_max	image_w-2 //�߽����ֵ
#define border_min	1	//�߽���Сֵ	
extern uint8 original_image[image_h][image_w];
extern uint8 bin_image[image_h][image_w];//ͼ������

extern void image_process(void); //ֱ�����жϻ�ѭ������ô˳���Ϳ���ѭ��ִ����

#endif 

