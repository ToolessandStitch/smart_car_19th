#ifndef _mt9v03x_h_
#define _mt9v03x_h_

#include "zf_common_headfile.h"

#define LOW_THRESHOLD           (100)//小于为黑
#define HIGH_THRESHOLD          (230)//大于为白

#define NMS_LOSS                (0)//非极大值抑制补偿,初始设置为0；

extern uint8 bina_image[MT9V03X_H][MT9V03X_W];// 简单二值化图像
extern uint8 Blur_image[MT9V03X_H][MT9V03X_W];// 高斯模糊图像
extern uint8 mean_image[MT9V03X_H][MT9V03X_W];//均值二值化图像
extern uint8 Sobel_image[MT9V03X_H][MT9V03X_W];//sobel边沿提取图像
extern uint8 Scharr_image[MT9V03X_H][MT9V03X_W];//scharr边沿提取图像
extern uint8 dir[MT9V03X_H][MT9V03X_W];//dir用来保存像素点的方向梯度
extern uint8 nms[MT9V03X_H][MT9V03X_W];//nms用来保存非极大值抑制之后的图像
extern uint8 link[MT9V03X_H][MT9V03X_W];//双阈值+边缘链接的图像


void simple_binaryzation(const uint8 *image0,uint8 height,uint8 width,uint8 threshold);
void mean_binaryzation(const uint8 *image2,uint8 height,uint8 width);
void Gaussain_Blur(uint8 *image1);
void Sobel_edge(uint8 *image3);
void Scharr_edge(uint8 *image4);
void NMS(void);
void TwoThreshold(void);
void my_camera(uint8 camera_ips_show);
#endif