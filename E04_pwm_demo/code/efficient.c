/********************************************************************************************************************
* 高效函数代码
*
* 高效算法包含：Atan2求反正切、swap交换算法、Glenn W. Rowe划分算法、快速排序算法、卡马尔开方算法
*（高效算法是用来提高MCU运行效率和降低算法复杂度的，和具体物理实现无关，调用总钻风代码时可忽略不管）
*
* 文件名称          efficient.c 
* 创建时间：2023-9-14
* 作者：长了牙的无牙仔


* 备注：以下代码为本人通过查阅论文和技术博客文章整理而来，具体为啥是这样的我也不太清楚，如果读者想深入了解
*       学习的话，可以参考readme里的文章链接
********************************************************************************************************************/

//头文件调用
#include "efficient.h"
#include "zf_common_headfile.h"
#include "zf_common_debug.h"
#include "isr.h"
#include "electrical_machine.h"
#include "efficient.h"
#include "pid.h"
#include "math.h"
void swap(uint8 arr[], int index_i, int index_j)
{
  //将数组相应位置的两个数相交换
  uint8 k = arr[index_i];
  arr[index_i] = arr[index_j];
  arr[index_j] = k;
} 


//采用Glenn W. Rowe划分算法
uint8 partition_Rowe(uint8 arr[], int low, int high)
{
  //根据一个基准数，将数组分为基准数左边小于基准数，基准数右边大于或等于基准数的两部分
  //返回值是基准数在数组中的下标
  //这里选取数组元素的第0位作为基准数
  //low为最低下标，high为最高下标
  uint8 pivot = arr[low];//选取基准数
  uint8 low_index = low;
  for (int i = low + 1; i <= high; i++)
  {
    if (arr[i] < pivot)
  {
    //在序列中找到一个比pivot小的，就递增low_index
    low_index++;
    if (i!=low_index)//如果i和low_index相等，则在i之前都不存在需要交换的比pivot大的数
      {
        swap(arr, i, low_index);
      }
  }
  }
  //low_index的位置就是pivot应处在的位置，low_index指向的总是比pivot小的数
  arr[low] = arr[low_index];
  arr[low_index] = pivot;
  return low_index;
}


//快速分类
void quick_sort(uint8 arr[], int low, int high)
{
  if (high > low)//如果需要排序的序列的元素个数大于1
  {
    uint8 pivot_pos = partition_Rowe(arr, low, high);
    quick_sort(arr, low, pivot_pos - 1);//左序列
    quick_sort(arr, pivot_pos + 1, high);//右序列
  }
}


//反正切函数的简化写法
uint8 Atan2(float y,float x){
  float tanNum;
  uint8 direction;//像素点的方向，值为Gy/Gx
  tanNum = y/x;
  //0.41421356对应的是22.5°，2.41421356对应的是67.5°
  //可以尝试0.57735026对应30°，1.73205080对应的是60°
  if ( tanNum> -0.41421356 && tanNum< 0.41421356 )  direction=0;//水平方向
  else if( tanNum>= 0.41421356 && tanNum< 2.41421356)  direction=1;//左下、右上
  else if( tanNum<= -0.41421356 && tanNum> -2.41421356)  direction=2;//左上、右下
//  if ( tanNum> -0.57735026 && tanNum< 0.57735026 )  direction=0;//水平方向
//  else if( tanNum>= 0.57735026 && tanNum< 1.73205080)  direction=1;//左下、右上
//  else if( tanNum<= -0.57735026 && tanNum> -1.73205080)  direction=2;//左上、右下
  else  direction=3;//竖直方向
  return direction;
}


//卡马尔开平方算法
float InvSqrt(float x){
    float xhalf = 0.5f*x;
    int i = *(int*)&x; // get bits for floating VALUE
    i = 0x5f3759df- (i>>1); // gives initial guess y0
    x = *(float*)&i; // convert bits BACK to float
    x = x*(1.5f-xhalf*x*x); // Newton step, repeating increases accuracy
    return 1/x;
}
//陀螺仪一阶互补滤波
float acc_ratio = 0;      //加速度计比例
float gyro_ratio = 14.08;    //陀螺仪比例,结合示波器调参
float dt = 0.001;           //采样周期,赶紧实际情况填写
float alpha; 
float angle_calc(float angle_m, float gyro_m)
{
    float temp_angle;
    float gyro_now;
    static float angle;
    float accel_angle;
    static uint8 first_angle;
		angle += gyro_m * dt;
		accel_angle=angle_m;
		temp_angle= alpha*(angle + gyro_m * dt)+(1 - alpha) * accel_angle;
    return temp_angle*29.8507;
}
////一阶互补滤波对角度的预测
//void update_attitude() 
//	{
//    // 使用陀螺仪数据更新角度估计
//    angle += gyro_rate * dt;
//    
//    // 使用加速度计数据估计重力加速度角度
//    accel_angle = /* 加速度计数据处理 */;
//    
//    // 使用互补滤波器进行数据融合
//    angle = alpha * (angle + gyro_rate * dt) + (1 - alpha) * accel_angle;
//}
int16 imu963ra_acc_x_last,imu963ra_acc_y_last,imu963ra_acc_z_last;
//陀螺仪角度z值均值滤波函数
float angle_z_time()
{
				imu963ra_gyro_z=0.82*imu963ra_gyro_z_last+0.01*(imu963ra_gyro_z+12);
				imu963ra_gyro_z_last=imu963ra_gyro_z;
	return imu963ra_gyro_z_last;	
}
//陀螺仪角度y值均值滤波函数
float angle_y_time()
{
				imu963ra_gyro_y=0.82*imu963ra_gyro_y_last+0.01*(imu963ra_gyro_y+12);
				imu963ra_gyro_y_last=imu963ra_gyro_y;
	return imu963ra_gyro_y_last;	
}
//陀螺仪角度x值均值滤波函数
float angle_x_time()
{
				imu963ra_gyro_x=0.82*imu963ra_gyro_x_last+0.01*(imu963ra_gyro_x+12);
				imu963ra_gyro_x_last=imu963ra_gyro_x;
	return imu963ra_gyro_x_last;	
}
//陀螺仪加速度x值均值滤波函数
float imu963ra_acc_x_time()
{
				imu963ra_acc_x=0.65*imu963ra_acc_x_last+0.01*(imu963ra_acc_x-28);
				imu963ra_acc_x_last=imu963ra_acc_x;
	return imu963ra_acc_x_last;	
}
//陀螺仪加速度y值均值滤波函数
float imu963ra_acc_y_time()
{
				imu963ra_acc_y=0.82*imu963ra_acc_y_last+0.01*(imu963ra_acc_y+12);
				imu963ra_acc_y_last=imu963ra_acc_y;
	return imu963ra_acc_y_last;	
}
//陀螺仪加速度z值均值滤波函数
float imu963ra_acc_z_time()
{
				imu963ra_acc_z=0.85*imu963ra_acc_z_last+0.01*(imu963ra_acc_z-4150);
				imu963ra_acc_z_last=imu963ra_acc_z;
	return imu963ra_acc_z_last;	
}

// 初始化卡尔曼滤波器
void kalmanFilterInit(KalmanFilter* filter, float Q_angle, float Q_bias, float R_measure) {
    filter->Q_angle = Q_angle;
    filter->Q_bias = Q_bias;
    filter->R_measure = R_measure;
    filter->angle = 0.0;
    filter->bias = 0.0;
    filter->P[0][0] = 0.0;
    filter->P[0][1] = 0.0;
    filter->P[1][0] = 0.0;
    filter->P[1][1] = 0.0;
}

// 更新卡尔曼滤波器
float kalmanFilterUpdate(KalmanFilter* filter, float gyroData, float accelData, float dt) {
    // 更新估计的角度和偏置预测
    filter->angle += (gyroData - filter->bias) * dt;

    // 更新估计误差协方差矩阵预测
    filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_angle);
    filter->P[0][1] -= dt * filter->P[1][1];
    filter->P[1][0] -= dt * filter->P[1][1];
    filter->P[1][1] += filter->Q_bias * dt;

    // 计算测量残差
    float y = accelData - filter->angle;

    // 更新观测
    float S = filter->P[0][0] + filter->R_measure;
    float K[2];
    K[0] = filter->P[0][0] / S;
    K[1] = filter->P[1][0] / S;

    // 更新估计
    filter->angle += K[0] * y;
    filter->bias += K[1] * y;

    // 更新估计误差协方差矩阵
    float P00_temp = filter->P[0][0];
    float P01_temp = filter->P[0][1];

    filter->P[0][0] -= K[0] * P00_temp;
    filter->P[0][1] -= K[0] * P01_temp;
    filter->P[1][0] -= K[1] * P00_temp;
    filter->P[1][1] -= K[1] * P01_temp;

    return filter->angle;
}

void adjustFilterParameters(KalmanFilter* filter) 
	{
    // 观察陀螺仪输出的角度数据的波动情况
    if (imu963ra_gyro_z_last > 0.1) {
        filter->Q_angle += 0.001;
    } else {
        filter->Q_angle -= 0.001;
    }
    
    // 观察陀螺仪输出的零偏情况
    if (imu963ra_gyro_z > 0.05) {
        filter->Q_bias += 0.001;
    } else {
        filter->Q_bias -= 0.001;
    }
    
    // 观察加速度计输出的加速度数据的波动情况
    if (imu963ra_acc_z_last > 0.1) {
        filter->R_measure += 0.01;
    } else {
        filter->R_measure -= 0.01;
    }
}
void beiyong_angle()
{
				turn=((imu963ra_gyro_z+5)/16.4)*0.001f;
				if(fabs(turn)<0.04)
				{
					turn=0;
				}
				A_XY=2*turn+A_XY;
				A_XY=A_XY;
}