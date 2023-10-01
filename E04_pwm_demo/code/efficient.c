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
    quick_sort(arr, pivot_pKTos + 1, high);//右序列
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




