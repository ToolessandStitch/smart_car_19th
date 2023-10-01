#ifndef _efficient_h_
#define _efficient_h_

#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

void swap(uint8 arr[], int index_i, int index_j);
uint8 partition_Rowe(uint8 arr[], int low, int high);
void quick_sort(uint8 arr[], int low, int high);
uint8 Atan2(float y,float x);
float InvSqrt(float x);
void IPS_Draw(uint8 j,uint8 i);


#endif