/********************************************************************************************************************
* 总钻风Canny二值化处理算法
*
* 该文件包含算法：
* 简单二值化、均值二值化、高斯模糊、sobel算子、scharr算子、
* 非极大值抑制（NMS）、双阈值法、边缘链接
*
* 文件名称          mt9v03x.c 
* 创建时间：2023-9-14
* 作者：长了牙的无牙仔

* 备注：1. Canny算子是一整套图像处理的集合，其中包含了高斯模糊、Sobel算子（Scharr算子）、非极大值抑制、双边缘检测与链接
*       2. readme里会提供详细讲解相关算法的文章链接
********************************************************************************************************************/
/******头文件的声明******/
#include "mt9v03x.h"
#include "efficient.h"
//全局变量
uint8 bina_image[MT9V03X_H][MT9V03X_W];         // 简单二值化图像
uint8 Blur_image[MT9V03X_H][MT9V03X_W];         // 高斯模糊图像
uint8 mean_image[MT9V03X_H][MT9V03X_W];         //均值二值化图像
uint8 Sobel_image[MT9V03X_H][MT9V03X_W];        //sobel边沿提取图像
uint8 Scharr_image[MT9V03X_H][MT9V03X_W];       //scharr边沿提取图像
uint8 dir[MT9V03X_H][MT9V03X_W];                //dir用来保存像素点的方向梯度
uint8 nms[MT9V03X_H][MT9V03X_W];                //nms用来保存非极大值抑制之后的图像
uint8 link[MT9V03X_H][MT9V03X_W];               //双阈值+边缘链接的图像


/**************************************************************************
函数功能：简单二值化
入口参数：(const uint8 *image0,uint8 height,uint8 width,uint8 threshold)
         （图像，图像高度，图像宽度，二值化阈值）
返回值：无
备注：该函数处理后的结果保存在二维数组bina_image中，可直接对该数组进行数据处理
调用示例：simple_binaryzation((const uint8 *)mt9v03x_image,MT9V03X_H,MT9V03X_W,120);
**************************************************************************/
void simple_binaryzation(const uint8 *image0,uint8 height,uint8 width,uint8 threshold){
  uint16 i,j,temp=0;
  for(i=0;i<height;i++)
  {
    for(j=0;j<width;j++)
    {
      temp=*(image0+i*width+j);
        if(temp<threshold)
          bina_image[i][j]=0;
        else
          bina_image[i][j]=255;
    }
  }
}


/**************************************************************************
函数功能：均值二值化
入口参数：(const uint8 *image1,uint8 height,uint8 width)
         （图像，图像高度，图像宽度）
返回值：无
备注：该函数处理后的结果保存在二维数组mean_image中，可直接对该数组进行数据处理
调用示例：mean_binaryzation((const uint8 *)mt9v03x_image,MT9V03X_H,MT9V03X_W);
**************************************************************************/
void mean_binaryzation(const uint8 *image1,uint8 height,uint8 width){
        uint32 gray_sum=0;
        uint16 i,j,temp1,temp2,meannum=0;
	for(int i=0;i<height;i++)
	{
		for(int j=0;j<width;j++)
		{
			temp1 = *(image1+i*width+j);
			gray_sum += temp1;
		}
	}
	meannum = gray_sum/(height*width);//平均阈值
        for(i=0;i<height;i++)
        {
                for(j=0;j<width;j++)
                {
                        temp2=*(image1+i*width+j);
                        if(temp2<meannum)
                            mean_image[i][j]=0;
                        else
                            mean_image[i][j]=255;
    }
  }      
}


/**************************************************************************
函数功能：高斯模糊
入口参数：uint8 *image2  （image2指的是需要进行高斯模糊处理的图像）
返回值：无
备注：该函数处理后的结果保存在二维数组Blur_image中，可直接对该数组进行数据处理
调用示例：Gaussain_Blur(*mt9v03x_image);//对总钻风的原始图像进行高斯模糊
**************************************************************************/
void Gaussain_Blur(uint8 *image2) {
	uint8 *map;
        
	float Gaussain_weight_matrix[9] = { 0.0947416, 0.118318, 0.0947416, 0.118318, 0.147761, 0.118318, 0.0947416, 0.118318, 0.0947416 };
	//Gaussain_weight_matrix对应的是高斯模糊算法的权重矩阵，当前这个是常用的权重矩阵，网上也有其他的权重矩阵，读者可自行查找使用
        
        uint8 temp_fix[9];
	map = image2;	// 获取原灰度图
	for (int j = 0; j < MT9V03X_W ; ++j) {	// 第一行不做处理
		Blur_image[0][j] = *(map);
		map++;
	}
	for (int i = 1; i < MT9V03X_H - 1; ++i) {
		Blur_image[i][0] = *(map);	// 第一列不做处理
		map++;
		/* 高斯模糊 */
		for (int j = 1; j < MT9V03X_W  - 1; ++j) {
			temp_fix[0] = *(map - MT9V03X_W  - 1);
			temp_fix[1] = *(map - MT9V03X_W );
			temp_fix[2] = *(map - MT9V03X_W  + 1);
			temp_fix[3] = *(map - 1);
			temp_fix[4] = *(map);
			temp_fix[5] = *(map + 1);
			temp_fix[6] = *(map + MT9V03X_W  - 1);
			temp_fix[7] = *(map + MT9V03X_W );
			temp_fix[8] = *(map + MT9V03X_W  + 1);
			float fix_sum = 0;
			for (int k = 0; k < 9; ++k) {
				fix_sum += (float)temp_fix[k] * Gaussain_weight_matrix[k];
			}
			Blur_image[i][j] = (int)fix_sum;//直接灰度值赋值，要用一个int（强制类型转化）来表示0-255之间的整数值
			map++;
		}
		Blur_image[i][MT9V03X_W  - 1] = *(map); // 最后一列不做处理
		map++;
	}
	for (int j = 0; j < MT9V03X_W ; ++j) { // 最后一行不做处理
		Blur_image[MT9V03X_H - 1][j] = *(map);
		map++;
	}
}


/**************************************************************************
函数功能：Sobel算子
入口参数：uint8 *image3  （image3指的是需要进行sobel算子处理的图像）
返回值：无
备注：该函数处理后的结果保存在二维数组Sobel_image中，可直接对该数组进行数据处理
调用示例：Sobel_edge(*Blur_image);//对高斯模糊之后的图像进行sobel算子处理
**************************************************************************/
void Sobel_edge(uint8 *image3){
  int Gx,Gy,G=0;
  uint8 sobel_fix[9];
  uint8* map;
  map = image3;
  for(int j=0;j<MT9V03X_W;j++){   //第一行不处理
    Sobel_image[0][j]=*(map);
    map++;
  }
  for(int i=1;i<MT9V03X_H-1;i++){
    Sobel_image[i][0]=*(map);  //第一列不处理
    map++;
    for(int j=1;j<MT9V03X_W-1;j++){
      	sobel_fix[0] = *(map - MT9V03X_W - 1);
	sobel_fix[1] = *(map - MT9V03X_W);
	sobel_fix[2] = *(map - MT9V03X_W + 1);
	sobel_fix[3] = *(map - 1);
	sobel_fix[4] = *(map);
	sobel_fix[5] = *(map + 1);
	sobel_fix[6] = *(map + MT9V03X_W - 1);
	sobel_fix[7] = *(map + MT9V03X_W);
	sobel_fix[8] = *(map + MT9V03X_W + 1);
        Gx = abs(sobel_fix[2]-sobel_fix[0]+2*sobel_fix[5]-2*sobel_fix[3]+sobel_fix[8]-sobel_fix[6]);
        Gx = Gx>255?255:Gx;//限幅
        Gy = abs(sobel_fix[0]-sobel_fix[6]+2*sobel_fix[1]-2*sobel_fix[7]+sobel_fix[2]-sobel_fix[8]);
        Gy = Gy>255?255:Gy;//限幅
        G = (int)InvSqrt(Gx*Gx+Gy*Gy);//用到了卡马尔开方InvSqrt
        Sobel_image[i][j]=G;
        map++;
    }
    Sobel_image[i][MT9V03X_W-1]=*(map);  //最后一列不处理
    map++;
  }
  for(int j=0;j<MT9V03X_W;j++){
    Sobel_image[MT9V03X_H-1][j]=*(map);
    map++;
  }
}


/**************************************************************************
函数功能：Scharr算子
入口参数：uint8 *image4  （image4指的是需要Scharr算子处理的图像）
返回值：无
备注：1. Scharr算子是对sobel算子的改进，一般都是使用scharr算子
         两者在代码上的区别主要是经过不同的算子得到Gx和Gy
      2. 该函数处理后的结果保存在二维数组Scharr_image中，可直接对该数组进行数据处理
调用示例：Scharr_edge(*Blur_image);//对高斯模糊之后的图像进行Scharr算子处理
**************************************************************************/
void Scharr_edge(uint8 *image4){
  int Gx,Gy,G=0;
  uint8 scharr_fix[9];
  uint8* map;
  map = image4;
  for(int j=0;j<MT9V03X_W;j++){   //第一行不处理
    Scharr_image[0][j]=*(map);
    map++;
  }
  for(int i=1;i<MT9V03X_H-1;i++){
    Scharr_image[i][0]=*(map);  //第一列不处理
    map++;
    for(int j=1;j<MT9V03X_W-1;j++){
      	scharr_fix[0] = *(map - MT9V03X_W - 1);
	scharr_fix[1] = *(map - MT9V03X_W);
	scharr_fix[2] = *(map - MT9V03X_W + 1);
	scharr_fix[3] = *(map - 1);
	scharr_fix[4] = *(map);
	scharr_fix[5] = *(map + 1);
	scharr_fix[6] = *(map + MT9V03X_W - 1);
	scharr_fix[7] = *(map + MT9V03X_W);
	scharr_fix[8] = *(map + MT9V03X_W + 1);
        Gx = 3*scharr_fix[2]-3*scharr_fix[0]+10*scharr_fix[5]-10*scharr_fix[3]+3*scharr_fix[8]-3*scharr_fix[6];
        Gy = 3*scharr_fix[0]-3*scharr_fix[6]+10*scharr_fix[1]-10*scharr_fix[7]+3*scharr_fix[2]-3*scharr_fix[8];
        G = (int)InvSqrt(Gx*Gx+Gy*Gy);//用到卡马尔开方
        G = G>255?255:G; 
        Scharr_image[i][j]=G;
        dir[i][j]=Atan2(Gy,Gx);//用到了Atan2算法，dir表示像素梯度的方向，会在NMS()函数中用到
        map++;
    }
    Scharr_image[i][MT9V03X_W-1]=*(map);  //最后一列不处理
    map++;
  }
  for(int j=0;j<MT9V03X_W;j++){     //最后一行不处理
    Scharr_image[MT9V03X_H-1][j]=*(map);
    map++;
  }
}


/**************************************************************************
函数功能：非极大值抑制（NMS）
入口参数：无（默认是对scharr算子处理后的图形进行非极大值抑制）
返回值：无
备注：1. 函数中的dir在scharr算子函数中赋值
      2. 该函数处理后的结果保存在二维数组nms中，可直接对该数组进行数据处理
调用示例：NMS();//调用该函数会给nms[MT9V03X_H][MT9V03X_W]赋值
**************************************************************************/
void NMS(){
  uint8 loss=NMS_LOSS;//非极大值抑制补偿，根据实际效果来确定用不用，可手动调，初始设置为0；

  for(int i=1;i<MT9V03X_H-1;i++){
    for(int j=1;j<MT9V03X_W-1;j++){
      switch(dir[i][j])
      {
      case 0:
        {
          if( (Scharr_image[i][j]>Scharr_image[i][j+1]-loss) && (Scharr_image[i][j]>Scharr_image[i][j-1]-loss) )
          nms[i][j]=Scharr_image[i][j];       //这里默认在scharr图像的基础上进行非极大值抑制，如果要修改的话切记这行以及下面的scharr都要修改
          else  nms[i][j]=0;
          
          break;
        }
      case 1:
        {
          if( (Scharr_image[i][j]>Scharr_image[i+1][j-1]-loss) && (Scharr_image[i][j]>Scharr_image[i-1][j+1]-loss) )
          nms[i][j]=Scharr_image[i][j];
          else  nms[i][j]=0;
          
          break;
        }
      case 2:
        {
          if( (Scharr_image[i][j]>Scharr_image[i-1][j-1]-loss) && (Scharr_image[i][j]>Scharr_image[i+1][j+1]-loss) )
          nms[i][j]=Scharr_image[i][j];
          else  nms[i][j]=0;
          
          break;
        }
      case 3:
        {
          if( (Scharr_image[i][j]>Scharr_image[i+1][j]-loss) && (Scharr_image[i][j]>Scharr_image[i-1][j]-loss) )
          nms[i][j]=Scharr_image[i][j];
          else  nms[i][j]=0;
          
          break;
        }
      default:
        break;
      }
    
    }

  }
}


/**************************************************************************
函数功能：双阈值法+边缘链接
入口参数：无（暂时默认是对NMS后的图像进行处理）
返回值：无
备注：该函数处理后的结果保存在二维数组link中，可直接对该数组进行数据处理
调用示例：TwoThreshold();//调用该函数会给link[MT9V03X_H][MT9V03X_W]赋值
**************************************************************************/
void TwoThreshold(){
  uint8 Link_fix[8];//当前点不算，只读取八邻域的值
  static uint16 lowThr,highThr;
  
  lowThr=LOW_THRESHOLD;//低阈值设定
  highThr=HIGH_THRESHOLD;//高阈值设定

  for(int i=1;i<MT9V03X_H-1;i++){
    for(int j=1;j<MT9V03X_W-1;j++){
      if(nms[i][j]<lowThr)  link[i][j]=0;
      else if(nms[i][j]>highThr)  link[i][j]=255;
      else{
        Link_fix[0]=nms[i-1][j-1];
        Link_fix[1]=nms[i-1][j];
        Link_fix[2]=nms[i-1][j+1];
        Link_fix[3]=nms[i][j-1];
        Link_fix[4]=nms[i][j+1];
        Link_fix[5]=nms[i+1][j-1];
        Link_fix[6]=nms[i+1][j];
        Link_fix[7]=nms[i+1][j+1];
        quick_sort(Link_fix,0,8);//快速排序算法，对八领域的灰度值进行排序;7最大，0最小
        if(Link_fix[4]>highThr)  link[i][j]=255;//满足条件，链接
        else  link[i][j]=0;//不满足条件，灰度值置为0(黑)       
      }
    }
  }

}


/**************************************************************************
函数功能：Canny算法封装
入口参数：camera_ips_show；
返回值：无
备注：1. camera_ips_show为0表示不开启屏幕，为1表示开启屏幕显示
      2. 这个函数只是示例，将前面用到的算法封装成一个函数方便调用，读者可以自行选择调用
调用示例：my_camera(1);
**************************************************************************/
void my_camera(uint8 camera_ips_show)
{
  Gaussain_Blur(*mt9v03x_image); //高斯模糊
  Scharr_edge(*mt9v03x_image);//Scharr算子
  NMS();//非极大值抑制（NMS）
  TwoThreshold();//双边缘检测和边缘链接

  //下面这个if else只是图像显示处理而已，我这里对应的是逐飞ips200的屏幕，如果你用的是其他型号的屏幕就改成对应屏幕的显示函数
    if(mt9v03x_finish_flag&&camera_ips_show)
   {
     ips200_displayimage03x((const uint8 *)link, 188, 120);
     mt9v03x_finish_flag = 0;
   }
  else 
    mt9v03x_finish_flag=0;
}



