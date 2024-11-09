/*
 * filter.c
 *
 *  Created on: 2024年4月18日
 *      Author: 86138
 */


#include "filter.h"
#include <math.h>
//这是一个实现了二阶Butterworth低通滤波器的代码，同时包含了一个用于计算滤波器系数的函数和一个用于进行滤波的函数。
float filter_calc(filter_handle_s *fh, float val)
//函数用于执行滤波操作
{
    int i ;
  float *b ;   //float b[3];a,b,x,y都是一个数组
  float *a ;
  float *y ;
  float *x ;
  float y0 ;
  if (!fh->is_inited) //a,b,x,y初始是一个负值，如果指向is_inited则为0，是检查有没有初始化
  {
    return 0;
  }
  b = fh->b;//在fh中寻找b并赋给b
  a = fh->a;
  y = fh->y;
  x = fh->x;
  y0 = 0.;
  for (i = fh->order; i > 0; i--) //order是fh中的
  {
    x[i] = x[i - 1];
    y[i] = y[i - 1];//都向后移一位
    y0 += b[i] * x[i] - a[i] * y[i];
  }
  x[0] = val;
  y[0] = y0 + b[0] * x[0];
  return y[0];   //最终滤波的计算结构
}

void filter_calc_butterworth_2nd_lowpass_coeff(filter_handle_s *fh, float cutoff_freq, float sample_rate)
//函数用于计算Butterworth低通滤波器的系数
{
  float *b;
  float *a;
  float *y;
  float *x;
    float ita, ita2, qita;
  const float sqrt2 = 1.4142135623730950488016887242097;//更号2的值
  const float pi = 3.1415926535897932384626433832795;//π的值
  fh->order = 2;
  b = fh->b;
  a = fh->a;
  y = fh->y;
  x = fh->x;
  x[0] = x[1] = x[2] = 0;
  y[0] = y[1] = y[2] = 0;

  if (cutoff_freq  / sample_rate < 0.002) //cutoff_freq是截止频率，sample_rate是采样频率
  {
    b[0] = b[1] = b[2] = 0;
    a[0] = a[1] = a[2] = 0;
  }
  else {
    ita = 1. / tan(pi * cutoff_freq / sample_rate);
    ita2 = ita * ita;
    qita = sqrt2 * ita;
    b[0] = 1. / (1. + qita + ita2);
    b[1] = 2. * b[0];
    b[2] = b[0];
    a[0] = 1.;
    a[1] = 2. * (1. - ita2) * b[0];
    a[2] = (1. - qita + ita2) * b[0];
  }
  fh->is_inited = 1;
}




