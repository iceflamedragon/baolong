/*
 * act_persp.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "act_persp.h"
// 逆透视变换矩阵
// float
// a1=0.488,a2=-0.000,a3=-0.000,b1=-0.776,b2=0.371,b3=-0.008,c1=47.835,c2=3.883;
// float
// a1=0.4571,a2=0.0000,a3=0.0000,b1=-0.7694,b2=0.3314,b3=-0.0086,c1=48.1971,c2=4.9143;
double a1 = 0.457143, a2 = 0.000000, a3 = 0.000000, b1 = -0.801429,
       b2 = 0.331429, b3 = -0.008571, c1 = 50.757143, c2 = 4.914286;
// struct persp_struct persp_line[50];
// uint8_t persp_x_left[50][94];
// uint8_t persp_x_right[50][94];
int16_t persp_x[80][188];
uint8_t persp_y[80];
// 静态逆透视初始化（先计算一遍，存入数组，后面就不用再算一遍了，以空间换时间）
void act_perst_init() {
  int ox, oy;
  for (int y = 0; y < 80; y++) // x轴只存放30行到79行的数据，节省空间
  {
    for (int x = 0; x < 188; x++) {
      ox = (int)((a1 * x + b1 * y + c1) / (a3 * x + b3 * y + 1));
      persp_x[y][x] = (int16_t)(ox);
    }
  }
  for (int y = 0; y < 80; y++) // y轴全部存放
  {
    oy = (int)((a2 * 0 + b2 * y + c2) / (a3 * 0 + b3 * y + 1));
    if (oy > 255)
      ox = 255;
    else if (oy < 0)
      ox = 0;
    persp_y[y] = (uint8_t)oy;
  }
}

// 逆透视变换函数
int16_t get_persp_data_ox(int16_t x, int16_t y) {
  if (y >= 0 && y < 80)
    return (persp_x[y][x]);
  else        // 如果不在范围内就计算
    return 0; //(a1*x+b1*y+c1)/(a3*x+b3*y+1);
              // return (a1*x+b1*y+c1)/(a3*x+b3*y+1);
}
int16_t get_persp_data_oy(int16_t x, int16_t y) {
  if (y >= 0 && y < 80)
    return persp_y[y]; // 如果在存储范围内就直接取出
  else                 // 如果不在范围内就计算
    return 0;          //(a2*x+b2*y+c2)/(a3*x+b3*y+1);
}

// 进行逆透视变换并计算斜率（以下为祖传算法）
void persp_task(int16_t xl, int16_t xr, int16_t y) {
  int16_t oxl, oxr, oyl, oyr;
  // 限幅
  if (xl < 0)
    xl = 0;
  else if (xl > 187)
    xl = 187;
  if (xr < 0)
    xr = 0;
  else if (xr > 187)
    xr = 187;
  // 记录补线后的结果
  lineinfo[y].left_adjust = xl;
  lineinfo[y].right_adjust = xr;
  // 记录扫线值
  //  edge_line(y,xl,xr);
  // if(y<80)
  {
    oxl = get_persp_data_ox(xl, y);
    oyl = get_persp_data_oy(xl, y);
    oxr = get_persp_data_ox(xr, y);
    oyr = get_persp_data_oy(xr, y);
    lineinfo[y].persp_lx = oxl;
    lineinfo[y].persp_ly = oyl;
    lineinfo[y].persp_rx = oxr;
    lineinfo[y].persp_ry = oyr;

    // 计算此行左边界点到左轮的斜率的倒数的1000倍
    lineinfo[y].angel_left =
        125 * (lineinfo[y].persp_lx - setpara.camwl) /
        (lineinfo[y].persp_ly - setpara.camwf); // persp_lx=67
    // 计算此行右边界点到右轮的斜率的倒数的1000倍
    lineinfo[y].angel_right =
        125 * (lineinfo[y].persp_rx - setpara.camwr) /
        (lineinfo[y].persp_ry - setpara.camwf); // persp_lf=97
  }
}
