/*
 * adjust.h
 *
 *  Created on: 2023年6月23日
 *      Author: Admin
 */

#ifndef CODE_ADJUST_ADJUST_H_
#define CODE_ADJUST_ADJUST_H_
#include "global.h"

#include "vofa.h"
//调试相关参量
struct adjust_struct
{
   uint16 time1_count;   //中断1计次
   uint16 time2_count;   //中断2计次
   uint16 time3_count;   //中断3计次
   uint16 time4_count;   //中断4计次
   uint8 take_picture;   //拍照
   uint8 fps;            //摄像头帧率
   uint8 fps_count;      //摄像头帧率计数
   uint16 time;          //时间
   float target_gyro;    //调试时目标角速度
   float battery;
   uint8 left_line[120];
   uint8 right_line[120];
};
struct vofa_struct
{
   float speed[9];
   float scan_line[9];
   float dir[9];
   float turn[9];
   float cam_err[9];
   float Element[9];
   float loop[9];
   float adc_err[9];
   float element_rem[9];
   float obstacle[9];
   float broken[9];
   float garage[9];
   float gyro_pid[9];
};

extern struct adjust_struct adjust;
extern struct vofa_struct vofa;
extern float line_data[120][9];
extern float adjust_data[8];
extern IfxCpu_mutexLock core_original_err;
void CPU0_adjust();
void CPU1_adjust();
void timer1_adjust();
void timer4_adjust();
void vofa_display(float *data);
void swj_display();
void edge_line(uint8 y,uint8 xl,uint8 xr);
#endif /* CODE_ADJUST_ADJUST_H_ */
