/*
 * err_calculation.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "err_calculation.hpp"
#include "math.h"
#include "mycar.hpp"
// 小车偏差值计算（输出为原始偏差）
extern int imgshowmode;
void original_err_calculation() {
  // Element=broken_circuit;
  switch (Element) {
  case broken_circuit:
    if (mycar.tracking_mode == 0)
      mycar.original_err = cam_err_calculation();
    // else beep(20);
    break;
  case outgarage:
    if (watch.out_garage_flag == 2)
      mycar.original_err = -setpara.out_garage_err * 0.1;
    else if (watch.out_garage_flag == 4)
      mycar.original_err = setpara.out_garage_err * 0.1;
    else
      mycar.original_err = 0;
    break;
  case Slope:
    mycar.original_err = cam_err_calculation();
    break;
  case obstacle:
    mycar.original_err = 0;
    break;
  case ingarage:
    if (watch.zebra_flag == 3 || watch.zebra_flag == 6)
      mycar.original_err = 0;
    else
      mycar.original_err = cam_err_calculation();
    break;
  default:
    mycar.original_err = cam_err_calculation();
    break;
  }
}

// 摄像头err计算
#define far_lim 80 // 计算切线的最远行
#define d_can_err_limit 30
#define cam_limit 200
float cam_err_calculation() {
  ////////////////////////舵机角度计算, 220527单车修改
  int AngleLeft = (int)0x80000000;
  int AngleRight = 0x7FFFFFFF;
  // int AngleLeft=lineinfo[80].angel_left;
  // int AngleRight=lineinfo[80].angel_right;
  int AngleLeftLast = 0, AngleRightLast = 0;
  int target_line_y = forward_far - 2;

  int watchleft = setpara.far_line;
  int watchright = setpara.far_line;
  int left_lost_count = 0, right_lost_count = 0;
  static float angle_target, angle_target_last;
  static float err, d_err;

  for (int y = watch.angle_near_line; y < watch.angle_far_line;
       y++) // 打角范围遍历，可以根据不同元素的需求更改遍历的打角
  {
    if (0.625 * ((int)lineinfo[y].persp_ly + lineinfo[y].persp_ry) <
        forward_near) {
      continue;
    }

    // 取得左轮到左边线的切线的斜率与右轮到右边线的切线的斜率（没有三岔路时，通常最多只有一边能取到切线）
    if (left_lost_count < 10) {
      if (y > 40) {
        if (lineinfo[y].left_lost && Element == None && watch.track_count < 75)
          left_lost_count++;
        else
          left_lost_count = 0;
      }
      if (AngleLeft < lineinfo[y].angel_left
          //&&!lineinfo[y].left_lost
          && (abs(AngleLeft - AngleLeftLast) < 400)) {
        AngleLeft =
            lineinfo[y].angel_left; // 寻找斜率最小的线（即左切线）的斜率
        watchleft = y; // 寻找斜率最小的线的切点所在行
      }
    }
    if (right_lost_count < 10) {
      if (y > 40) {
        if (lineinfo[y].right_lost && Element == None && watch.track_count < 75)
          right_lost_count++;
        else
          right_lost_count = 0;
      }
      if (AngleRight > lineinfo[y].angel_right
          //&&!lineinfo[y].right_lost

          && (abs(AngleRight - AngleRightLast) < 400)) {
        AngleRight =
            lineinfo[y].angel_right; // 寻找斜率最大的线（即右切线）的斜率
        watchright = y; // 寻找斜率最大小的线的切点所在行
      }
    }
    // 如果某一行左斜率大于右斜率（说明此行有一条线已经越过垂直线），或者此行太远了，那么停止计算斜率
    if (AngleLeft > AngleRight ||
        0.4 * ((int)lineinfo[y].persp_ly + lineinfo[y].persp_ry) >
            forward_far) {
      target_line_y = y - 1;
      break;
    }
    AngleLeftLast = AngleLeft;
    AngleRightLast = AngleRight;
    /*        line_data[y][0]=y;
            line_data[y][1]=lineinfo[y].angel_left;
            line_data[y][2]=lineinfo[y].angel_right;
            line_data[y][3]=(float)AngleLeft;
            line_data[y][4]=(float)AngleRight;
            line_data[y][5]=lineinfo[y].persp_lx;
            line_data[y][6]=mycar.original_err;
            line_data[y][7]=mycar.original_err;*/

    // vofa_display(vofa.cam_err);
  }
  watch.watchleft = watchleft + 1;
  watch.watchright = watchright + 1;
  /*    vofa.scan_line[0]=(float)AngleLeftLast;
      vofa.scan_line[1]=(float)AngleRightLast;
      vofa.scan_line[2]=(float)AngleLeftLast+AngleRightLast;*/

  // vofa_display(vofa.cam_err);
  // 最终原始偏差值为左斜率与右斜率之和
  /*    if(AngleLeftLast==lineinfo[80].angel_left&&lineinfo[80].left_lost==1)//左线全丢
          angle_target=atan(0.001*AngleLeftLast) + atan(0.001*AngleRightLast);
      else
     if(AngleRightLast==lineinfo[80].angel_right&&lineinfo[80].right_lost==1)//右线全丢
          angle_target=atan(0.001*AngleLeftLast) + atan(0.001*AngleRightLast);
      else */
  // angle_target=atan(0.001*AngleLeftLast) + atan(0.001*AngleRightLast);
  //    angle_target=atan(0.001*(AngleLeftLast+AngleRightLast));
  angle_target = AngleLeftLast + AngleRightLast;
  // 对变化量进行限制
  if ((angle_target - angle_target_last) > d_can_err_limit) {
    angle_target = angle_target_last + d_can_err_limit;
  } else if ((angle_target - angle_target_last) < -d_can_err_limit) {
    angle_target = angle_target_last - d_can_err_limit;
  }
  if (angle_target > cam_limit)
    angle_target = cam_limit;
  else if (angle_target < -cam_limit)
    angle_target = -cam_limit;
  angle_target_last = angle_target;

  // if(angle_target==0)beep(20);
  // original_err=(float)AngleLeftLast+(float)AngleRightLast;
  // line_data[100][8]=mycar.original_err;
  // vofa.scan_line[3]=angle_target;
  // 以下内容关于显示
  if (imgshowmode == 3) // 188*120
  {
    // 在赛道最远端画一条水平线
    // ips200_draw_horizon(0, 119 - watch.watch_lost, 188, 119 -
    // watch.watch_lost, RGB565_WHITE);
  }
  if (1) // 128*128
  {
    // 两轮位置
    imo4[0][setpara.camwl] = 3; // 76
    imo4[0][setpara.camwr] = 4; // 97     原来为4
    // 变化点位置
    for (uint8_t y = forward_near; y < 100; y++) {
      uint16_t imo_l =
          (setpara.camwl +
           AngleLeftLast * (lineinfo[y].persp_ly - setpara.camwf) / 125);
      uint16_t imo_r =
          (setpara.camwr +
           AngleRightLast * (lineinfo[y].persp_ry - setpara.camwf) / 125);
      if (imo_l < (188-1) && imo_l >= 0 && lineinfo[y].persp_ly < 119 &&
          lineinfo[y].persp_ly > 0) {
        imo4[lineinfo[y].persp_ly][imo_l] = 3;
      }
      if (imo_r < (188 - 1) && imo_r >= 0 && lineinfo[y].persp_ry < 119 &&
          lineinfo[y].persp_ry > 0)
          {
        imo4[lineinfo[y].persp_ry][imo_r] = 3;}
    }
  }

  return (1.f + 1e-7f * setpara.SteerKpchange * angle_target * angle_target) *
         angle_target;
}

int prev_track_diff_cutoff_freq = 0;

filter_handle_s track_diff_filter;
int Steer_PWM_Cal(int track_pos) {
  static float prev_nonlinear_track_pos = 0.f;
  float cutoff = 5;
  int steertemp;

  // 计算非线性车道线位置
  mycar.nonlinear_trackpos = track_pos;
  // 计算比例系数
  mycar.Steer_PWM_Kp = (int)(CAM_Turn.kp_ratio * CAM_Turn.PID_para->kp *
                             mycar.nonlinear_trackpos);

  // 车道线位置差分
  if ((!track_diff_filter.is_inited) ||
      setpara.track_diff_cutoff != prev_track_diff_cutoff_freq) {
    prev_track_diff_cutoff_freq = setpara.track_diff_cutoff;
    if (setpara.track_diff_cutoff > 5) {
      cutoff = (float)setpara.track_diff_cutoff;
    }
    //        filter_calc_butterworth_2nd_lowpass_coeff(&track_diff_filter,
    //        cutoff, 1000 / 2);//pit_ms=6.25
  }

  // 计算车道线位置变化率
  //    mycar.raw_track_diff =(float)(mycar.nonlinear_trackpos -
  //    prev_nonlinear_track_pos);
  mycar.raw_track_diff =
      0.6 * mycar.raw_track_diff -
      (float)(0.4 * mycar.d_original_err); //(float)(mycar.nonlinear_trackpos -
                                           // prev_nonlinear_track_pos);

  //    mycar.track_diff = filter_calc(&track_diff_filter,
  //    mycar.raw_track_diff);
  mycar.track_diff = mycar.raw_track_diff;

  // 计算微分系数
  if (fabs(mycar.track_diff) <= 2)
    mycar.Steer_PWM_Kd = 0;
  else
    mycar.Steer_PWM_Kd =
        (int)(CAM_Turn.kd_ratio * CAM_Turn.PID_para->kd * mycar.track_diff);
  // 计算方向盘 PWM
  //    mycar.Steer_PWM_Kp = (mycar.Steer_PWM_Kp > 900 ? 900 :
  //    (mycar.Steer_PWM_Kp < -900 ? -900 : mycar.Steer_PWM_Kp));
  mycar.Steer_PWM_Kd =
      (mycar.Steer_PWM_Kd > 600
           ? 600
           : (mycar.Steer_PWM_Kd < -600 ? -600 : mycar.Steer_PWM_Kd)); // 限幅

  steertemp = mycar.Steer_PWM_Kp + mycar.Steer_PWM_Kd;
  prev_nonlinear_track_pos = mycar.nonlinear_trackpos;
  steertemp =
      (steertemp > 1000 ? 1000 : (steertemp < -1000 ? -1000 : steertemp));

  return 1.f * 500 * steertemp / 1000;
}
