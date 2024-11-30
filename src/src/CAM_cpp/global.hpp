/*
 * global.h
 *
 *  Created on: 2024年11月6日
 *      Author: ChengFrost
 */

#ifndef CODE_GLOBAL_HPP_
#define CODE_GLOBAL_HPP_
#include <iostream>
using namespace std;
#define MYCAR 1
typedef unsigned char u8;   // 无符号  8 bits
typedef unsigned short u16; // 无符号 16 bits
typedef unsigned long u32;  // 无符号长整形 32bits
#define _LIMIT(data, min, max)                                                 \
  (((int)data) < (min) ? (min) : ((data) > (max) ? (max) : (data)))
#define _MIN(x, y) (x) < (y) ? (x) : (y)
#define abs_m(a, b) (a >= b ? a - b : b - a)
#define abs_f(a) (a > 0 ? a : -a)
typedef struct PID_para_STRUCT {
  int kp;
  int ki;
  int kd;
} PID_para_STRUCT;
struct vofa_struct {
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
  float cross[9];
  float speed_differ[9];
  float fuzzy_pid[9];
  float fuzzy_speed[9];
  float adrc[9];
  float steer_cal[9];
  float zebra[9];
  float current[9];
};
typedef struct ADRC_para_STRUCT {
  // TD para
  int r;
  int h;

  // ESO para
  int b;     // 系统系数
  int delta; // delta为fal（e，alpha，delta）函数的线性区间宽度

  int belta01; // 扩张状态观测器反馈增益1
  int belta02; // 扩张状态观测器反馈增益2
  int belta03; // 扩张状态观测器反馈增益3
  int w0;      // 扩张状态观测器带宽

  // int w=0.7;
  // int belta01;//扩张状态观测器反馈增益1
  // int belta02;//扩张状态观测器反馈增益2
  // int belta03;//扩张状态观测器反馈增益3

  // NLSEF para
  int a1;
  int a2;
  int kp; // 跟踪输入信号增益
  int kd; // 跟踪微分信号增益
} ADRC_para_STRUCT;
// extern uint8_t Grayscale[120][188];
extern float tim;
extern float cam_original_err; // 摄像头原始偏差
#include "GUI/application.cpp"
#include "GUI/application.hpp"
#include "camera_process/Binarization.hpp"
#include "camera_process/Element_recognition.cpp"
#include "camera_process/Element_recognition.hpp"
#include "camera_process/act_persp.cpp"
#include "camera_process/act_persp.hpp"
#include "camera_process/black_obstacle.cpp"
#include "camera_process/black_obstacle.hpp"
#include "camera_process/broken_circuit.cpp"
#include "camera_process/broken_circuit.hpp"
#include "camera_process/circle.cpp"
#include "camera_process/circle.hpp"
#include "camera_process/cross.cpp"
#include "camera_process/cross.hpp"
#include "camera_process/garage.cpp"
#include "camera_process/garage.hpp"
#include "camera_process/obstacle.cpp"
#include "camera_process/obstacle.hpp"
#include "camera_process/patch_line.cpp"
#include "camera_process/patch_line.hpp"
#include "camera_process/scan_line.cpp"
#include "camera_process/scan_line.hpp"
#include "camera_process/slope.cpp"
#include "camera_process/slope.hpp"
#include "camera_process/zebra.cpp"
#include "camera_process/zebra.hpp"
#include "control/err_calculation.cpp"
#include "control/err_calculation.hpp"
#include "control/filter.cpp"
#include "control/filter.hpp"
#include "control/fuzzy_pid.cpp"
#include "control/fuzzy_pid.hpp"
#include "control/integral.cpp"
#include "control/integral.hpp"
#include "control/mycar.cpp"
#include "control/mycar.hpp"
#include "control/pid.cpp"
#include "control/pid.hpp"
#endif /* CODE_GLOBAL_H_ */