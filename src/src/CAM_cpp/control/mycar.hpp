/*

 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CONTROL_MYCAR_HPP_
#define CODE_CONTROL_MYCAR_HPP_
#include "../global.hpp"
// #include "err_calculation.hpp"
// // #include "adc_err_calculation.h"
// #include "integral.hpp"
// #include "pid.hpp"
// #include "fuzzy_pid.hpp"
// // #include "ADRC.h"
// #include "filter.hpp"

#define PI 3.14151926
#define TREAD 15;
#define WHEEL_DISTANCE 24;
#define STEER_MID 747 // 换了频率之后一定要调整
// 小车状态标志位
typedef struct mycar_STRUCT {
  ////板卡特用参数
  float uart_speed; // 发送的速度值，
  uint16_t uart_servo; // 发送的PWM值，经过上位机各种控制计算后需要发出的值

  ////
  //       u8 status;   //小车运行状态
  uint32_t RUNTIME; // 小车运行时间(单位为10ms)//////可能改为1ms
  /*
          u32 save_times;
          u32 savenum;
          u32 save_img_count;
          u32 senddata;
          u32 WAITTIME;//发车等待时间
          uint8 out_garage;//出车库标记，发车时刷新
          uint8 in_garage;//入库标记
  */
  uint8_t car_running; // 小车发车状态 1：发车  0：停止
  uint8_t car_stop;    // 刹车标记位，置1时刹车
  float present_speed; ////当前平均速度（滤波后的）
  float present_speed2;
  float left_speed,      // 左侧电机读取的速度(本质上是脉冲数)
      right_speed,       // 右侧电机读取的速度(本质上是脉冲数)
      original_err,      // 原始偏差值
      last_original_err, // 上次偏差
      d_original_err,    // 原始偏差值变化量
      target_gyro,       // 目标角速度
      gyro_err,          // 角速度偏差
      Speed_err,         // 方向环输出的转向的速度差
      target_current_l,  // 左速度环PID输出的电流值
      target_current_r;  // 右速度环PID输出的电流值
  float target_left_speed;
  float target_right_speed;

  int16_t left_pwm_set, // 左侧电机要设定的PWM
      right_pwm_set,    // 右侧电机要设定的PWM
      speed_PWM;        // 由速度环PID输出的值

  int16_t speed_Middle_PWM, // 速度环输出的PWM中值
      Turn_PWM;             // 方向环输出的PWM差值
  int16_t steer_pwm;
  float target_speed; // 小车当前目标速度
  uint8_t CircCount;  // 小车当前圈数
  uint8_t speed_ctrl; // 设置当前小车速度(偏差越大，速度越慢)
  uint8_t pid_ctrl;
  uint8_t tracking_mode; // 循迹方式，设1时用电感循迹

  float speed_differ;       // 左右轮差速（百分比）
  float speed_left_differ;  // 左轮差速（百分比）
  float speed_right_differ; // 右轮差速（百分比）

  float nonlinear_trackpos; // 非线性车道线位置
  float raw_track_diff;     // 车道线位置变化率
  int Steer_PWM_Kp;
  int Steer_PWM_Kd;
  float track_diff;
  uint8_t start_warn_flag;
  uint8_t track_warn_flag;
  int steer_buchang;
  float fan_buchang;
  int left_corner_flag;
  int right_corner_flag;
  int corner_flag;
  int distance_count;
  float imu_speed;
  int stop_mode;
} mycar_STRUCT;

extern mycar_STRUCT mycar;
void speed_pid_adjust();
void common_running();
void running_protect();
void car_begin();
void car_stop();
void dir_control();
void speed_ctrl();
void current_control();
float speed_differ_set(int16_t steer_pwm);
float speed_left_differ_set(int16_t steer_pwm);  // 设置左轮差速
float speed_right_differ_set(int16_t steer_pwm); // 设置左轮差速
void set_speed(uint16_t speed);
void fan_begin(uint8_t speed);
void steer_buchang_cal();
void fan_buchang_cal();
#include "err_calculation.hpp"
// #include "adc_err_calculation.h"
#include "fuzzy_pid.hpp"
#include "integral.hpp"
#include "pid.hpp"
// #include "ADRC.h"
#include "filter.hpp"
#endif /* CODE_CONTROL_MYCAR_H_ */