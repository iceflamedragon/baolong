/*
 * ecoder.h
 *
 *  Created on: 2023年2月10日
 *      Author: Admin
 */

#ifndef CODE_BASE_ENCODER_H_
#define CODE_BASE_ENCODER_H_
#include "base.h"

#define ENCODER             TIM2_ENCODER
#define ENC_CHA             TIM2_ENCODER_CH1_P33_7
#define ENC_CHB             TIM2_ENCODER_CH2_P33_6
#define ENCODER_COUNT       5500        //meter


typedef struct
{
    int left_speed;    //左轮原始速度
    int right_speed;    //右轮原始速度

    float left_arrange_speed; //左轮滑动平均滤波后的速度
    float right_arrange_speed;//右轮滑动平均滤波后的速度

 /////单个电机
    int speed;                  //        原始速度
    float  arrange_speed;       //滑动平均滤波后的速度
}encoder_STRUCT;
extern encoder_STRUCT encoder;
void encoder_init();
float encoder_get_speed_Left();
float encoder_get_speed_Right();
float moving_average_filtre_right_speed(float xn);
float moving_average_filtre_left_speed(float xn);
float speed_Kalman_Filter(float speed_measure,float accel_x);
float speed_Kalman_Filter2(float present_speed);

#endif /* CODE_BASE_ENCODER_H_ */
