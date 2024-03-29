/*
 * ecoder.h
 *
 *  Created on: 2023��2��10��
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
    int left_speed;    //����ԭʼ�ٶ�
    int right_speed;    //����ԭʼ�ٶ�

    float left_arrange_speed; //���ֻ���ƽ���˲�����ٶ�
    float right_arrange_speed;//���ֻ���ƽ���˲�����ٶ�

 /////�������
    int speed;                  //        ԭʼ�ٶ�
    float  arrange_speed;       //����ƽ���˲�����ٶ�
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
