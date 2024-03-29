/*
 * ecoder.c
 *
 *  Created on: 2023��2��10��
 *      Author: Admin
 */
#include "encoder.h"
//5500������1m
//�ۺ�ÿһǧ������8.6����
//ÿ������Լ 8.6x10^-5 ��
//������϶�ʱ���жϼ����ʵ���ٶ�
//��������ʼ��
encoder_STRUCT encoder;
void encoder_init()
{
    encoder_quad_init(ENCODER,ENC_CHA,ENC_CHB);
}
//��ȡ�����������ݣ���������ƽ���˲�����λ��m/s
float encoder_get_speed_Left()
{
    int16 speed;
    static float speed_least;
    speed=encoder_get_count(ENCODER);
    encoder_clear_count(ENCODER);
    speed_least=moving_average_filtre_left_speed(speed);
    return -speed_least;
}

//��ȡ�����������ݣ���������ƽ���˲���

float encoder_get_speed()
{
    static float speed_least;
    int16 speed;
    speed=encoder_get_count(ENCODER);
    encoder_clear_count(ENCODER);
    speed_least=moving_average_filtre_right_speed(speed);
    return speed_least;
}

//�ٶȻ���ƽ���˲�����
#define MVF_LENGTH_present_speed 10 /*����ƽ���˲�������*/
float moving_average_filtre_left_speed(float xn)
{
  static int index = -1;
  static float buffer[MVF_LENGTH_present_speed];
  static float sum = 0;
  float yn = 0;
  int i = 0;
  if(index == -1)
  {
    //��ʼ��
    for(i = 0; i <MVF_LENGTH_present_speed; i++)
    {
      buffer[i] = xn;
    }
    sum = xn*MVF_LENGTH_present_speed;
    index = 0;
  }
  else
  {
    sum -= buffer[index];
    buffer[index] = xn;
    sum += xn;
    index++;
    if(index >= MVF_LENGTH_present_speed)
    {
      index = 0;
    }
  }
  yn = sum/MVF_LENGTH_present_speed;
  return yn;
}

float moving_average_filtre_right_speed(float xn)
{
  static int index = -1;
  static float buffer[MVF_LENGTH_present_speed];
  static float sum = 0;
  float yn = 0;
  int i = 0;
  if(index == -1)
  {
    //��ʼ��
    for(i = 0; i <MVF_LENGTH_present_speed; i++)
    {
      buffer[i] = xn;
    }
    sum = xn*MVF_LENGTH_present_speed;
    index = 0;
  }
  else
  {
    sum -= buffer[index];
    buffer[index] = xn;
    sum += xn;
    index++;
    if(index >= MVF_LENGTH_present_speed)
    {
      index = 0;
    }
  }
  yn = sum/MVF_LENGTH_present_speed;
  return yn;
}
//�ٶ��ں�
void speed_fusion(float speed_measure,float accel_x)
{

}
/************************�������˲�**************************************/
float best_estimate_value=10;//���Ź���ֵ
float best_estimate_err=2;  //���Ź���ƫ��
float Kalman_K;           //Kֵ
#define encoder_err 2    //���������
float speed_Kalman_Filter(float speed_measure,float accel_x)
{
    Kalman_K=best_estimate_err/(best_estimate_err+encoder_err);
    best_estimate_value=best_estimate_err+Kalman_K*(speed_measure-best_estimate_value);
    best_estimate_err=(1-Kalman_K)*best_estimate_err;
    return best_estimate_value;
}

float X_pre,P_pre,X_kalman_last,P_kalman_last,X_kalman=10,P_kalman=2 , speed_ec;   //����ֵ
float H , Q = 0.001 , R = 0.5;
#define INTEGRAL_RANGE 1000
float speed_Kalman_Filter2(float present_speed)
{
    P_kalman_last = P_kalman;
    X_kalman_last = X_kalman;      //��ȡ��һ��X
    X_pre = X_kalman_last;
    P_pre = P_kalman_last + Q;
    H = P_pre / (P_pre + R);
    P_kalman = P_pre - H * P_pre;
    X_kalman = X_pre + H * (present_speed - X_pre);
    speed_ec = X_kalman - X_kalman_last;
    return X_kalman;
    //indata.ad_pid_out = setpara.ad_kp * X_kalman * 0.1f + setpara.ad_kd * speed_ec * 0.1f ;
}

