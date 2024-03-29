/*
 * integeral.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "integral.h"
integeral_STRUCT angle_integral,//�ǶȻ��ֽṹ��
                distance_integral;//·�̻��ֽṹ��
//��ʼ�ǶȻ��֣���ڲ���ΪĿ��Ƕ�
void begin_angle_integeral(int16 target_angle)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_Thres=target_angle;
    angle_integral.integeral_flag=1;
}
//����ǶȻ��ֱ���
void clear_angle_integeral(void)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_flag=0;
}
//�ǶȻ��ֺ��������������Ƕ�ȡ���ݵĶ�ʱ���ж���
void angal_integeral(float gyro_z)
{
    if(angle_integral.integeral_flag==1)
    {
        angle_integral.integeral_data+=gyro_z;
        if(angle_integral.integeral_Thres>0)
        {
            if(angle_integral.integeral_data>angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //angle_integral.integeral_data=0;
            }
        }
        else
        {
            if(angle_integral.integeral_data<angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //indata.YawAngle=0;
            }
        }
    }
}
//��ʼ·�̻��ֺ�������ڲ���ΪĿ����־���
void begin_distant_integeral(int target_distant)
{
    distance_integral.integeral_flag=1;
    distance_integral.integeral_data=0;
    distance_integral.integeral_Thres=target_distant;
}
//���·�̻��ֱ���
void clear_distant_integeral(void)
{
    distance_integral.integeral_flag=0;
    distance_integral.integeral_data=0;
}
//·�̻��ֺ��������ڶ�ȡ���������жϺ�����
void distant_integeral(float present_speed)  //·�̻���
{
    if(distance_integral.integeral_flag==1)
    {
        distance_integral.integeral_data+=present_speed;
        if(distance_integral.integeral_data>distance_integral.integeral_Thres)
        {
            //distance_integral.integeral_data=0;
            distance_integral.integeral_flag=2;
        }
    }
}
//��ȡ����״��
uint8 get_integeral_state(integeral_STRUCT* integeral)
{
    return integeral->integeral_flag;
}
//��ȡ��������
float get_integeral_data(integeral_STRUCT* integeral)
{
    return integeral->integeral_data;
}



