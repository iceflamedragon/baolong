/*
 * timer1_pwm.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "timer.h"

void timer1_Call_Back()
{
      mycar.TIME++;
//    imu_get_data();//��ȡ����������
//    imu.pitch+=imu.gyroy*0.002;//�ǶȻ���
//    mpu_get_data();
//    angal_integeral(imu.gyroz*0.001);//�ǶȻ��ֺ���
    get_battery_value();
    beep_warning();
//      dir_pid_adjust();
    dir_control();//�������
    motor_control();//�������

}

