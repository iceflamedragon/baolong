/*
 * timer1_pwm.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "timer.h"

void timer1_Call_Back()
{
      mycar.TIME++;
//    imu_get_data();//获取陀螺仪数据
//    imu.pitch+=imu.gyroy*0.002;//角度积分
//    mpu_get_data();
//    angal_integeral(imu.gyroz*0.001);//角度积分函数
    get_battery_value();
    beep_warning();
//      dir_pid_adjust();
    dir_control();//舵机控制
    motor_control();//电机控制

}

