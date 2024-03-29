/*
 * imu660.c
 *
 *  Created on: 2023年6月23日
 *      Author: Admin
 */
#include "imu660.h"
struct imu660_data imu;
//陀螺仪读取数据
void imu_get_data()
{
    //imu660ra_get_acc();
    imu660ra_get_gyro();
//    imu.accx=imu660ra_acc_transition(imu660ra_acc_x);
//    imu.accy=imu660ra_acc_transition(imu660ra_acc_y);
//    imu.accz=imu660ra_acc_transition(imu660ra_acc_z);
    imu.gyrox=imu660ra_gyro_transition(imu660ra_gyro_x);
    imu.gyroy=imu660ra_gyro_transition(imu660ra_gyro_y);
    imu.gyroz=imu660ra_gyro_transition(imu660ra_gyro_z);
    imu.yaw+=imu.gyroz*0.001;
}

void imu_init()
{
//    float gyrox_sum,gyroy_sum,gyroz_sum,accx_sum,accy_sum,accz_sum;
    imu660ra_init();
/*    for(int i=0;i<100;i++)
    {
        imu660ra_get_acc();
        imu660ra_get_gyro();
        gyrox_sum+=imu660ra_gyro_transition(imu660ra_gyro_x);
        gyroy_sum+=imu660ra_gyro_transition(imu660ra_gyro_y);
        gyroz_sum+=imu660ra_gyro_transition(imu660ra_gyro_z);
        accx_sum+=imu660ra_acc_transition(imu660ra_acc_x);
        accy_sum+=imu660ra_acc_transition(imu660ra_acc_y);
        accz_sum+=imu660ra_acc_transition(imu660ra_acc_z);
        system_delay_ms(10);
    }
    imu.accx_base=accx_sum/100;
    imu.accy_base=accy_sum/100;
    imu.accz_base=accz_sum/100;
    imu.gyrox_base=gyrox_sum/100;
    imu.gyroy_base=gyroy_sum/100;
    imu.gyroz_base=gyroz_sum/100;*/
}


