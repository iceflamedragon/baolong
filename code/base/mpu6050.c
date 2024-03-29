/*
 * mpu6050.c
 *
 *  Created on: 2023Äê6ÔÂ22ÈÕ
 *      Author: Admin
 */
#include "mpu6050.h"
//struct mpu6050_data mpu;
void mpu_get_data()
{
    mpu6050_get_acc();
    mpu6050_get_gyro();
    imu.accx=mpu6050_acc_transition(mpu6050_acc_x);
    imu.accy=mpu6050_acc_transition(mpu6050_acc_y);
    imu.accz=mpu6050_acc_transition(mpu6050_acc_z);
    imu.gyrox=-(mpu6050_gyro_transition(mpu6050_gyro_x)-imu.gyrox_base);
    imu.gyroy=-(mpu6050_gyro_transition(mpu6050_gyro_y)-imu.gyroy_base);
    imu.gyroz=-(mpu6050_gyro_transition(mpu6050_gyro_z)-imu.gyroz_base);
}
void mpu_init()
{
    float gyrox_sum,gyroy_sum,gyroz_sum,accx_sum,accy_sum,accz_sum;

    mpu6050_init();

    for(int i=0;i<100;i++)
    {
        mpu6050_get_acc();
        mpu6050_get_gyro();
        gyrox_sum+=mpu6050_gyro_transition(mpu6050_gyro_x);
        gyroy_sum+=mpu6050_gyro_transition(mpu6050_gyro_y);
        gyroz_sum+=mpu6050_gyro_transition(mpu6050_gyro_z);
        accx_sum+=mpu6050_acc_transition(mpu6050_acc_x);
        accy_sum+=mpu6050_acc_transition(mpu6050_acc_y);
        accz_sum+=mpu6050_acc_transition(mpu6050_acc_z);
        system_delay_ms(10);
    }
    imu.accx_base=accx_sum/100;
    imu.accy_base=accy_sum/100;
    imu.accz_base=accz_sum/100;
    imu.gyrox_base=gyrox_sum/100;
    imu.gyroy_base=gyroy_sum/100;
    imu.gyroz_base=gyroz_sum/100;
}


