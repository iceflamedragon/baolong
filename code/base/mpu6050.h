/*
 * mpu6050.h
 *
 *  Created on: 2023Äê6ÔÂ22ÈÕ
 *      Author: Admin
 */

#ifndef CODE_BASE_MPU6050_H_
#define CODE_BASE_MPU6050_H_
#include "base.h"
struct mpu6050_data
{
     float pitch;
     float roll;
     float yaw;
     float gyrox;
     float gyroy;
     float gyroz;
     float accx;
     float accy;
     float accz;
     float gyrox_base;
     float gyroy_base;
     float gyroz_base;
     float accx_base;
     float accy_base;
     float accz_base;
};
//extern struct mpu6050_data mpu;
void mpu_get_data();
void mpu_init();


#endif /* CODE_BASE_MPU6050_H_ */
