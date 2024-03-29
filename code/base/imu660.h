/*
 * imu660.h
 *
 *  Created on: 2023Äê6ÔÂ23ÈÕ
 *      Author: Admin
 */

#ifndef CODE_BASE_IMU660_H_
#define CODE_BASE_IMU660_H_
#include "base.h"
struct imu660_data
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
extern struct imu660_data imu;
void imu_get_data();
void imu_init();



#endif /* CODE_BASE_IMU660_H_ */
