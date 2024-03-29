/*
 * base.h
 *
 *  Created on: 2023Äê6ÔÂ20ÈÕ
 *      Author: Admin
 */

#ifndef CODE_BASE_BASE_H_
#define CODE_BASE_BASE_H_
#include "global.h"
#include "beep.h"
#include "encoder.h"
#include "inductance_adc.h"
#include "motor.h"
#include "tof.h"
#include "mpu6050.h"
#include "imu660.h"


void hardwareinit();
void clear_all_flags(void);
void GPIO_Init(void);

#endif /* CODE_BASE_BASE_H_ */
