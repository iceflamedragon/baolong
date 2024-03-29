/*
 * motor.h
 *
 *  Created on: 2023年2月10日
 *      Author: Admin
 */

#ifndef CODE_BASE_MOTOR_H_
#define CODE_BASE_MOTOR_H_
#include "base.h"

#define STEER_PWM                 (ATOM2_CH4_P33_12)
#define MOTOR_DIR                 (P21_2) //控制方向
#define MOTOR_PWM                 (ATOM1_CH1_P21_3)

#define STEER_MID 747
#define PWM_MAX 7000

#define steer_pwm_max 1250
#define steer_pwm_min 250


void motor_init();
void set_pwm(int PWM);
void set_steer(int steer_pwm);


#endif /* CODE_BASE_MOTOR_H_ */
