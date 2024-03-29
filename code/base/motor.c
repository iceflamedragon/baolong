/*
 * motor.c
 *
 *  Created on: 2023年2月10日
 *      Author: Admin
 */
#include "motor.h"
//电机初始化
void motor_init()
{
    gpio_init(MOTOR_DIR, GPO, 1, GPO_PUSH_PULL);
    pwm_init(MOTOR_PWM,17000,0);
    pwm_init(STEER_PWM,50,STEER_MID);
}

void set_steer(int steer_pwm)
{
    if(steer_pwm>steer_pwm_max)
        steer_pwm=steer_pwm_max;
    else if(steer_pwm<steer_pwm_min)
        steer_pwm=steer_pwm_min;
    pwm_set_duty(STEER_PWM,steer_pwm);
}


void set_pwm(int PWM)
{
    if(PWM>PWM_MAX)PWM=PWM_MAX;
    if(PWM<-PWM_MAX)PWM=-PWM_MAX;
    if(PWM<=0)
    {
//        gpio_low(MOTOR_DIR);
        gpio_high(MOTOR_DIR);
        pwm_set_duty(MOTOR_PWM,-PWM);
//        pwm_set_duty(MOTOR_PWM,0);
    }
    else if(PWM>0)
    {
        gpio_low(MOTOR_DIR);
//        gpio_high(MOTOR_DIR);
        pwm_set_duty(MOTOR_PWM,PWM);
    }


}


