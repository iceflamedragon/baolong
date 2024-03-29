/*
 * pid.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CONTROL_PID_H_
#define CODE_CONTROL_PID_H_
#include "global.h"

//extern struct PID_para_STRUCT;
extern PID_STRUCT  ADC_Turn,     //电磁转向PID
            gyro,
            CAM_Turn,    //摄像头转向PID
            Speed_left,  //左轮速度PID
            Speed_right, //右轮速度PID
            Speed_middle,//速度中值PID
            current_left,//左轮电流环PID
            current_right;//右轮电流环PID
void change_pid_para(PID_STRUCT *PID,PID_para_STRUCT*PID_para);
void clear_integral(PID_STRUCT *PID);
void PID_init();
float ADC_Turn_PID(float present_value);
float CAM_Turn_PID(float present_value);
float velocity_PID_left(float present_value,int target_value);
float velocity_PID_right(float present_value,int target_value);
float ADC_Turn_PID_PWM(float present_value);
float CAM_Turn_PID_PWM(float present_value);
float velocity_PID_middle(float present_value,int target_value);
float gyro_PID(float present_value,int target_value);
int current_PID_left(float present_value,float target_value);
int current_PID_right(float present_value,float target_value);
void PID_Positional_calculation(PID_STRUCT*PID);
void PID_Incremental_calculation(PID_STRUCT*PID);
float PID_Positional(PID_STRUCT* PID,float present_value,float target_value);


#endif /* CODE_CONTROL_PID_H_ */
