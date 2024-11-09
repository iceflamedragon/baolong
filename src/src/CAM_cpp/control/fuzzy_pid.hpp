/*
 * fuzzy_pid.h
 *
 *  Created on: 2024��3��25��
 *      Author: 14250
 */

#ifndef CODE_CONTROL_FUZZY_PID_H_
#define CODE_CONTROL_FUZZY_PID_H_
#include "../global.hpp"
#define PB 3
#define PM 2
#define PS 1
#define ZO 0
#define NS -1
#define NM -2
#define NB -3
extern PID_para_STRUCT CAM_FUZZY_PID;
void updata_fuzzy_pid(float err,float d_err);
void updata_fuzzy_speed(float err,float k);
float fuzzy_speed2(float d_k);
void speed_ctrl_cal();


#endif /* CODE_CONTROL_FUZZY_PID_H_ */
