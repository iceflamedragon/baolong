/*
 * pid.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CONTROL_PID_h_
#define CODE_CONTROL_PID_h_
#ifdef __cplusplus
extern "C" {
#endif
#include "../global.h"

//extern struct PID_para_STRUCT;
typedef struct PID_STRUCT
{
    float kp_ratio;         //kp比例系数，用于调整数量级
    float ki_ratio;         //ki比例系数，用于调整数量级
    float kd_ratio;         //kd比例系数，用于调整数量级
    float kf_ratio;         //kf比例系数，用于调整数量级
    float present_value;    //变量值
    float value_err[3];     //偏差
    float PID_output;       //PID总输出
    float PID_output_last;  //上次PID总输出
    float d_output;         //输出增量
    float integral_Limit;   //积分限幅
    float total_limit;      //总限幅
    float PID_out_P;        //P分量
    float PID_out_I;        //I分量
    float PID_out_D;        //D分量
    float target_value;     //目标变量
    float d_limit;          //限幅

    PID_para_STRUCT* PID_para;

}PID_STRUCT;
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

float PID_Positional(PID_STRUCT* PID,float present_value,float target_value);
float PID_Incremental(PID_STRUCT* PID,float present_value,float target_value);
#ifdef __cplusplus
}
#endif 
#endif /* CODE_CONTROL_PID_H_ */
