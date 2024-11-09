/*
 * pid.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "pid.hpp"
PID_STRUCT  ADC_Turn={0},     //电磁转向PID
            gyro={0},        //角速度PID
            CAM_Turn={0},    //摄像头转向PID
            Speed_left={0},  //左轮速度PID
            Speed_right={0}, //右轮速度PID
            Speed_middle={0},//速度中值PID
            current_left={0},//左轮电流环PID
            current_right={0};//右轮电流环PID
void PID_init()
{
    ADC_Turn.kp_ratio=0.00001;
    ADC_Turn.ki_ratio=0.00001;
    ADC_Turn.kd_ratio=0.00001;
    ADC_Turn.integral_Limit=1000;
    ADC_Turn.total_limit=1000;
    ADC_Turn.d_limit=100;
    ADC_Turn.PID_para=&setpara.adc_turn_PID;

    CAM_Turn.kp_ratio=0.1;
    CAM_Turn.ki_ratio=0.001;
    CAM_Turn.kd_ratio=1;
    CAM_Turn.integral_Limit=150;
    CAM_Turn.total_limit=490+setpara.steer_adjust;
    CAM_Turn.PID_para=&CAM_FUZZY_PID;
    CAM_Turn.d_limit=0;

    Speed_left.kp_ratio=1;//纯速度环1，电流环串速度环0.01
    Speed_left.ki_ratio=0.1;//增量式0.1,位置式0.01//纯速度环0.1，电流环串速度环0.0001
    Speed_left.kd_ratio=0.0;//纯速度环0，电流环串速度环0.01
    Speed_left.kf_ratio=1;
    Speed_left.integral_Limit=9000;//纯速度环9000
    Speed_left.total_limit=9900;//纯速度环9900
//    Speed_left.d_limit=50;
    Speed_left.PID_para=&setpara.com_speed_PID;


    Speed_right.kp_ratio=1;
    Speed_right.ki_ratio=0.1;
    Speed_right.kd_ratio=0.0;
    Speed_left.kf_ratio=1;
    Speed_right.integral_Limit=9000;
    Speed_right.total_limit=9900;
//    Speed_right.d_limit=50;
    Speed_right.PID_para=&setpara.com_speed_PID;

    current_left.kp_ratio=100;
    current_left.ki_ratio=1;
    current_left.kd_ratio=0.1;
    current_left.integral_Limit=9000;
    current_left.total_limit=9900;
    current_left.PID_para=&setpara.current_PID;

    current_right.kp_ratio=100;
    current_right.ki_ratio=1;
    current_right.kd_ratio=0.1;
    current_right.integral_Limit=9000;
    current_right.total_limit=9900;
    current_right.PID_para=&setpara.current_PID;

    gyro.kp_ratio=0.1;
    gyro.ki_ratio=0.001;
    gyro.kd_ratio=0.1;
    gyro.integral_Limit=3000;
    gyro.total_limit=6000;
    gyro.PID_para=&setpara.gyro_PID;
    gyro.d_limit=600;
}
void PID_struct_init(PID_STRUCT *PID,float kp_ratio,float ki_ratio,float kd_ratio,float integral_Limit,PID_para_STRUCT*PID_para)
{
    PID->PID_para=PID_para;
    PID->kp_ratio=kp_ratio;
    PID->ki_ratio=ki_ratio;
    PID->kd_ratio=kd_ratio;
    PID->integral_Limit=integral_Limit;
}
//改变PID参数组
void change_pid_para(PID_STRUCT *PID,PID_para_STRUCT*PID_para)
{
    PID->PID_para=PID_para;
}
//清除积分项
void clear_integral(PID_STRUCT *PID)
{
    PID->PID_out_I=0;
    PID->PID_output=0;
}


//位置式PID
float PID_Positional(PID_STRUCT* PID,float present_value,float target_value)
{
    PID->target_value=target_value;                      //获取目标变量值
    PID->present_value=present_value;                    //获取当前变量值
    PID->value_err[0]=PID->target_value- PID->present_value; //更新本次偏差

    PID->PID_out_P=(PID->PID_para->kp)*(PID->kp_ratio)*PID->value_err[0];//计算PID中的KP分量
    PID->PID_out_I+=(PID->PID_para->ki)*(PID->ki_ratio)*(PID->value_err[0]);//计算PID中的KI分量
    PID->PID_out_D=(PID->PID_para->kd)*(PID->kd_ratio)*(PID->value_err[0]-PID->value_err[1]);//计算PID中的KD分量
    if(PID->PID_out_I>PID->integral_Limit)PID->PID_out_I=PID->integral_Limit;//积分限幅
    else if(PID->PID_out_I<-PID->integral_Limit)PID->PID_out_I=-PID->integral_Limit;

    PID->PID_output=PID->PID_out_P+PID->PID_out_I+PID->PID_out_D;

    PID->value_err[1]=PID->value_err[0];                 //更新上次偏差

    if(PID->d_limit!=0) //变化量限幅
    {
        if(PID->PID_output-PID->PID_output_last>PID->d_limit)
            PID->PID_output=PID->PID_output_last+PID->d_limit;
        else if(PID->PID_output-PID->PID_output_last<-PID->d_limit)
            PID->PID_output=PID->PID_output_last-PID->d_limit;
        PID->PID_output_last=PID->PID_output;
    }
    if(PID->PID_output>PID->total_limit)PID->PID_output=PID->total_limit;
    else if(PID->PID_output<-PID->total_limit)PID->PID_output=-PID->total_limit;
    return PID->PID_output;
}


//增量式PID
float PID_Incremental(PID_STRUCT* PID,float present_value,float target_value)
{
    PID->target_value=target_value;                      //获取目标变量值
    PID->present_value=present_value;                    //获取当前变量值
    PID->value_err[0]=PID->target_value- PID->present_value; //更新本次偏差
    //if(fabs((PID->value_err[0]) < 2)&&(PID==&Speed_left||PID==&Speed_right)) PID->value_err[0] = 0;//pid死区
    PID->PID_out_P=(PID->PID_para->kp)*(PID->kp_ratio)*(PID->value_err[0]-PID->value_err[1]);//计算PID中的KP分量
    PID->PID_out_I=(PID->PID_para->ki)*(PID->ki_ratio)*(PID->value_err[0]);//计算PID中的KI分量
    PID->PID_out_D=(PID->PID_para->kd)*(PID->kd_ratio)*(PID->value_err[0]-2*PID->value_err[1]+PID->value_err[2]);//计算PID中的KD分量
//    if(PID->PID_out_I>PID->integral_Limit)PID->PID_out_I=PID->integral_Limit;//积分限幅
//    else if(PID->PID_out_I<-PID->integral_Limit)PID->PID_out_I=-PID->integral_Limit;

    PID->PID_output+=PID->PID_out_P+PID->PID_out_I+PID->PID_out_D;

    PID->value_err[2]=PID->value_err[1];
    PID->value_err[1]=PID->value_err[0];                 //更新上次偏差

    if(PID->d_limit!=0) //变化量限幅
    {
        if(PID->PID_output-PID->PID_output_last>PID->d_limit)
            PID->PID_output=PID->PID_output_last+PID->d_limit;
        else if(PID->PID_output-PID->PID_output_last<-PID->d_limit)
            PID->PID_output=PID->PID_output_last-PID->d_limit;
        PID->PID_output_last=PID->PID_output;
    }
    if(PID->PID_output>PID->total_limit)PID->PID_output=PID->total_limit;
    else if(PID->PID_output<-PID->total_limit)PID->PID_output=-PID->total_limit;
    return PID->PID_output;
}

