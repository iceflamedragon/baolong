/*
 * mycar.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CONTROL_MYCAR_H_
#define CODE_CONTROL_MYCAR_H_
#include "global.h"


//小车状态标志位
typedef struct mycar_STRUCT{
        uint8 status;   //小车运行状态
        uint32 RUNTIME; //小车运动时间(单位为1ms)
        uint32 TIME;    //小车运行时间(单位为1ms)
        uint32 senddata;
/*
        u32 save_times;
        u32 savenum;
        u32 save_img_count;

        u32 WAITTIME;//发车等待时间
        uint8 out_garage;//出车库标记，发车时刷新
        uint8 in_garage;//入库标记
*/
        uint8 car_running; //小车发车状态 1：发车  0：停止
        uint8 car_stop;    //刹车标记位，置1时刹车
        float present_speed;  ////当前读取的脉冲数速度
        float present_speed2;
        float present_physical_speed;//当前物理速度

        float   left_speed,         //左侧电机读取的速度(本质上是脉冲数)
                right_speed,        //右侧电机读取的速度(本质上是脉冲数)
//


                original_err,      //原始偏差值
                target_gyro,   //目标角速度
                gyro_err,     //角速度偏差
                Speed_err,  //方向环输出的转向的速度差
                current_left,//左速度环PID输出的电流值
                current_right;//右速度环PID输出的电流值
        float target_left_speed;
        float target_right_speed;

        int16   speed,          //电机读取的速度(本质上是脉冲数)

                left_pwm_set,      //左侧电机要设定的PWM
                right_pwm_set,      //右侧电机要设定的PWM

                pwm_set,        //只有一个电机设定的pwm

                speed_PWM;          //由速度环PID输出的值

        int16   speed_Middle_PWM, //速度环输出的PWM中值
                Turn_PWM;         //方向环输出的PWM差值
        int16 steer_pwm;
        float target_speed;  //小车当前目标速度，物理
        uint8 CircCount;     //小车当前圈数
        uint8 speed_ctrl;     //设置当前小车速度(偏差越大，速度越慢)
        uint8 tracking_mode;  //循迹方式，设1时用电感循迹

        float speed_differ;  //左右轮差速（百分比）

}mycar_STRUCT;

extern mycar_STRUCT mycar;
void motor_control(void);
void dir_control(void);
void dir_pid_adjust();
void speed_pid_adjust();
void common_running();
void running_protect();
void car_begin();
void car_stop();
void dir_control();
void speed_ctrl();
float speed_differ_set(int16 steer_pwm);
void set_speed(uint16 speed);
#endif /* CODE_CONTROL_MYCAR_H_ */
