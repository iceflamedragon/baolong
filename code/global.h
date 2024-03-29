/*
 * global.h
 *
 *  Created on: 2021年11月29日
 *      Author: fun_314
 */

#ifndef CODE_GLOBAL_H_
#define CODE_GLOBAL_H_
#include "zf_common_headfile.h"

#define MYCAR   1
typedef unsigned char         u8;                                               // 无符号  8 bits
typedef unsigned short        u16;                                              // 无符号 16 bits
typedef unsigned long         u32;

#define TIME_MS     1                   //interrupt_time

#define TIME_MS_1   1
#define _LIMIT(data, min, max) (((int)data) < (min) ? (min) : ((data) > (max) ? (max) : (data)))
#define _MIN(x, y) (x) < (y) ? (x) : (y)
#define abs_m(a,b) (a>=b ? a-b:b-a)
#define abs_f(a) (a>0?a:-a)
//#define PI 3.14159265358979f

//////////调参编码器的引脚
#define PRESS_PROT      P10_2
#define CW_PROT         P10_3
#define CCW_PROT        P10_1
///////////////调参KEY
#define KEY_UP_PROT     P10_6
#define KEY_PUSH_PROT   P10_5
#define KEY_DOWN_PROT   P02_0
////////////核心板上的LED
#define LED1                    (P20_9)
#define LED2                    (P20_8)

#pragma section all "cpu0_dsram"

typedef struct FLAG_STRUCT
{
    int CircCount;
    int outloop;
    int JuncChange;
    int JuncCount;
    int last_junccount;
    int JuncDirection;
    int MPU_ready_flag;
    int16 DetectMode;
    uint8 STRAIGHT_FLAG;
    uint8 slope_flag;
    int slopecount;
    uint8 out_jun_flag;
    int outloop2;
    int getspeedflag;
    int speed_flag;
    int accelerate_flag;
    int injuncflag;
} FLAG_STRUCT;
//摄像头标志

struct camcfg_s
{
    int wl;
    int wr;
    int wf;
    int forward_near;
    int forward_far;
    int manbri;
    int exptime;
    int autogain;
    int setedge;
    int pitch;
    int rotate_x;
    int rotate_y;
    int shift;
    int loop_flag;
};
/********************摄像头相关****************************/
struct caminfo_s
{


    int16 dir;
    int16 ddir;
    int16 zebra_count;
    int16 apriltag_count;
    int16 loop_process;
    int16 loop_flag;

    int16 estimation_dir;
    int16 delmax_dir;

};
//
//typedef struct mycar_STRUCT{
//
//        uint8 status;   //小车运行状态
//        uint32 RUNTIME; //小车运行时间
//        uint32 save_times;
//        uint32 savenum;
//        uint32 save_img_count;
//        uint32 senddata;
//        uint32 WAITTIME;//发车等待时间
//
//}mycar_STRUCT;

#pragma section all restore

#pragma section all "cpu0_dsram"
//模糊控制相关
struct fuzzy_params{
        int m_ke;
        int m_kec;
        int m_ku;
        int s_ke;
        int s_ku;
        int s_kud;
        int s_kp;
        int s_kd;
        int s_ki;
};

/********************输入数据****************************/
typedef struct indata_STRUCT
{

    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float angle_acc;
    float angle_gyo;
    float angle_last;
    int angle;
    int speed_BM;
    int T;

    int all_distance;
    int inloop_setspeed;

    float avg_gyr_x[10];
    float gyr_x;
    float jun_yaw;
    uint8 jun_left;
    uint8 jun_right;
    uint8 out_jun_flag;
    /********************移植后加入****************************/

        int d_dir;

        int battvolt12_6;
        int battvolt3_7;
        int16 Speed;


        int16 fps;

        // int angle;
        float YawAngle;
        float PitchAngle;

        /*camera*/
        int16 cam_valid_count;
        int16 cam_fps;
        int16 cam_dir;
        int16 cam_ddir;

        /*speed*/
        int speed;
        int setspeed;
        /**********************************/

        struct caminfo_s caminfo;

        float journey;
        float target;
        int16 DetectFlag;

        float distance;
        float slope_distance;
        float jun_distance;
        float distance_loop2;
        float out_slope_distance;
        float changelamjunc_dis;
        float yaw;
        float roll;
        float pitch;

        int gyro_adjust_count;
        float qualization_factor;
        float fuzzy_turning_angle;
        float fuzzy_balance_Kp;
        float ad_pid_out;
        int fuzzy_steer_angle;
//        float base_length;


}indata_STRUCT;


typedef struct outdata_STRUCT
{
    int Motor_PWM;
    int Steer_PWM;
    int Bala_PWM;
    float angle_Kalman;
    float angax;
    float expect_gyor;
    float inter_pwm;
    float position_pwm;

}outdata_STRUCT;


typedef struct dir_control_struct{
        uint8 mode;                                         // 0 右巡线模式 ，  1 左巡线模式 ， 3 寻中线模式
        uint32 lim_dir;
        uint16 far_lim;
        int Lambda_dir;                                     //控制转向模式变化参数
        int dLambda_dir;
        int wl;
        int wr;
        int pic_mode;
}dir_control_struct;

typedef struct PID_para_STRUCT
{
   int kp;
   int ki;
   int kd;
}PID_para_STRUCT;
typedef struct PID_STRUCT
{
    float kp_ratio;         //kp比例系数，用于调整数量级
    float ki_ratio;         //ki比例系数，用于调整数量级
    float kd_ratio;         //kd比例系数，用于调整数量级
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

//typedef struct chain_STRUCT
//{
//    int16 x;                    //起点坐标 X
//    int16 y;                    //起点坐标 Y
//    int16 len;                  //长度
//    int16 line[119];            //差码表
//    int16 t_sum[40];            //三点链码和
//    int diff;
//    int   curvature;            //两点差之和        对于只有一个弯的线与曲率成正比
//} chain_STRUCT;


/********************可调参数*******************************/
typedef struct setpara_STRUCT
{
       int Bal_kp;
       int Bal_kd;
       int Pos_kp;
       int Pos_ki;
       int outer_p;
       int outer_i;
       int inter_p;
       int inter_i;
       int inter_d;
       int middle;
       int trans;       //0无模式 1图传+数据   2图传高帧率   3VOFA+
       int pwm_set;
       int out_Jun_angle;
//移植后加入


       //各种PID
       struct PID_para_STRUCT com_turn_PID;  //常规情况下的方向环PID
       struct PID_para_STRUCT com_speed_PID;  //常规情况下的速度环PID
       struct PID_para_STRUCT loop_turn_PID;  //环内方向环PID
       struct PID_para_STRUCT big_loop_PID;   //大环方向环PID
       struct PID_para_STRUCT loop_speed_PID; //环内速度环PID
       struct PID_para_STRUCT adc_turn_PID;  //断路内电感方向环PID
       struct PID_para_STRUCT adc_speed_PID; //断路内速度环PID
       struct PID_para_STRUCT current_PID;   //电流环PID
       struct PID_para_STRUCT garage_turn_PID;   //车库中转向PID
       struct PID_para_STRUCT stop_PID;   //停止时速度环PID
       struct PID_para_STRUCT gyro_PID;  //角速度环PID
       struct PID_para_STRUCT obstacle_turn1_PID;
       struct PID_para_STRUCT obstacle_turn2_PID;
       struct PID_para_STRUCT obstacle_turn3_PID;
       struct PID_para_STRUCT speed_down_PID;//减速PID

       //不同情况下的目标速度
         float control_speed;//板卡目标速度


         int com_target_speed;  //常规情况下的目标速度
         int loop_target_speed;  //圆环中的目标速度
         int big_loop_speed;     //大环速度
         int broken_target_speed;  //断路中的目标速度
         int garage_target_speed; //车库中的目标速度
         int gyro_kp;             //角速度环kp值
         int speed_max;           //最大速度
         int speed_min;           //最小速度
         int speed_adjust_kp;     //速度调节比例
         //摄像头循迹相关
         int camwf;          //在逆透视平面中，摄像头到视野下沿的距离（单位为像素）（一般为负值）
         int camwl;          //在逆透视平面中，左轮到小车中心的距离  （单位为像素）
         int camwr;          //在逆透视平面中，右轮到小车中心的距离  （单位为像素）
         //上位机调试相关
         int swj_adjust;
         //出库转向err值
         int out_garage_err;
         ////入库转向err值
         int in_garage_err;
         int set_element[21];//元素设置
         int loop_data[4];   //圆环设置
         //电感增益
         int adc_L_gain;
         int adc_M_gain;
         int adc_R_gain;

 //移植后加入
        int mode;        // 起步 0直线 1左出库 2右出库   3推车
            int settime;
        /////////速度相关
            int CamAveSpeed;
            int SpeedKp;
            int SpeedKi;
            int SpeedKd;
            int diffK;
            int SpeedUP;
            int SpeedDOWN;
        /////////舵机相关
            int SteerMid;
            int MaxTurn;
            int CamKp;
            int CamKd;
        /////////属性、视觉相关
            //struct camcfg_s camcfg;
            int exp_ki;
        /////////车库相关
            int garage_outtime;
            int garage_outangle;
            int ZebraCount;
            int stop_kp;
            int stop_kd;
            int StopSpeed;
        /////////圆环相关
            int CamLoopKp;
            int CamLoopKd;
            int InLoopLine;              //入环前所补直线消失的阈值
            int WhenInLoop;
            int InLoopDis;
            int LoopThres;
            int OutLoopThres;
            int LoopWl;
            int LoopWr;
            int LoopOutAngle;
            int loop_out_distance;
            int big_loop_out_distance;
            //int
        //////////三岔路相关
            int StrThres; //直线阈值
            int TempRL; //三岔路临时变量，左1右2，后期由OpenMV发送
            int JuncProcess; //过三岔路的时间
            int WhenJuncout;   //三岔路停车时机
            int Jun_out_distance;
            int DetectTime;//识别时间
            int number;//识别时间
            int OutJunc2Lambda;
            int OutJunc2speed;
        /////////测试相关

            int TextLine;
            int TextRow; //图片上显示的调试的相关点
            // int TextDistance;

            int SlopeDown;
            int base_length;
            int set_rolling_angle;
            int balance_state;
            int rolling_angle_mid;
         //遥控调参
            int remote_control;
            int test_speed;
         //模糊控制参数
            //struct fuzzy_params fuzzy_s;
            int input_e_range;
            int input_ec_range;
            int input_u_range;
            int ad_kp;
            int ad_ki;
            int ad_kd;
            //struct dir_control_struct cardir;
            int jun_dLambda_dir;
            int loop_dLambda_dir;
            int jun_wl;
            int jun_wr;
            int steer_far;
            int jun_CamKp;
            int jun_CamKd;

            int delmax_dir;
            int Loop_deldir;
            int Junc_deldir;
            int O_steerfar;
            int Slopedistance;
            int junlength;

        //////图像显示模式
            int img_showmode;
            int K_dist2anglerate; //将距离引入到方向环的系数

            int out_garge;
            int sureout_garge;
            int in_gargethres;
        /////////速度相关
            int power_set;
            int SpeedMAX;
            int SpeedMIN;
            int SpeedSet;
            int speed_start;
        /////////圆环相关
            int InLoopRow;

            int distanceThres_in_loop;//入环积分阈值
            int distanceThres_out_loop;//出环积分阈值
        //////////三岔路相关
            int WhenJuncStop;   //三岔路停车时机
            int JuncSpeed;   //三岔路停车时机
        /////////二维码相关
            int AprilCount;
            int AprilStop;
            int AprilSpeed ;
            int AprilTime;
            int BackTime;
            int LaserTime;
        /////////测试相关
            int testL;
            int testR;

            float PWMMAX;
            float PWMMIN;

            int T_outthres;

            //断路相关
            int ad_adjust;
            int L_threshold;
            int H_threshold;
            int distanceThres_before_broken_circuit;//进断路前积分阈值
            int broken_circuit_slow_distane;     //断路减速积分
            int distanceThres_in_broken_circuit;//进断路积分阈值
            int distanceThres_out_broken_circuit;//出断路积分阈值

            //两核同步变量  0：不同步  1：同步
            int core_synchronization;

            //二值化阈值下限
            int Low_threshold;
            int  threshold_max;
            int threshold_min;
            //车库相关
            int in_garage_distance;//入库前路程积分
            int in_garage_angle;   //入库角度积分
            int out_garage_distance;//出库前路程积分
            int out_garage_angle;//出库前角度积分
            int garage_stop_distance;//停止前路程积分
            int start_mode;//发车模式 0：直行发车不入库 1左出库左入库 2右出库右入库 3:直行发车且左入库 4直行发车且右入库
            int garage_dis;
            int garage_angle;
            int garage_err;
            int obstacle_angle1;
            int obstacle_angle2;
            int garage_begin_distance;
            //避障相关
            int obstacle_dir;//避障方向选取（0左1右）
            int obstacle_dis;  //避障前路程积分距离
            int obstacle_dis2; //避障第一段转向后路程积分
            int obstacle_dis3; //避障第一段转向后路程积分
            int obstacle_dis4; //转向前积分
            int obstacle_detect_distance;//探测距离
            int obstacle_speed;
            //发车相关
            int begin_pwm;
            int begin_time;
            int stop_time;
            int adc_protect;
            int loop_angle_out;
            int big_loop_out;
            //参表隔行
            int steer_pwm;
            int far_line;
            int fan_speed;

            int steer_adjust;//舵机调试



} setpara_STRUCT;

////////////////    myinclude
#include "mycar.h"
#include "base.h"
#include "GUI.h"
#include "pid.h"
#include "integral.h"
#include "timer.h"
#include <edgeboard_uart.h>
#include   "vofa.h"
#include "adjust.h"
#include "display.h"


extern struct mycar_STRUCT mycar;


#pragma section all restore


#endif /* CODE_GLOBAL_H_ */
