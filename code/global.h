/*
 * global.h
 *
 *  Created on: 2021��11��29��
 *      Author: fun_314
 */

#ifndef CODE_GLOBAL_H_
#define CODE_GLOBAL_H_
#include "zf_common_headfile.h"

#define MYCAR   1
typedef unsigned char         u8;                                               // �޷���  8 bits
typedef unsigned short        u16;                                              // �޷��� 16 bits
typedef unsigned long         u32;

#define TIME_MS     1                   //interrupt_time

#define TIME_MS_1   1
#define _LIMIT(data, min, max) (((int)data) < (min) ? (min) : ((data) > (max) ? (max) : (data)))
#define _MIN(x, y) (x) < (y) ? (x) : (y)
#define abs_m(a,b) (a>=b ? a-b:b-a)
#define abs_f(a) (a>0?a:-a)
//#define PI 3.14159265358979f

//////////���α�����������
#define PRESS_PROT      P10_2
#define CW_PROT         P10_3
#define CCW_PROT        P10_1
///////////////����KEY
#define KEY_UP_PROT     P10_6
#define KEY_PUSH_PROT   P10_5
#define KEY_DOWN_PROT   P02_0
////////////���İ��ϵ�LED
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
//����ͷ��־

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
/********************����ͷ���****************************/
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
//        uint8 status;   //С������״̬
//        uint32 RUNTIME; //С������ʱ��
//        uint32 save_times;
//        uint32 savenum;
//        uint32 save_img_count;
//        uint32 senddata;
//        uint32 WAITTIME;//�����ȴ�ʱ��
//
//}mycar_STRUCT;

#pragma section all restore

#pragma section all "cpu0_dsram"
//ģ���������
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

/********************��������****************************/
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
    /********************��ֲ�����****************************/

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
        uint8 mode;                                         // 0 ��Ѳ��ģʽ ��  1 ��Ѳ��ģʽ �� 3 Ѱ����ģʽ
        uint32 lim_dir;
        uint16 far_lim;
        int Lambda_dir;                                     //����ת��ģʽ�仯����
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
    float kp_ratio;         //kp����ϵ�������ڵ���������
    float ki_ratio;         //ki����ϵ�������ڵ���������
    float kd_ratio;         //kd����ϵ�������ڵ���������
    float present_value;    //����ֵ
    float value_err[3];     //ƫ��
    float PID_output;       //PID�����
    float PID_output_last;  //�ϴ�PID�����
    float d_output;         //�������
    float integral_Limit;   //�����޷�
    float total_limit;      //���޷�
    float PID_out_P;        //P����
    float PID_out_I;        //I����
    float PID_out_D;        //D����
    float target_value;     //Ŀ�����
    float d_limit;          //�޷�
    PID_para_STRUCT* PID_para;

}PID_STRUCT;

//typedef struct chain_STRUCT
//{
//    int16 x;                    //������� X
//    int16 y;                    //������� Y
//    int16 len;                  //����
//    int16 line[119];            //�����
//    int16 t_sum[40];            //���������
//    int diff;
//    int   curvature;            //�����֮��        ����ֻ��һ������������ʳ�����
//} chain_STRUCT;


/********************�ɵ�����*******************************/
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
       int trans;       //0��ģʽ 1ͼ��+����   2ͼ����֡��   3VOFA+
       int pwm_set;
       int out_Jun_angle;
//��ֲ�����


       //����PID
       struct PID_para_STRUCT com_turn_PID;  //��������µķ���PID
       struct PID_para_STRUCT com_speed_PID;  //��������µ��ٶȻ�PID
       struct PID_para_STRUCT loop_turn_PID;  //���ڷ���PID
       struct PID_para_STRUCT big_loop_PID;   //�󻷷���PID
       struct PID_para_STRUCT loop_speed_PID; //�����ٶȻ�PID
       struct PID_para_STRUCT adc_turn_PID;  //��·�ڵ�з���PID
       struct PID_para_STRUCT adc_speed_PID; //��·���ٶȻ�PID
       struct PID_para_STRUCT current_PID;   //������PID
       struct PID_para_STRUCT garage_turn_PID;   //������ת��PID
       struct PID_para_STRUCT stop_PID;   //ֹͣʱ�ٶȻ�PID
       struct PID_para_STRUCT gyro_PID;  //���ٶȻ�PID
       struct PID_para_STRUCT obstacle_turn1_PID;
       struct PID_para_STRUCT obstacle_turn2_PID;
       struct PID_para_STRUCT obstacle_turn3_PID;
       struct PID_para_STRUCT speed_down_PID;//����PID

       //��ͬ����µ�Ŀ���ٶ�
         float control_speed;//�忨Ŀ���ٶ�


         int com_target_speed;  //��������µ�Ŀ���ٶ�
         int loop_target_speed;  //Բ���е�Ŀ���ٶ�
         int big_loop_speed;     //���ٶ�
         int broken_target_speed;  //��·�е�Ŀ���ٶ�
         int garage_target_speed; //�����е�Ŀ���ٶ�
         int gyro_kp;             //���ٶȻ�kpֵ
         int speed_max;           //����ٶ�
         int speed_min;           //��С�ٶ�
         int speed_adjust_kp;     //�ٶȵ��ڱ���
         //����ͷѭ�����
         int camwf;          //����͸��ƽ���У�����ͷ����Ұ���صľ��루��λΪ���أ���һ��Ϊ��ֵ��
         int camwl;          //����͸��ƽ���У����ֵ�С�����ĵľ���  ����λΪ���أ�
         int camwr;          //����͸��ƽ���У����ֵ�С�����ĵľ���  ����λΪ���أ�
         //��λ���������
         int swj_adjust;
         //����ת��errֵ
         int out_garage_err;
         ////���ת��errֵ
         int in_garage_err;
         int set_element[21];//Ԫ������
         int loop_data[4];   //Բ������
         //�������
         int adc_L_gain;
         int adc_M_gain;
         int adc_R_gain;

 //��ֲ�����
        int mode;        // �� 0ֱ�� 1����� 2�ҳ���   3�Ƴ�
            int settime;
        /////////�ٶ����
            int CamAveSpeed;
            int SpeedKp;
            int SpeedKi;
            int SpeedKd;
            int diffK;
            int SpeedUP;
            int SpeedDOWN;
        /////////������
            int SteerMid;
            int MaxTurn;
            int CamKp;
            int CamKd;
        /////////���ԡ��Ӿ����
            //struct camcfg_s camcfg;
            int exp_ki;
        /////////�������
            int garage_outtime;
            int garage_outangle;
            int ZebraCount;
            int stop_kp;
            int stop_kd;
            int StopSpeed;
        /////////Բ�����
            int CamLoopKp;
            int CamLoopKd;
            int InLoopLine;              //�뻷ǰ����ֱ����ʧ����ֵ
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
        //////////����·���
            int StrThres; //ֱ����ֵ
            int TempRL; //����·��ʱ��������1��2��������OpenMV����
            int JuncProcess; //������·��ʱ��
            int WhenJuncout;   //����·ͣ��ʱ��
            int Jun_out_distance;
            int DetectTime;//ʶ��ʱ��
            int number;//ʶ��ʱ��
            int OutJunc2Lambda;
            int OutJunc2speed;
        /////////�������

            int TextLine;
            int TextRow; //ͼƬ����ʾ�ĵ��Ե���ص�
            // int TextDistance;

            int SlopeDown;
            int base_length;
            int set_rolling_angle;
            int balance_state;
            int rolling_angle_mid;
         //ң�ص���
            int remote_control;
            int test_speed;
         //ģ�����Ʋ���
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

        //////ͼ����ʾģʽ
            int img_showmode;
            int K_dist2anglerate; //���������뵽���򻷵�ϵ��

            int out_garge;
            int sureout_garge;
            int in_gargethres;
        /////////�ٶ����
            int power_set;
            int SpeedMAX;
            int SpeedMIN;
            int SpeedSet;
            int speed_start;
        /////////Բ�����
            int InLoopRow;

            int distanceThres_in_loop;//�뻷������ֵ
            int distanceThres_out_loop;//����������ֵ
        //////////����·���
            int WhenJuncStop;   //����·ͣ��ʱ��
            int JuncSpeed;   //����·ͣ��ʱ��
        /////////��ά�����
            int AprilCount;
            int AprilStop;
            int AprilSpeed ;
            int AprilTime;
            int BackTime;
            int LaserTime;
        /////////�������
            int testL;
            int testR;

            float PWMMAX;
            float PWMMIN;

            int T_outthres;

            //��·���
            int ad_adjust;
            int L_threshold;
            int H_threshold;
            int distanceThres_before_broken_circuit;//����·ǰ������ֵ
            int broken_circuit_slow_distane;     //��·���ٻ���
            int distanceThres_in_broken_circuit;//����·������ֵ
            int distanceThres_out_broken_circuit;//����·������ֵ

            //����ͬ������  0����ͬ��  1��ͬ��
            int core_synchronization;

            //��ֵ����ֵ����
            int Low_threshold;
            int  threshold_max;
            int threshold_min;
            //�������
            int in_garage_distance;//���ǰ·�̻���
            int in_garage_angle;   //���ǶȻ���
            int out_garage_distance;//����ǰ·�̻���
            int out_garage_angle;//����ǰ�ǶȻ���
            int garage_stop_distance;//ֹͣǰ·�̻���
            int start_mode;//����ģʽ 0��ֱ�з�������� 1���������� 2�ҳ�������� 3:ֱ�з���������� 4ֱ�з����������
            int garage_dis;
            int garage_angle;
            int garage_err;
            int obstacle_angle1;
            int obstacle_angle2;
            int garage_begin_distance;
            //�������
            int obstacle_dir;//���Ϸ���ѡȡ��0��1�ң�
            int obstacle_dis;  //����ǰ·�̻��־���
            int obstacle_dis2; //���ϵ�һ��ת���·�̻���
            int obstacle_dis3; //���ϵ�һ��ת���·�̻���
            int obstacle_dis4; //ת��ǰ����
            int obstacle_detect_distance;//̽�����
            int obstacle_speed;
            //�������
            int begin_pwm;
            int begin_time;
            int stop_time;
            int adc_protect;
            int loop_angle_out;
            int big_loop_out;
            //�α����
            int steer_pwm;
            int far_line;
            int fan_speed;

            int steer_adjust;//�������



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
