/*
 * application.h
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */

#ifndef CODE_GUI_APPLICATION_H_
#define CODE_GUI_APPLICATION_H_
#include "GUI.h"
//typedef struct setpara_STRUCT
//{
//        //����PID
//        struct PID_para_STRUCT com_turn_PID;  //��������µķ���PID
//        struct PID_para_STRUCT com_speed_PID;  //��������µ��ٶȻ�PID
//        struct PID_para_STRUCT loop_turn_PID;  //���ڷ���PID
//        struct PID_para_STRUCT big_loop_PID;   //�󻷷���PID
//        struct PID_para_STRUCT loop_speed_PID; //�����ٶȻ�PID
//        struct PID_para_STRUCT adc_turn_PID;  //��·�ڵ�з���PID
//        struct PID_para_STRUCT adc_speed_PID; //��·���ٶȻ�PID
//        struct PID_para_STRUCT current_PID;   //������PID
//        struct PID_para_STRUCT garage_turn_PID;   //������ת��PID
//        struct PID_para_STRUCT stop_PID;   //ֹͣʱ�ٶȻ�PID
//        struct PID_para_STRUCT gyro_PID;  //���ٶȻ�PID
//        struct PID_para_STRUCT obstacle_turn1_PID;
//        struct PID_para_STRUCT obstacle_turn2_PID;
//        struct PID_para_STRUCT obstacle_turn3_PID;
//        struct PID_para_STRUCT speed_down_PID;//����PID
//        //��ͬ����µ�Ŀ���ٶ�
//        int com_target_speed;  //��������µ�Ŀ���ٶ�
//        int loop_target_speed;  //Բ���е�Ŀ���ٶ�
//        int big_loop_speed;     //���ٶ�
//        int broken_target_speed;  //��·�е�Ŀ���ٶ�
//        int garage_target_speed; //�����е�Ŀ���ٶ�
//        int gyro_kp;             //���ٶȻ�kpֵ
//        int speed_max;           //����ٶ�
//        int speed_min;           //��С�ٶ�
//        int speed_adjust_kp;     //�ٶȵ��ڱ���
//        //����ͷѭ�����
//        int camwf;          //����͸��ƽ���У�����ͷ����Ұ���صľ��루��λΪ���أ���һ��Ϊ��ֵ��
//        int camwl;          //����͸��ƽ���У����ֵ�С�����ĵľ���  ����λΪ���أ�
//        int camwr;          //����͸��ƽ���У����ֵ�С�����ĵľ���  ����λΪ���أ�
//        //��λ���������
//        int swj_adjust;
//        //����ת��errֵ
//        int out_garage_err;
//        ////���ת��errֵ
//        int in_garage_err;
//        int set_element[21];//Ԫ������
//        int loop_data[4];   //Բ������
//        //�������
//        int adc_L_gain;
//        int adc_M_gain;
//        int adc_R_gain;
//        //����PID
//       int Bal_kp;
//       int Bal_kd;
//       int Pos_kp;
//       int Pos_ki;
//       int outer_p;
//       int outer_i;
//       int inter_p;
//       int inter_i;
//       int inter_d;
//       int middle;
//       int trans;       //0��ģʽ 1ͼ��+����   2ͼ����֡��   3VOFA+
//       int pwm_set;
//       int out_Jun_angle;
////��ֲ�����
//       int mode;        // �� 0ֱ�� 1����� 2�ҳ���   3�Ƴ�
//           int settime;
//       /////////�ٶ����
//           int CamAveSpeed;
//           int SpeedKp;
//           int SpeedKi;
//           int SpeedKd;
//           int diffK;
//           int SpeedUP;
//           int SpeedDOWN;
//       /////////������
//           int SteerMid;
//           int MaxTurn;
//           int CamKp;
//           int CamKd;
//       /////////���ԡ��Ӿ����
//           //struct camcfg_s camcfg;
//           int exp_ki;
//       /////////�������
//           int garage_outtime;
//           int garage_outangle;
//           int ZebraCount;
//           int stop_kp;
//           int stop_kd;
//           int StopSpeed;
//       /////////Բ�����
//           int CamLoopKp;
//           int CamLoopKd;
//           int InLoopLine;              //�뻷ǰ����ֱ����ʧ����ֵ
//           int WhenInLoop;
//           int InLoopDis;
//           int LoopThres;
//           int OutLoopThres;
//           int LoopWl;
//           int LoopWr;
//           int LoopOutAngle;
//           int loop_out_distance;
//           int big_loop_out_distance;
//           //int
//       //////////����·���
//           int StrThres; //ֱ����ֵ
//           int TempRL; //����·��ʱ��������1��2��������OpenMV����
//           int JuncProcess; //������·��ʱ��
//           int WhenJuncout;   //����·ͣ��ʱ��
//           int Jun_out_distance;
//           int DetectTime;//ʶ��ʱ��
//           int number;//ʶ��ʱ��
//           int OutJunc2Lambda;
//           int OutJunc2speed;
//       /////////�������
//           int TextLine;
//           int TextRow; //ͼƬ����ʾ�ĵ��Ե���ص�
//           // int TextDistance;
//
//           int SlopeDown;
//           int base_length;
//           int set_rolling_angle;
//           int balance_state;
//           int rolling_angle_mid;
//        //ң�ص���
//           int remote_control;
//           int test_speed;
//        //ģ�����Ʋ���
//           //struct fuzzy_params fuzzy_s;
//           int input_e_range;
//           int input_ec_range;
//           int input_u_range;
//           int ad_kp;
//           int ad_ki;
//           int ad_kd;
//           //struct dir_control_struct cardir;
//           int jun_dLambda_dir;
//           int loop_dLambda_dir;
//           int jun_wl;
//           int jun_wr;
//           int steer_far;
//           int jun_CamKp;
//           int jun_CamKd;
//
//           int delmax_dir;
//           int Loop_deldir;
//           int Junc_deldir;
//           int O_steerfar;
//           int Slopedistance;
//           int junlength;
//
//       //////ͼ����ʾģʽ
//           int img_showmode;
//           int K_dist2anglerate; //���������뵽���򻷵�ϵ��
//
//           int out_garge;
//           int sureout_garge;
//           int in_gargethres;
//       /////////�ٶ����
//           int power_set;
//           int SpeedMAX;
//           int SpeedMIN;
//           int SpeedSet;
//           int speed_start;
//       /////////Բ�����
//           int InLoopRow;
//
//           int distanceThres_in_loop;//�뻷������ֵ
//           int distanceThres_out_loop;//����������ֵ
//       //////////����·���
//           int WhenJuncStop;   //����·ͣ��ʱ��
//           int JuncSpeed;   //����·ͣ��ʱ��
//       /////////��ά�����
//           int AprilCount;
//           int AprilStop;
//           int AprilSpeed ;
//           int AprilTime;
//           int BackTime;
//           int LaserTime;
//       /////////�������
//           int testL;
//           int testR;
//
//           float PWMMAX;
//           float PWMMIN;
//
//           int T_outthres;
//
//           //��·���
//           int ad_adjust;
//           int L_threshold;
//           int H_threshold;
//           int distanceThres_before_broken_circuit;//����·ǰ������ֵ
//           int broken_circuit_slow_distane;     //��·���ٻ���
//           int distanceThres_in_broken_circuit;//����·������ֵ
//           int distanceThres_out_broken_circuit;//����·������ֵ
//
//           //����ͬ������  0����ͬ��  1��ͬ��
//           int core_synchronization;
//
//           //��ֵ����ֵ����
//           int Low_threshold;
//           int  threshold_max;
//           int threshold_min;
//           //�������
//           int in_garage_distance;//���ǰ·�̻���
//           int in_garage_angle;   //���ǶȻ���
//           int out_garage_distance;//����ǰ·�̻���
//           int out_garage_angle;//����ǰ�ǶȻ���
//           int garage_stop_distance;//ֹͣǰ·�̻���
//           int start_mode;//����ģʽ 0��ֱ�з�������� 1���������� 2�ҳ�������� 3:ֱ�з���������� 4ֱ�з����������
//           int garage_dis;
//           int garage_angle;
//           int garage_err;
//           int obstacle_angle1;
//           int obstacle_angle2;
//           int garage_begin_distance;
//           //�������
//           int obstacle_dir;//���Ϸ���ѡȡ��0��1�ң�
//           int obstacle_dis;  //����ǰ·�̻��־���
//           int obstacle_dis2; //���ϵ�һ��ת���·�̻���
//           int obstacle_dis3; //���ϵ�һ��ת���·�̻���
//           int obstacle_dis4; //ת��ǰ����
//           int obstacle_detect_distance;//̽�����
//           int obstacle_speed;
//           //�������
//           int begin_pwm;
//           int begin_time;
//           int stop_time;
//           int adc_protect;
//           int loop_angle_out;
//           int big_loop_out;
//           //�α����
//           int steer_pwm;
//           int far_line;
//           int fan_speed;
//
//           int steer_adjust;//�������
//
//} setpara_STRUCT;

struct paralist_s
{
  int* para;        //������ŵĵ�ַ
  char label[20];   //����������
  unsigned short precision;//�����ĵ�������
};
void GUI_Init();
void GUI_scan();
void GUI_Display();
void init_setpara();
extern struct paralist_s paralist[100];
extern struct setpara_STRUCT setpara;
extern struct paralist_s paralist2[100];

#endif /* CODE_GUI_APPLICATION_H_ */
