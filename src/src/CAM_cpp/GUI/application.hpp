/*
 * application.h
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */

#ifndef CODE_GUI_APPLICATION_H_
#define CODE_GUI_APPLICATION_H_
#include "../global.hpp"
typedef struct setpara_STRUCT
{
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
        struct PID_para_STRUCT black_obstacle_turn_PID;//黑色路障方向环PID
        struct PID_para_STRUCT speed_down_PID;//减速PID

        struct ADRC_para_STRUCT com_turn_ADRC;
        //不同情况下的目标速度
        float com_target_speed;  //常规情况下的目标速度
        float loop_target_speed;  //圆环中的目标速度
        float big_loop_speed;     //大环速度
        float broken_target_speed;  //断路中的目标速度
        float garage_target_speed; //车库中的目标速度
        float bla_obs_speed;//路障中的目标速度
        float slope_speed;//坡道中的目标速度
        float zebra_speed;//斑马线停车速度
        int gyro_kp;             //角速度环kp值
        float speed_max;           //最大速度
        float speed_min;           //最小速度
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
        //各种PID
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
           float speed_add;
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
           int zebra_distance;
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
           float obstacle_speed;
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
           int steer_buchang;//舵机中值补偿
           int differ_ratio;//差速比例
           int fuzzy_kp;//模糊pid增益
           int fuzzy_kd;//模糊pid增益
           int fuzzy_k;

           int fan_start;//风扇初始速度

           int track_diff_cutoff;//跟踪差值截止频率
           int SteerKpchange;//舵机变化系数
           float speed_kf;//速度前馈系数
           float cross_speed;
           int cross_open_flag;//十字检测启停标志位
           int bla_obs_open_flag;//黑色路障启停标志位

           int bla_obs_begin_time;
           int slope_begin_time;
           int zebra_begin_time;
           int stop_over_count;
           int USART_flag;
           int zebra_line_count;
           int track_open_flag;
           int fan_differ_ratio;
           int pwm_adjust;
} setpara_STRUCT;

struct paralist_s
{
  int* para;        //参数存放的地址
  char label[20];   //参数的名称
  unsigned short precision;//参数的调整精度
};
void GUI_Init();
void GUI_scan();
void GUI_Display();
void init_setpara();
extern struct paralist_s paralist[100];
extern struct setpara_STRUCT setpara;
extern struct paralist_s paralist2[100];

#endif /* CODE_GUI_APPLICATION_H_ */
