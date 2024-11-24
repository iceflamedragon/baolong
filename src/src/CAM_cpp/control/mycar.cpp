/*
 * mycar.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "mycar.hpp"

#include "math.h"
mycar_STRUCT mycar;
float Gyro_Z;
// 左右轮差速表（百分比）
float speed_dif_list[30] = {-0.226, -0.199, -0.18,  -0.169, -0.151, -0.133,
                            -0.119, -0.101, -0.087, -0.075, -0.057, -0.042,
                            -0.026, -0.015, 0,      0.009,  0.026,  0.045,
                            0.061,  0.072,  0.084,  0.096,  0.115,  0.126,
                            0.142,  0.159,  0.182,  0.195,  0.214};
// float
// speed_dif_list_left[30]={0.004,0.006,0.019,0.028,0.043,0.056,0.066,0.080,0.094,0.105,0.120,0.137,0.149,0.156,0.179,0.193,0.214,0.239};
// float
// speed_dif_list_right[30]={0.004,-0.003,-0.020,-0.032,-0.046,-0.054,-0.070,-0.081,-0.088,-0.100,-0.118,-0.128,-0.145,-0.163,-0.178,-0.188,-0.204,-0.217};
float speed_dif_list_left[30] = {0.002, 0.005, 0.026, 0.044, 0.057, 0.074,
                                 0.085, 0.102, 0.114, 0.128, 0.150, 0.167,
                                 0.182, 0.202, 0.229, 0.245, 0.257, 0.261};
float speed_dif_list_right[30] = {
    0.002,  -0.003, -0.029, -0.042, -0.058, -0.073, -0.090, -0.107, -0.121,
    -0.134, -0.152, -0.166, -0.183, -0.204, -0.224, -0.246, -0.252, -0.261};

void steer_buchang_cal() {
  if (mycar.corner_flag == 2 && watch.track_count >= 70)
    mycar.steer_buchang = setpara.steer_buchang * (watch.track_count - 70) / 15;
  else if (mycar.corner_flag == 1 && watch.track_count >= 70)
    mycar.steer_buchang =
        -setpara.steer_buchang * (watch.track_count - 70) / 15;
  else
    mycar.steer_buchang = 0;
}

void fan_buchang_cal() {}

// 方向控制
void dir_control() {
  // get_ad_value();//获取电感数据
  // mycar.tracking_mode=1;
  // if(mycar.tracking_mode==0)//摄像头循迹
  // mycar.steer_pwm=(int16_t)CAM_Turn_PID(-mycar.original_err);
  //  mycar.original_err = (mycar.original_err > 150 ? 150 : (mycar.original_err
  //  < -150 ? -150 : mycar.original_err));

  mycar.d_original_err = mycar.original_err - mycar.last_original_err;
  mycar.last_original_err = mycar.original_err;

  if (mycar.pid_ctrl == 1) {
    updata_fuzzy_pid(mycar.nonlinear_trackpos, mycar.track_diff);
  }
  // mycar.target_gyro=PID_Positional(&CAM_Turn,mycar.original_err,0)*mycar.present_speed;
  // mycar.steer_pwm=setpara.steer_adjust;

  // mycar.steer_pwm=ADRC(&cam_Turn,mycar.original_err*500,0);

  // set_steer(STEER_MID+mycar.steer_pwm);
  //        mycar.steer_pwm=PID_Positional(&CAM_Turn,mycar.original_err,0);
  cout<<"err"<<mycar.original_err<<endl;
  mycar.steer_pwm =
      Steer_PWM_Cal(-mycar.original_err); /////////////////根据情况调整符号
  mycar.uart_servo = STEER_MID+mycar.steer_pwm+setpara.steer_adjust+mycar.steer_buchang;     // 传入
  if (Element == None || Element == crossing)
    steer_buchang_cal();
  else
    mycar.steer_buchang = 0;
  ////////////////////////////
  ///set_steer(STEER_MID+mycar.steer_pwm+setpara.steer_adjust+mycar.steer_buchang);

  //    set_steer(STEER_MID+setpara.steer_adjust);
}

// 电机控制
void motor_control() {

  // mycar.present_speed2=speed_Kalman_Filter2(mycar.present_speed);
  // distant_integeral(
  //     mycar.present_speed); // 路程积分函数///////////////需要把路程积分写入
  // 速度决策.
  speed_ctrl_cal();

  // if(mycar.speed_ctrl==1)
  // {
  //      updata_fuzzy_speed(mycar.original_err,imu.gyroz/(mycar.present_speed+0.01));////////////////先注释要借助z轴加速度
  // }
  //
  //    if(t==0)
  //    {
  //        mycar.steer_pwm=Steer_PWM_Cal(-mycar.original_err)+setpara.steer_adjust;
  //        set_steer(STEER_MID+mycar.steer_pwm);
  //    }
  running_protect(); // 运行保护
  common_running();
  if (mycar.car_running == 0) {
    mycar.uart_speed = 0;
    clear_integral(&Speed_middle);
    clear_integral(&gyro);
  }
}
// 调试速度环的函数
void speed_pid_adjust() {}

// 调试电流环的函数
void current_pid_adjust() {}
// 正常行驶
void common_running() {
  if (mycar.car_running == 1) {
    //        mycar.target_left_speed=mycar.target_speed*(1-mycar.speed_differ/1000*(1000+setpara.differ_ratio));
    //        mycar.target_right_speed=mycar.target_speed*(1+mycar.speed_differ/1000*(1000+setpara.differ_ratio));
    //        mycar.target_left_speed=mycar.target_speed*(1-2*mycar.speed_left_differ/(1+mycar.speed_left_differ)*(1000+setpara.differ_ratio)/1000);
    //        mycar.target_right_speed=mycar.target_speed*(1+2*mycar.speed_right_differ/(1-mycar.speed_right_differ)*(1000+setpara.differ_ratio)/1000);
    /////后轮差速
    // // 根据转向角度计算左右轮的速度，以实现差速转向
    // if(mycar.steer_pwm>=0)mycar.speed_left_differ=tanf((float)mycar.steer_pwm
    // * PI / 1800/2)*setpara.differ_ratio / 100; else
    // mycar.speed_left_differ=0;
    // if(mycar.steer_pwm<=0)mycar.speed_right_differ=tanf((float)mycar.steer_pwm
    // * PI / 1800/2)*setpara.differ_ratio / 100; else
    // mycar.speed_right_differ=0; mycar.target_left_speed =
    // mycar.target_speed*(1-2*mycar.speed_left_differ/(1+mycar.speed_left_differ));
    // mycar.target_right_speed =
    // mycar.target_speed*(1+2*mycar.speed_right_differ/(1-mycar.speed_right_differ));

    // 电流环串速度环
    //        mycar.target_current_l=PID_Positional(&Speed_left,
    //        mycar.left_speed, mycar.target_left_speed);
    //        mycar.target_current_r=PID_Positional(&Speed_right,
    //        mycar.right_speed, mycar.target_right_speed);

    // 纯速度环
    //  mycar.left_pwm_set=PID_Incremental(&Speed_left,mycar.left_speed,mycar.target_left_speed);//+setpara.speed_kf*Speed_left.kf_ratio*mycar.target_left_speed;
    //  mycar.right_pwm_set=PID_Incremental(&Speed_right,mycar.right_speed,mycar.target_right_speed);//+setpara.speed_kf*Speed_right.kf_ratio*mycar.target_right_speed;
    //  set_pwm(mycar.left_pwm_set,mycar.right_pwm_set);
    mycar.uart_speed = mycar.target_speed; //////////需要的速度给到串口
                                           // fan_begin(setpara.fan_speed);
                                           // set_pwm(0,1000);
    //        mycar.speed_Middle_PWM=velocity_PID_middle(mycar.present_speed2,mycar.target_speed);
    //        mycar.left_pwm_set=mycar.speed_Middle_PWM+mycar.Turn_PWM;
    //        mycar.right_pwm_set=mycar.speed_Middle_PWM-mycar.Turn_PWM;
  }
}
// 电流环控制
void current_control() {}
// 小车运行保护
void running_protect() {
  // static int danger_time_speed,danger_time_distence,danger_time_black;
  // if(mycar.car_running==1)
  // {
  //     mycar.RUNTIME++;
  //     //时间保护
  //     if(mycar.RUNTIME>setpara.stop_time)
  //     {
  //         if(setpara.stop_time>0&&setpara.stop_time<30000)
  //         {
  //             mycar.RUNTIME=0;
  //             car_stop();
  //             if(mycar.stop_mode==0)
  //             mycar.stop_mode=1;
  //         }
  //     }
  //     if(mycar.RUNTIME>1000&&Element!=zebra)//失速保护
  //     {
  //         if((mycar.left_speed<1&&abs(mycar.left_pwm_set)>1000)||(mycar.right_speed<1&&abs(mycar.right_pwm_set)>1000))
  //         {
  //             danger_time_speed++;
  //             if(danger_time_speed>200)
  //             {
  //                 danger_time_speed=0;
  //                 mycar.car_running=0;
  //                 car_stop();
  //                 stop_warning();
  //                 clear_integral(&Speed_left);
  //                 clear_integral(&Speed_right);
  //                 if(mycar.stop_mode==0)
  //                 mycar.stop_mode=2;
  //             }
  //         }
  //         else if(danger_time_speed>0)danger_time_speed--;
  //     }
  //     //距离保护
  //     if(dl1b_distance_mm<setpara.obstacle_detect_distance)
  //     {
  //         danger_time_distence++;
  //         if(danger_time_distence>100)
  //         {
  //             danger_time_distence=0;
  //             mycar.car_running=0;
  //             car_stop();
  //             stop_warning();
  //             clear_integral(&Speed_left);
  //             clear_integral(&Speed_right);
  //             if(mycar.stop_mode==0)
  //             mycar.stop_mode=3;
  //         }
  //     }
  //     else if(danger_time_distence>0)danger_time_distence--;
  //     //电感保护
  //     if(adc.data[0]<setpara.adc_protect&&adc.data[1]<setpara.adc_protect&&adc.data[2]<setpara.adc_protect&&Element!=ingarage&&Element!=outgarage&&Element!=obstacle)
  //     {
  //         //car_stop();
  //     }

  //     //视觉保护
  //     if(setpara.track_open_flag>=1){
  //     if(mycar.track_warn_flag)
  //     {
  //         danger_time_black++;
  //         if(danger_time_black>100)
  //         {
  //             danger_time_black=0;
  //             mycar.car_running=0;
  //             car_stop();
  //             // stop_warning();
  //             clear_integral(&Speed_left);
  //             clear_integral(&Speed_right);
  //             if(mycar.stop_mode==0)
  //             mycar.stop_mode=4;
  //             }
  //     }
  //     else if(danger_time_black>0)danger_time_black--;
  //     }
  // }
}

// 发车函数
void car_begin() /// 放在主函数开头以设置初始标识位
{
  clear_all_flags();
  // element_init();
  mycar.car_stop = 0;
  mycar.car_running = 1;
  mycar.RUNTIME = 0;
  mycar.stop_mode = 0;
  // out_garage();//出库起步函数
}
void car_stop() /// 放在主函数开头以设置初始标识位
{
  // clear_all_flags();
  // mycar.car_stop=1;
  mycar.car_running = 0;
  mycar.start_warn_flag = 0;
  //    set_speed(0);
  //    change_pid_para(&Speed_middle,&setpara.stop_PID);
  // fan_stop();
  // set_fan(50,50);
  setpara.fan_start = 40;
}
void fan_begin(uint8_t speed) {
  // int left_fan_speed,right_fan_speed;
  // left_fan_speed=(int)10*speed*(1-mycar.fan_buchang);
  // right_fan_speed=(int)10*speed*(1+mycar.fan_buchang);
  // set_fan(left_fan_speed,right_fan_speed);
}
// 速度控制，偏差越大，速度越慢
uint16_t speed_list[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
#define err_max 1
#define err_min 0
#define MTC_LKT_ERR_MIN_INPUT 0
#define MTC_LKT_ERR_SIZE 0.1
#define MTC_LKT_ERR_COUNT 10
float mtc_get_speed_result(unsigned int val, unsigned int *lookup_table) {
  int seg_idx;
  int inter_val;
  if (val <= 0) {
    return (float)lookup_table[0];
  }
  if (val >= 1) {
    return (float)lookup_table[MTC_LKT_ERR_COUNT];
  }
  seg_idx = 1.0 * val / MTC_LKT_ERR_SIZE;
  inter_val = val - seg_idx * MTC_LKT_ERR_SIZE;
  return (float)((lookup_table[seg_idx] * (MTC_LKT_ERR_SIZE - inter_val) +
                  lookup_table[seg_idx + 1] * inter_val) /
                 MTC_LKT_ERR_SIZE);
}
float speed_differ_set(int16_t steer_pwm) // 设置左右两轮差速
{
  uint8_t step; // 当前舵机所对应的级别
  int16_t pos;
  float membership; // 隶属度
  float result;     // 结果
  if (steer_pwm >= 0) {
    step = steer_pwm / 30;
    if (step > 17)
      step = 17;
    membership = (float)(steer_pwm - 30 * step) / 30;
    result = speed_dif_list_left[step] * (1 - membership) +
             speed_dif_list_left[step + 1] * membership;
  } else if (steer_pwm < 0) {
    steer_pwm = -steer_pwm;
    step = steer_pwm / 30;
    if (step > 17)
      step = 17;
    membership = (float)(steer_pwm - 30 * step) / 30;
    result = speed_dif_list_right[step] * (1 - membership) +
             speed_dif_list_right[step + 1] * membership;
  }
  return result;
}

float speed_left_differ_set(int16_t steer_pwm) // 设置左轮差速
{
  uint8_t step; // 当前舵机所对应的级别
  int16_t pos;
  float membership; // 隶属度
  float result;     // 结果
  if (steer_pwm >= 0) {
    step = steer_pwm / 30;
    if (step > 17)
      step = 17;
    membership = (float)(steer_pwm - 30 * step) / 30;
    result = speed_dif_list_left[step] * (1 - membership) +
             speed_dif_list_left[step + 1] * membership;
  } else if (steer_pwm < 0) {
    result = 0;
  }
  return result;
}
float speed_right_differ_set(int16_t steer_pwm) // 设置右轮差速
{
  uint8_t step; // 当前舵机所对应的级别
  int16_t pos;
  float membership; // 隶属度
  float result;     // 结果
  if (steer_pwm >= 0) {
    result = 0;
  } else if (steer_pwm < 0) {
    steer_pwm = -steer_pwm;
    step = steer_pwm / 30;
    if (step > 17)
      step = 17;
    membership = (float)(steer_pwm - 30 * step) / 30;
    result = speed_dif_list_right[step] * (1 - membership) +
             speed_dif_list_right[step + 1] * membership;
  }
  return result;
}

void speed_ctrl() {
  static float target_speed;
  float k;
  float rate;
  if (mycar.speed_ctrl == 1) {
    /*        if(abs(mycar.original_err)>err_max)k=100;
            k=(abs(mycar.original_err)-err_min)/err_max*100;
            rate=0.1*(k%10);
            target_speed=k*speed_list[k/10]*rate+speed_list[k/10+1]*(1-rate);*/
    //        if(mycar.original_err<0)
    //            target_speed=(float)setpara.speed_max+mycar.original_err*(float)setpara.speed_adjust_kp;
    //        else
    //            target_speed=(float)setpara.speed_max-mycar.original_err*(float)setpara.speed_adjust_kp;
    //        if(target_speed<setpara.speed_min)target_speed=setpara.speed_min;
    //        mycar.target_speed=target_speed;
  }
}
// 设置固定速度,此时取消速度调整,直到调用clear_all_flag()函数
void set_speed(uint16_t speed) {
  mycar.speed_ctrl = 0;
  mycar.target_speed = speed;
}
