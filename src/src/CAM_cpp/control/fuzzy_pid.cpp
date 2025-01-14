/*
 * fuzzy_pid.c
 *
 *  Created on: 2024年3月25日
 *      Author: 14250
 */
#include "fuzzy_pid.hpp"
#include "mycar.hpp"
#define E_MAX 200
#define E_MIN -200
#define dE_MAX 20
#define dE_MIN -20
#define K_MAX 18
#define K_MIN -18
int watchleft_count=0;
int watchright_count=0;
int watch_count=0;
int watch_count2=0;
float result_last=10;
float result=10; //结果
float speed_add;

int far_line_count;
PID_para_STRUCT CAM_FUZZY_PID;
int16_t FUZZY_LIST_KP[7][7]=
{
PB,PB,PM,PM,PS,PS,ZO,
PB,PM,PM,PS,PS,ZO,PS,
PM,PM,PM,PS,ZO,PS,PS,
PM,PS,PS,ZO,PS,PS,PM,
PS,PS,ZO,PS,PM,PM,PM,
PS,ZO,PS,PS,PM,PM,PB,
ZO,PS,PS,PM,PM,PB,PB};

int16_t FUZZY_LIST_KD[7]=
//{4,3,1,0,1,3,4};
{0,0,0,0,0,0,0};

//{
//PB,PS,PS,NS,PS,PS,PM,
//PB,PS,PS,NS,NS,ZO,PS,
//ZO,ZO,ZO,NM,NM,NS,NS,
//ZO,NS,NS,NB,NS,NS,ZO,
//NS,NS,NM,NM,ZO,ZO,ZO,
//PS,ZO,NS,NS,PS,PS,PB,
//PM,PS,PS,NS,PS,PS,PB};

//{
//PB,PB,PM,PM,PS,PS,ZO,
//PB,PM,PM,PS,PS,ZO,PS,
//PM,PM,PM,PS,ZO,PS,PS,
//PM,PS,PS,ZO,PS,PS,PM,
//PS,PS,ZO,PS,PM,PM,PM,
//PS,ZO,PS,PS,PM,PM,PB,
//ZO,PS,PS,PM,PM,PB,PB};


int16_t FUZZY_LIST_SPEED[7][7]=
{
ZO,ZO,PS,PS,ZO,ZO,ZO,
ZO,PS,PS,PS,PS,ZO,ZO,
PS,PS,PM,PM,PS,PS,ZO,
ZO,PS,PM,PB,PM,PS,PS,
ZO,PS,PS,PM,PM,PS,PS,
ZO,ZO,PS,PS,PS,PS,ZO,
ZO,ZO,ZO,PS,PS,ZO,ZO};
//mycar.original_err和imu.gyroz/mycar.present_speed(近似于曲率K)
//极性相反
float get_fuzzy_kp_data(float err,float d_err)
{
    uint8_t fuzzy_pos_e,fuzzy_pos_de;
    float membership_e,membership_de,fuzzy_out,fuzzy_out1,fuzzy_out2,fuzzy_out3,fuzzy_out4;
    fuzzy_pos_e=err*3/E_MAX+3;
    if(fuzzy_pos_e<0)fuzzy_pos_e=0;
    else if(fuzzy_pos_e>5)fuzzy_pos_e=5;
    membership_e=(err*3/E_MAX+3-fuzzy_pos_e);
    fuzzy_pos_de=d_err*3/dE_MAX+3;
    if(fuzzy_pos_de<0)fuzzy_pos_de=0;
    else if(fuzzy_pos_de>5)fuzzy_pos_de=5;
    membership_de=(d_err*3/dE_MAX+3-fuzzy_pos_de);
    if(err*3/E_MAX+3>6)membership_e=1;
    else if(err*3/E_MAX+3<0)membership_e=0;
    if(d_err*3/dE_MAX+3>6)membership_de=1;
    else if(d_err*3/dE_MAX+3<0)membership_de=0;
    fuzzy_out1=(1-membership_e)*(1-membership_de)*FUZZY_LIST_KP[fuzzy_pos_e][fuzzy_pos_de];
    fuzzy_out2=(membership_e)*(1-membership_de)*FUZZY_LIST_KP[fuzzy_pos_e+1][fuzzy_pos_de];
    fuzzy_out3=(1-membership_e)*(membership_de)*FUZZY_LIST_KP[fuzzy_pos_e][fuzzy_pos_de+1];
    fuzzy_out4=(membership_e)*(membership_de)*FUZZY_LIST_KP[fuzzy_pos_e+1][fuzzy_pos_de+1];
    fuzzy_out=fuzzy_out1+fuzzy_out2+fuzzy_out3+fuzzy_out4;
    return fuzzy_out;
}
//
//float get_fuzzy_kd_data(float err,float d_err)
//{
//    uint8_t fuzzy_pos_e,fuzzy_pos_de;
//    float membership_e,membership_de,fuzzy_out,fuzzy_out1,fuzzy_out2,fuzzy_out3,fuzzy_out4;
//    fuzzy_pos_e=3+err*3/E_MAX;
//    if(fuzzy_pos_e<0)fuzzy_pos_e=0;
//    else if(fuzzy_pos_e>5)fuzzy_pos_e=5;
//    membership_e=(err*3/E_MAX+3-fuzzy_pos_e);
//    fuzzy_pos_de=3+d_err*3/dE_MAX;
//    if(fuzzy_pos_de<0)fuzzy_pos_de=0;
//    else if(fuzzy_pos_de>5)fuzzy_pos_de=5;
//    membership_de=(d_err*3/dE_MAX+3-fuzzy_pos_de);
//    if(err*3/E_MAX+3>6)membership_e=1;
//    else if(err*3/E_MAX+3<0)membership_e=0;
//    if(d_err*3/dE_MAX+3>6)membership_de=1;
//    else if(d_err*3/dE_MAX+3<0)membership_de=0;
//    fuzzy_out1=(1-membership_e)*(1-membership_de)*FUZZY_LIST_KD[fuzzy_pos_e][fuzzy_pos_de];
//    fuzzy_out2=(membership_e)*(1-membership_de)*FUZZY_LIST_KD[fuzzy_pos_e+1][fuzzy_pos_de];
//    fuzzy_out3=(1-membership_e)*(membership_de)*FUZZY_LIST_KD[fuzzy_pos_e][fuzzy_pos_de+1];
//    fuzzy_out4=(membership_e)*(membership_de)*FUZZY_LIST_KD[fuzzy_pos_e+1][fuzzy_pos_de+1];
//    fuzzy_out=fuzzy_out1+fuzzy_out2+fuzzy_out3+fuzzy_out4;
//    return fuzzy_out;
//}

float get_fuzzy_kd_data_1(float err,float d_err)
{
    uint8_t fuzzy_pos_de;
        float membership_de,fuzzy_out,fuzzy_out1,fuzzy_out2;
        fuzzy_pos_de=3+d_err*3/dE_MAX;
        if(fuzzy_pos_de<0)fuzzy_pos_de=0;
        else if(fuzzy_pos_de>5)fuzzy_pos_de=5;
        membership_de=(d_err*3/dE_MAX+3-fuzzy_pos_de);
        if(d_err*3/dE_MAX+3>6)membership_de=1;
        else if(d_err*3/dE_MAX+3<0)membership_de=0;
        fuzzy_out1=membership_de*FUZZY_LIST_KD[fuzzy_pos_de];
        fuzzy_out2=(1-membership_de)*FUZZY_LIST_KD[fuzzy_pos_de+1];
        fuzzy_out=fuzzy_out1+fuzzy_out2;
        return fuzzy_out;
}



float get_fuzzy_speed_data(float err,float k)
{
    uint8_t fuzzy_pos_e,fuzzy_pos_k;
    float membership_e,membership_k,fuzzy_out,fuzzy_out1,fuzzy_out2,fuzzy_out3,fuzzy_out4;
    fuzzy_pos_e=err*3/E_MAX+3;
    if(fuzzy_pos_e<0)fuzzy_pos_e=0;
    else if(fuzzy_pos_e>5)fuzzy_pos_e=5;
    membership_e=(err*3/E_MAX+3-fuzzy_pos_e);
    fuzzy_pos_k=k*3/K_MAX+3;
    if(fuzzy_pos_k<0)fuzzy_pos_k=0;
    else if(fuzzy_pos_k>5)fuzzy_pos_k=5;
    membership_k=(k*3/K_MAX+3-fuzzy_pos_k);
    if(err*3/E_MAX+3>6)membership_e=1;
    else if(err*3/E_MAX+3<0)membership_e=0;
    if(k*3/K_MAX+3>6)membership_k=1;
    else if(k*3/K_MAX+3<0)membership_k=0;
    fuzzy_out1=(1-membership_e)*(1-membership_k)*FUZZY_LIST_SPEED[fuzzy_pos_e][fuzzy_pos_k];
    fuzzy_out2=(membership_e)*(1-membership_k)*FUZZY_LIST_SPEED[fuzzy_pos_e+1][fuzzy_pos_k];
    fuzzy_out3=(1-membership_e)*(membership_k)*FUZZY_LIST_SPEED[fuzzy_pos_e][fuzzy_pos_k+1];
    fuzzy_out4=(membership_e)*(membership_k)*FUZZY_LIST_SPEED[fuzzy_pos_e+1][fuzzy_pos_k+1];
    fuzzy_out=fuzzy_out1+fuzzy_out2+fuzzy_out3+fuzzy_out4;
    return fuzzy_out;
}
void updata_fuzzy_pid(float err,float d_err)
{
    float d_kp,d_kd;
    d_kp=0.1*setpara.fuzzy_kp*get_fuzzy_kp_data(err,d_err);
    d_kd=0.1*setpara.fuzzy_kd*get_fuzzy_kd_data_1(err,d_err);
    // CAM_FUZZY_PID.kp=setpara.com_turn_PID.kp+d_kp;///这个是模糊实际上没有用到
    // CAM_FUZZY_PID.kd=setpara.com_turn_PID.kd+d_kd;///
    CAM_FUZZY_PID.kp=setpara.com_turn_PID.kp;////////相当于直接用的这个pid
    CAM_FUZZY_PID.kd=setpara.com_turn_PID.kd;
    //////////////提前入弯时机，应该需要修改的地方
    if(watch.track_count<90&&watch.track_count>70)far_line_count++;
    else far_line_count--;
    if(far_line_count>=3)far_line_count=3;
    if(far_line_count<0)far_line_count=0;
//    if(far_line_count==3)watch.angle_far_line=80;
//    if(far_line_count==0)watch.angle_far_line=setpara.far_line;

}

void updata_fuzzy_speed(float err,float k)
{
    //speed_add
    
    if(watch.track_count<setpara.speed_max)watch_count2++;else watch_count2--;
    if(watch_count2<=0){watch_count2=0;speed_add=setpara.speed_add;}
    if(watch_count2>=3){watch_count2=3;speed_add=0;}
    cout<<watch_count2<<endl;
    //if(mycar.RUNTIME<setpara.begin_time)speed_add=setpara.speed_add;//起步斑马线黑块误识别
    //d_k
    float d_k;
    if(watch.track_count<=70)
    d_k=setpara.fuzzy_k*(watch.track_count)*0.1/70;
    else d_k=setpara.fuzzy_k*0.1;

       mycar.target_speed=setpara.speed_min+speed_add;//+d_k;
    //    cout<<"加和求得速度"<<mycar.target_speed<<endl;

}

float speed2[15]={0.2,0.2,0.2,0.21,0.21,0.22,0.24,0.27,0.30,0.33,0.38,0.4,0.45,0.5,1};

float fuzzy_speed2(float d_k)//没有用到
{
    uint8_t step;//当前舵机所对应的级别
    int16_t pos;
    float membership;//隶属度

    if(watch.watchleft<68&&watch.watchright>68 ){
//    step=watch.watchleft/5;
//    if(step>14)step=14;
//    membership=(float)(watch.watchleft-5*step)/5;
//    result=speed2[step]*(1-membership)+speed2[step+1]*membership;
//    result=0;
//        watchleft_count++;
//        watchright_count--;
        watch_count++;
    }

    else if(watch.watchleft>68&&watch.watchright<68){
//    step=watch.watchright/5;
//    if(step>14)step=14;
//    membership=(float)(watch.watchright-5*step)/5;
//    result=speed2[step]*(1-membership)+speed2[step+1]*membership;
////    result=0;
//        watchleft_count--;
//        watchright_count++;
        watch_count++;
    }
    else if(watch.watchleft>=68&&watch.watchright>=68){
//        watchleft_count--;
//        watchright_count--;
        watch_count--;
    }

    if(watch_count>=3){
        watch_count=3;
        result=0;
    }
    else if(watch_count<0)
    {
        watch_count=0;
        result=8;
    }
//    if(watchright_count>=3){
//        watchright_count=3;
//        result=0;
//    }
//    else if(watchright_count<0)
//    {
//        watchright_count=0;
//        result=8;
//    }

    return result;
}

int car_track[120]={65,66,66,66,66,66,66,67,67,67,67,68,68,68,69,69,69,69,70,70,
        70,70,71,71,71,71,72,72,72,72,72,73,73,73,73,74,74,74,74,74,
        75,75,75,75,76,76,76,76,77,77,77,77,78,78,78,79,79,79,79,79,
        80,80,80,80,81,81,81,81,82,82,82,82,83,83,83,83,84,84,84,84,
        84,84,85,85,85,85,86,86,86,86,86,86,87,87,87,87,88,88,88,88,
        88,88,88,89,89,89,89,89,89,90,90,90,90,91,91,91,92,92,92,92};

void speed_ctrl_cal()//速度决策
{
    int left_track_count=0;
    int right_track_count=0;
    int left_track_flag=0;
    int right_track_flag=0;
    for(int y=0;y<110;y++){
        if(Grayscale[119-y][car_track[y]+4]>50&&left_track_flag<4){
            left_track_count++;
            left_track_flag=0;}
        else if(Grayscale[119-y][car_track[y]+4]==0)left_track_flag++;
        if(Grayscale[119-y][183-car_track[y]]>50&&right_track_flag<4){
            right_track_count++;
            right_track_flag=0;}
        else if(Grayscale[119-y][183-car_track[y]]==0)right_track_flag++;
    }


    watch.track_count=left_track_count<=right_track_count?left_track_count:right_track_count;
    watch.track_count_far=left_track_count<=right_track_count?right_track_count:left_track_count;

    if(left_track_count<=3&&right_track_count<=3)mycar.track_warn_flag=1;
    else mycar.track_warn_flag=0;

    if(left_track_count-right_track_count>=2&&watch.track_count<85)mycar.corner_flag=1;
    else if(right_track_count-left_track_count>=2&&watch.track_count<85)mycar.corner_flag=2;
    else {mycar.corner_flag=0;}


}