/*
 * mycar.c
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */
#include "mycar.h"
#pragma section all "cpu0_dsram"

//�����ֲ��ٱ��ٷֱȣ�
float speed_dif_list[30]={
        -0.226,-0.199,-0.18,-0.169,-0.151,-0.133,-0.119,-0.101,-0.087,-0.075,-0.057,-0.042,-0.026,-0.015,
        0,0.009,0.026,0.045, 0.061,0.072,0.084,0.096,0.115,0.126,0.142,0.159,0.182,0.195,0.214};
//�������
void dir_control()
{
//        //mycar.steer_pwm=(int16)CAM_Turn_PID(-mycar.original_err);
//    mycar.steer_pwm=PID_Positional(&CAM_Turn,mycar.original_err,0);
//        //mycar.steer_pwm=setpara.steer_adjust;
//    mycar.steer_pwm=747;
    set_steer(mycar.steer_pwm);
//    set_steer(STEER_MID);

}
//�������
//uint8 left_or_right=0;
//void dir_pid_adjust()
//{
//
//    //mycar.steer_pwm=(int16)CAM_Turn_PID(-mycar.original_err);
//    if(mycar.TIME%5000==0)
//    {
//        if(left_or_right==0)
//        {
//            mycar.original_err=1000;
//            left_or_right=1;
//        }
//        else
//        {
//            left_or_right=0;
//            mycar.original_err=-1000;
//        }
//    }
//    mycar.steer_pwm=PID_Positional(&CAM_Turn,mycar.original_err,0);
//        //mycar.steer_pwm=setpara.steer_adjust;
//
//    set_steer(STEER_MID+mycar.steer_pwm);
//    //set_steer(STEER_MID+);
//
//}
//�������
void motor_control(void)
{
//    setpara.control_speed=2;
//    mycar.present_speed=200;
    mycar.target_speed=setpara.control_speed;
//    if(mycar.target_speed>0)mycar.car_running=1;
//    else mycar.car_running=0;
//    mycar.car_running=1;
//    mycar.present_speed=encoder_get_speed();
////    mycar.present_speed=encoder_get_count(TIM2_ENCODER);
//    mycar.present_speed2=mycar.present_speed/5500*1000;
//    set_pwm(3000);
////    distant_integeral(mycar.present_speed);//·�̻��ֺ���
//    speed_pid_adjust();
//    //speed_ctrl();//�ٶȾ���
//    running_protect();//���б���

    common_running();

}
//�����ٶȻ��ĺ���
void speed_pid_adjust()
{
    if(mycar.TIME>1000&&mycar.TIME<setpara.stop_time)
    {
        mycar.car_running=1;
    }
    //���Թ̶�PWM��ʻ
    if(mycar.RUNTIME<setpara.begin_time&&mycar.RUNTIME>0)
    {
        mycar.pwm_set=setpara.begin_pwm;
    }
    //�����ٶȻ�PID��ʻ
    else if(mycar.RUNTIME>setpara.begin_time&&mycar.RUNTIME<setpara.stop_time)
    {
//        mycar.target_speed=2.0;     //�����ٶ�
//        mycar.pwm_set=PID_Positional(&Speed_middle,mycar.present_speed,mycar.target_speed);
      mycar.pwm_set=PID_Positional(&Speed_middle,mycar.present_speed,mycar.target_speed);
    }
    //��ʻһ��ʱ��ֹͣ
    else if(mycar.RUNTIME>setpara.stop_time)
    {
        mycar.car_running=0;
//        mycar.RUNTIME=0;
    }
    if(mycar.car_running==1)
    {
        set_pwm(mycar.pwm_set);
    }

}
//������ʻ
void common_running()
{
    if(mycar.car_running==1)
    {   mycar.RUNTIME++;
        mycar.pwm_set=PID_Positional(&Speed_middle,mycar.present_speed,mycar.target_speed);
        set_pwm(mycar.pwm_set);
    }
    else
    {
        set_pwm(0);
    }
}
//С�����б���
void running_protect()
{
    static int danger_time;
    if(mycar.car_running==1)
    {
        mycar.RUNTIME++;
        //ʱ�䱣��
        if(mycar.RUNTIME>setpara.stop_time)
        {
            if(setpara.stop_time>0&&setpara.stop_time<20000)
            {
                mycar.RUNTIME=0;
                car_stop();
            }
        }
        if(mycar.RUNTIME>1000)//ʧ�ٱ���
        {
            if((mycar.present_speed<1&&abs(mycar.pwm_set)>1000))
            {
                danger_time++;
                if(danger_time>200)
                {
                    danger_time=0;
                    mycar.car_running=0;
                    car_stop();
                    clear_integral(&Speed_middle);

                }
            }
        }
        //���뱣��
        if(dl1a_distance_mm<100)
        {
           // mycar.car_running=0;
        }

    }

}
//��������
void car_begin()
{
//    clear_all_flags();
    //element_init();
    mycar.TIME=0;
    mycar.RUNTIME=0;
    mycar.car_stop=0;
    mycar.car_running=1;
    pit_enable(CCU60_CH0);//��ʱ���ж�1(PWM)

    //out_garage();//�����𲽺���
    car_start_warning();//������������Ȼ��ʼ����
//    beep_warning();
    UART_send_car_start();
}
void car_stop()
{
    //clear_all_flags();
    //mycar.car_stop=1;
    mycar.car_running=0;
    set_pwm(0);
    pit_disable(CCU60_CH0);
//    set_speed(0);
//    change_pid_para(&Speed_middle,&setpara.stop_PID);

}
//�ٶȿ��ƣ�ƫ��Խ���ٶ�Խ��
uint16 speed_list[10]={1,1,1,1,1,1,1,1,1,1};
#define err_max 1
#define err_min 0
#define MTC_LKT_ERR_MIN_INPUT 0
#define MTC_LKT_ERR_SIZE  0.1
#define MTC_LKT_ERR_COUNT 10
float mtc_get_speed_result(unsigned int val,unsigned int *lookup_table)
{
   int seg_idx;
   int inter_val;
     if (val <= 0)
     {
       return (float)lookup_table[0];
     }
     if (val >= 1)
     {
       return (float)lookup_table[MTC_LKT_ERR_COUNT];
     }
     seg_idx = 1.0*val / MTC_LKT_ERR_SIZE;
     inter_val = val - seg_idx * MTC_LKT_ERR_SIZE;
     return (float)((lookup_table[seg_idx] * (MTC_LKT_ERR_SIZE - inter_val) + lookup_table[seg_idx + 1] * inter_val) / MTC_LKT_ERR_SIZE);
}
float speed_differ_set(int16 steer_pwm)//�����������ֲ���
{
    uint8 step;//��ǰ�������Ӧ�ļ���
    int16 pos;
    float membership;//������
    float result; //���
    pos=steer_pwm+70;
    if(pos>140)pos=140;
    else if(pos<0)pos=0;
    step=pos/5;
    membership=(float)(pos-5*step)/5;
    result=speed_dif_list[step]*(1-membership)+speed_dif_list[step+1]*membership;
    return result;
}
void speed_ctrl()
{
    static float target_speed;
    float k;
    float rate;
    if(mycar.speed_ctrl==1)
    {
/*        if(abs(mycar.original_err)>err_max)k=100;
        k=(abs(mycar.original_err)-err_min)/err_max*100;
        rate=0.1*(k%10);
        target_speed=k*speed_list[k/10]*rate+speed_list[k/10+1]*(1-rate);*/
        if(mycar.original_err<0)
            target_speed=(float)setpara.speed_max+mycar.original_err*(float)setpara.speed_adjust_kp;
        else
            target_speed=(float)setpara.speed_max-mycar.original_err*(float)setpara.speed_adjust_kp;
        if(target_speed<setpara.speed_min)target_speed=setpara.speed_min;
        mycar.target_speed=target_speed;
    }
}
//���ù̶��ٶ�,��ʱȡ���ٶȵ���,ֱ������clear_all_flag()����
void set_speed(uint16 speed)
{
    mycar.speed_ctrl=0;
    mycar.target_speed=speed;

}
#pragma section all restore




