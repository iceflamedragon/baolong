/*
 * beep.c
 *
 *  Created on: 2023年3月14日
 *      Author: Admin
 */

#include "beep.h"
uint16 beep_time;
uint16 yinjie[8]={1000,262,294,330,349,392,440,494};
uint8 warn_flag;
uint8 start_warn_flag;
void beep_init()
{
    pwm_init(beep_port,1000,0);
}
void beeping()
{
    if(beep_time!=0)
        {
            beep_time--;
        }
    if(beep_time==1)
        pwm_set_duty(beep_port,0);

}
void beep(uint16 time)
{
    pwm_init(beep_port,yinjie[0],10000);
    beep_time=time;
}
void beep2(uint8 freq,uint16 time)
{
    pwm_init(beep_port,yinjie[freq],5000);
    beep_time=time;
}
//发车前蜂鸣器响,响完再发车
void car_start_warning()
{
    warn_flag=1;
    start_warn_flag=1;
}
void warning()
{
    warn_flag=1;
}
//蜂鸣器报警
void beep_warning()
{
    static uint16 num;
    if(warn_flag==1)
    {
       if(num==0)
       {
           beep2(1,40);
       }
       else if(num==50)
       {
           beep2(2,20);
       }
       else if(num==100)
       {
           beep2(3,20);
       }
       else if(num==150)
       {
           beep2(4,20);
       }
       else if(num==200)
       {
           beep2(5,20);
       }
       else if(num==250)
       {
           beep2(6,20);
       }
       else if(num==300)
       {
           beep2(7,20);
       }
       num++;
       if(num==350)
       {
           num=0;
           warn_flag=0;
           if(start_warn_flag==1)
            {
                start_warn_flag=0;
                //car_begin();
            }
       }
    }

}


