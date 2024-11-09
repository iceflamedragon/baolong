/*
 * broken_circuit.c
 *
 *  Created on: 2023年7月11日
 *      Author: Admin
 */
#include "broken_circuit.h"
//断路准备函数

void broken_circuit_prepare()
{
    if(watch.broken_circuit_flag==0)
    {
        clear_all_flags();
        enter_element(broken_circuit);
        watch.broken_circuit_flag=4;
        begin_distant_integeral(setpara.distanceThres_before_broken_circuit);
    }
}
//断路减速函数
void broken_circuit_slow()
{
    if(watch.broken_circuit_flag==4&&get_integeral_state(&distance_integral)==2)
    {
        clear_distant_integeral();
        watch.broken_circuit_flag=1;
        set_speed(setpara.broken_target_speed);
        change_pid_para(&Speed_middle,&setpara.speed_down_PID);
        begin_distant_integeral(setpara.broken_circuit_slow_distane);
    }
}
//进入断路函数
void broken_circuit_enter()
{
    if(watch.broken_circuit_flag==1&&get_integeral_state(&distance_integral)==2)
    {
        clear_distant_integeral();
        watch.broken_circuit_flag=2;
        //set_speed(setpara.broken_target_speed);
        mycar.tracking_mode=1;//开启电感循迹
        begin_distant_integeral(setpara.distanceThres_in_broken_circuit);
    }
}
//完全进入断路函数
void broken_circuit_complete_enter()
{
    uint8_t black_count,black_line_count=0;
    if(watch.broken_circuit_flag==2&&get_integeral_state(&distance_integral)==2)
    {
        for(int y=20;y<30;y++)
        {
            /*for(int x=0;x<188;x++)
            {
                if(Grayscale[119-y][x]==0)
                {
                    black_count++;
                }
            }*/
            if(lineinfo[y].left==0&&lineinfo[y].right==0)
            black_line_count++;
        }
        // vofa.broken[6]=black_line_count;
        if(black_line_count>=9)
        {
            watch.broken_circuit_flag=3;
        }
    }
}
void broken_circuit_complete_out()
{
    uint8_t white_count,white_line_count=0;
    if(watch.broken_circuit_flag==3)
    {
        for(int y=20;y<30;y++)
        {
/*            for(int x=0;x<188;x++)
            {
                if(Grayscale[119-y][x]==255)
                {
                    white_count++;
                }
            }*/
            if(lineinfo[y].left<10&&lineinfo[y].right>160)
            white_line_count++;
        }
        // vofa.broken[7]=white_line_count;
        if(white_line_count>=9)
        {

            out_element();
        }
    }
}


