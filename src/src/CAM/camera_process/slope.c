/*
 * slope.c
 *
 *  Created on: 2023年7月10日
 *      Author: Admin
 */
#include "slope.h"
//进入坡道函数

int slope_count;
void slope_enter()
{
    // //坡道前减速
    // if(Element==NONE){
    // if(dl1b_distance_mm<1000){
    //     slope_count++;
    //     if(slope_count>=3){
    //     set_speed(setpara.slope_speed);
    //     watch.slope_flag=1;
    //     }

    // }
    // else {
    //     slope_count=0;
    //     if(watch.slope_flag==1){
    //         mycar.speed_ctrl=1;
    //         watch.slope_flag=0;
    //     }
    // }
    // }
    // if(imu.pitch>15&&mycar.RUNTIME>setpara.slope_begin_time)//因陀螺仪位置不同修改
    // {
    //     clear_all_flags();
    //     enter_element(Slope);
    //     set_speed(setpara.slope_speed);
    //     watch.slope_flag=2;
    //     watch.angle_far_line=50;
    //     begin_distant_integeral(10000);
    //     // beep2(4,100);
    // }
}
//下坡函数
void slope_down()
{
    // if((imu.pitch<0||get_integeral_state(&distance_integral)==2)&&watch.slope_flag==2)//因陀螺仪位置不同修改
    // {
    //     watch.angle_far_line=setpara.far_line;
    //     watch.slope_flag=3;
    //     begin_distant_integeral(10000);
    // }

}
//出斜坡函数
void slope_out()
{
    // if(watch.slope_flag==3&&get_integeral_state(&distance_integral)==2)
    // {
    //     imu.pitch=0;
    //     out_element();
    // }
}


