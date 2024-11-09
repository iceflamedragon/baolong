/*
 * garage.c
 *
 *  Created on: 2023年6月22日
 *      Author: Admin
 */
#include "garage.hpp"
#include "../control/mycar.hpp"
//出库起步函数
using namespace std;
void out_garage()
{
    switch(setpara.start_mode)
    {
        case 1:  //左出库
            enter_element(outgarage);
            watch.out_garage_flag=1;
            begin_distant_integeral(setpara.out_garage_distance);
            break;
        case 2: //右出库
            enter_element(outgarage);
            watch.out_garage_flag=3;
            begin_distant_integeral(setpara.out_garage_distance);
            break;
        default: //不出库
        out_element();
        break;


    }
}
//出库转向
void out_garage_turn()
{
    if(watch.out_garage_flag==1&&get_integeral_state(&distance_integral)==2)
    {
        watch.out_garage_flag=2;
        begin_angle_integeral(setpara.out_garage_angle);
    }
    else if(watch.out_garage_flag==3&&get_integeral_state(&distance_integral)==2)
    {
        watch.out_garage_flag=4;
        begin_angle_integeral(-setpara.out_garage_angle);
    }
}
//完全离开车库
void complete_out_garage()
{
    if(watch.out_garage_flag==2&&get_integeral_state(&angle_integral)==2)
    {
        out_element();
    }
    else if(watch.out_garage_flag==4&&get_integeral_state(&angle_integral)==2)
    {
        out_element();
    }
}
void garage_prepare()
{
    if(get_integeral_state(&distance_integral)==0)
        begin_distant_integeral(setpara.garage_begin_distance);
   if(get_integeral_state(&distance_integral)==2)
   {
    clear_all_flags();
    enter_element(ingarage);    //正式进入车库元素
    set_speed(setpara.garage_target_speed);

    change_pid_para(&Speed_middle,&setpara.speed_down_PID);
   }

}
void left_garage_first_angle()
{
    if(watch.zebra_flag!=0&&watch.zebra_flag!=1)return;//在循环之前跳出，节省时间
    for(int y=25;y<100;y++)//逐行扫描
    {
        if(//lineinfo[y + 3].left_lost
            //&&lineinfo[y + 2].left_lost&&
            lineinfo[y + 1].left_lost
            && !lineinfo[y - 3].left_lost
            && !lineinfo[y - 2].left_lost
            && !lineinfo[y - 4].left_lost
            && !lineinfo[y].left_lost
            && !lineinfo[y+1].right_lost
            && !lineinfo[y].right_lost
            && !lineinfo[y-1].right_lost
            &&lineinfo[y].left-lineinfo[y+4].left>10
            &&y<watch.Zebra_Angle
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<87
            )
        {
            watch.Zebra_Angle = y;
            left_garage_confirm();
            break;
        }
    }
}

//左环二次确认函数
void left_garage_confirm()
{
    uint8_t zebra_confirm=0,white_count1=0,white_count2=0,white_count3=0,black_count,right_lost=0;
       for(int x=lineinfo[watch.Zebra_Angle-1].left;x>0;x--)
       {
           if(Grayscale[119-watch.Zebra_Angle][x]==255)
               white_count1++;
           if(Grayscale[119-(watch.Zebra_Angle-1)][x]==255)
               white_count2++;
           if(Grayscale[119-(watch.Zebra_Angle-2)][x]==255)
               white_count3++;
       }

       for(int y=watch.Zebra_Angle;y>25;y--)
       {
           if(Grayscale[119-y][lineinfo[watch.Zebra_Angle].left]==0)
              black_count++;
       }

       if((white_count1>=5&&white_count2>=5&&white_count3>=5)||(black_count<3))
       {
           //set_speed(setpara.garage_target_speed);
           set_speed(40);
           change_pid_para(&CAM_Turn,&setpara.garage_turn_PID);//将转向PID参数调为环内转向PID
           watch.zebra_flag=1;
           return;
       }
       watch.Zebra_Angle;
}
void left_garage_second_angle()
{
    if(watch.zebra_flag != 1&&watch.zebra_flag != 2)return;//在循环之前跳出，节省时间
    for(int y=25;y<100;y++)//逐行扫描
    {
        if (y<watch.Zebra_Angle2
             &&watch.zebra_flag==1
             &&watch.Zebra_Angle2==120
             &&lineinfo[watch.Zebra_Angle-1].left_lost
             //&&get_integeral_state(&distance_integral)==2
             &&y>watch.Zebra_Angle
           &&y > 60
           &&y < 100
          // &&lineinfo[watch.Zebra_Angle].left_lost
           &&lineinfo[y+1].left > 30
           &&(lineinfo[y+1].left-lineinfo[y].left)<=2
           &&(lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/2
           //&&lineinfo[y-1].zebra_flag==1
           )
           {
               watch.Zebra_Angle2 = y;
               watch.Zebra_Angle2_x=lineinfo[watch.Zebra_Angle2_x].left;
               break;
           }
    }
    if(watch.Zebra_Angle2!=120
        &&watch.Zebra_Angle2>40
        )
    {
        find_angle_left_down(&watch.Zebra_Angle2_x,&watch.Zebra_Angle2);
    }
}
void left_garage_begin_turn()
{
    if(watch.zebra_flag!=1)return;//在循环之前跳出，节省时间
    if(watch.zebra_flag==1
        //&&lineinfo[watch.Zebra_Angle].left_lost
        &&watch.Zebra_Angle2<=90
        )
    {
        //clear_distant_integeral();//清除路程积分变量
        begin_angle_integeral(70);
        watch.zebra_flag=2;
        //beep2(2,20);
    }
}
void left_garage_stop_turn()
{
    if(watch.zebra_flag==2&&get_integeral_state(&angle_integral)==2)
    {
        watch.zebra_flag=3;
        clear_angle_integeral();
        begin_distant_integeral(setpara.garage_stop_distance);
        //watch.angle_far_line=60;
    }
}
void garage_stop2()
{
    if((watch.zebra_flag==3||watch.zebra_flag==6)&&get_integeral_state(&distance_integral)==2)
    {
        car_stop();
    }
}

void right_garage_first_angle()
{
    if(watch.InLoop != 0)return;
    for(int y=25;y<100;y++)//逐行扫描
        {
        if (
          y < 86
         //&&lineinfo[y + 3].right_lost
         && !lineinfo[y - 3].right_lost
         //&&lineinfo[y + 2].right_lost
         && !lineinfo[y - 2].right_lost
         &&!lineinfo[y - 1].right_lost
         &&lineinfo[y + 1].right_lost
         &&!lineinfo[y].right_lost
         &&!lineinfo[y +1].left_lost
         &&!lineinfo[y + 3].left_lost
         &&!lineinfo[y + 2].left_lost
         &&!lineinfo[y + 1].left_lost
         &&!lineinfo[y - 3].left_lost
         &&!lineinfo[y - 2].left_lost
         &&!lineinfo[y - 1].left_lost && !lineinfo[y - 4].left_lost
         &&y<watch.Zebra_Angle
         &&lineinfo[y+4].right-lineinfo[y].right>10
    //     &&abs_m(lineinfo[y - 5].right,lineinfo[y - 4].right)<8
    //     &&abs_m(lineinfo[y - 6].right,lineinfo[y - 5].right)<8
         )
            { //左圆环的第一个角点所在行
                watch.Zebra_Angle = y;
                   //Element=Rifht_ring_confirm;
                right_garage_confirm();       //进入右环二次确认
            }
        }
}
void right_garage_confirm()
{
    uint8_t zebra_confirm,white_count1=0,white_count2=0,white_count3=0,left_lost=0,black_count=0;
        //right_ring_circular_arc();  //扫描是否存在右环上弧
        //left_ring_first_angle();   //扫描是否存在左环角点

    for(int x=lineinfo[watch.Zebra_Angle-1].right;x<188;x++)
    {
        if(Grayscale[119-watch.Zebra_Angle][x]==255)
            white_count1++;
        if(Grayscale[119-watch.Zebra_Angle-1][x]==255)
            white_count2++;
        if(Grayscale[119-watch.Zebra_Angle-2][x]==255)
            white_count3++;
    }
    for(int y=watch.Zebra_Angle;y>25;y--)
    {
        if(Grayscale[119-y][lineinfo[watch.Zebra_Angle].right]==0)
           black_count++;
    }
    if((white_count1>=5&&white_count2>=5&&white_count3>=5)||black_count<3)
    {
        //Element=Right_ring;        //正式进入右环元素
        change_pid_para(&CAM_Turn,&setpara.garage_turn_PID);//将转向PID参数调为环内转向PID
        watch.zebra_flag = 4;
        return;
    }
}
void right_garage_second_angle()
{
    if(watch.zebra_flag != 4&&watch.zebra_flag != 5)return;//在循环之前跳出，节省时间
    for(int y=25;y<100;y++)//逐行扫描
    {
        if (y<watch.Zebra_Angle2
             &&watch.Zebra_Angle2==120
             &&lineinfo[watch.Zebra_Angle-1].right_lost
             //&&get_integeral_state(&distance_integral)==2
             &&y>watch.Zebra_Angle
           &&y > 60
           &&y < 100
          // &&lineinfo[watch.Zebra_Angle].left_lost
           &&lineinfo[y+1].right < 158
           &&(lineinfo[y].right-lineinfo[y+1].right)<=2
           &&(lineinfo[y-4].right-lineinfo[y].right)>(188-lineinfo[y].left)/2
           //&&lineinfo[y-1].zebra_flag==1
           )
           {
               watch.Zebra_Angle2 = y;
               watch.Zebra_Angle2_x=lineinfo[watch.Zebra_Angle2].right;
               break;
           }
    }
    if(watch.Zebra_Angle2!=120
        &&watch.Zebra_Angle2>40
        )
    {
        find_angle_right_down(&watch.Zebra_Angle2_x,&watch.Zebra_Angle2);
    }
}
void right_garage_begin_turn()
{
    if(watch.zebra_flag==4
        //&&lineinfo[watch.Zebra_Angle].left_lost
        &&watch.Zebra_Angle2<=90
        )
    {
        //clear_distant_integeral();//清除路程积分变量
        begin_angle_integeral(-70);
        watch.zebra_flag=5;
        //beep2(2,20);
    }
}
void right_garage_stop_turn()
{
    if(watch.zebra_flag==5&&get_integeral_state(&angle_integral)==2)
    {
        watch.zebra_flag=6;
        clear_angle_integeral();
        begin_distant_integeral(setpara.garage_stop_distance);
        //watch.angle_far_line=60;
    }
}
