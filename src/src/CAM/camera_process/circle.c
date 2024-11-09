/*
 * circle.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "circle.h"

#define loop_forward_far 100
#define loop_forward_near 20
//检测左环第一个角点(Inloop=1)
void left_ring_first_angle()
{
    if(watch.InLoop!=0&&watch.InLoop!=1)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if(//lineinfo[y + 3].left_lost
            //&&lineinfo[y + 2].left_lost&&
            ((lineinfo[y + 1].left_lost)||((lineinfo[y].left-lineinfo[y+1].left)>=5*(lineinfo[y+1].left-lineinfo[y+2].left)))///////////
            && !lineinfo[y - 3].left_lost
            && !lineinfo[y - 2].left_lost
            && !lineinfo[y - 1].left_lost
            && !lineinfo[y].left_lost
            && !lineinfo[y + 5].right_lost
            && !lineinfo[y + 3].right_lost
            && !lineinfo[y + 2].right_lost
            && !lineinfo[y + 1].right_lost
            && !lineinfo[y].right_lost
            && !lineinfo[y - 1].right_lost
            && !lineinfo[y - 2].right_lost
            && !lineinfo[y - 3].right_lost
            && !lineinfo[y - 4].right_lost
            && !lineinfo[y - 5].right_lost

            &&lineinfo[y].left-lineinfo[y+4].left>10
            &&y<watch.InLoopAngleL
            //&&lineinfo[y].left>=lineinfo[y-2].left
            &&y<75
            )
        {//左圆环的第一个角点所在行
            watch.InLoopAngleL = y;
            if(Element==None)     //在当前无元素时进行以下操作，其他时候只找角点
            {
                   left_ring_confirm();
            }
            break;
        }
    }
}
//左环二次确认函数
void left_ring_confirm()
{
    uint8_t zebra_confirm=0,white_count1=0,white_count2=0,white_count3=0,black_count=0,right_lost=0;
    //right_ring_first_angle();//扫描是否存在右环角点
    //left_ring_circular_arc();//扫描是否存在左环上弧
    for(int y=loop_forward_near;y<95;y++)//逐行扫描
    {
        if(((lineinfo[y+2].right-lineinfo[y].right)>2)||(lineinfo[y].right-lineinfo[y+2].right)>4)
            right_lost+=1;
//        if(lineinfo[y].right-lineinfo[y+2].right>20
//           &&lineinfo[y-1].right-lineinfo[y+3].right>20
//           &&lineinfo[y-2].right-lineinfo[y+4].right>20)
//            right_lost+=3;
    }
       if(right_lost<3)
       {
//           for(int x=lineinfo[watch.InLoopAngleL-1].left;x>0;x--)
//           {
//               if(Grayscale[119-watch.InLoopAngleL][x]==255)
//                   white_count1++;
//               if(Grayscale[119-(watch.InLoopAngleL-1)][x]==255)
//                   white_count2++;
//               if(Grayscale[119-(watch.InLoopAngleL-2)][x]==255)
//                   white_count3++;
//           }
           //vofa.loop[5]=white_count1;
           //vofa.loop[6]=white_count2;
           //vofa.loop[7]=white_count3;
           for(int y=watch.InLoopAngleL;y>loop_forward_near;y--)
           {
               if(Grayscale[119-y][lineinfo[watch.InLoopAngleL].left]==0)
                  black_count++;
           }

           if(/*(white_count1>=10&&white_count2>=10&&white_count3>=10)&&*/(black_count<10))
           {
               enter_element(Left_ring);    //正式进入左圆环元素
               begin_distant_integeral(6000);
              // if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
               {
                   set_speed(setpara.loop_target_speed);
                   mycar.pid_ctrl=0;
                   change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
               }
/*               else//如果是大环
               {
                   set_speed(setpara.big_loop_speed);
                   //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
               }*/
               watch.InLoop = 1;
            //    beep2(1,20);//蜂鸣器
               return;
           }
       }

    Element=None;
    watch.InLoopAngleR=120;
    watch.InLoopAngleL=120;
}


//检测左环入环第一个角点后的圆弧
void left_ring_circular_arc()
{
    if (watch.InLoop != 1&&watch.InLoop != 2)return;//在循环之前跳出，节省时间
    //beep(20);
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if (//y <watch.InLoopAngle2  &&
               (watch.InLoopAngleL<65||get_integeral_state(&distance_integral)==2)
           &&(y>(watch.InLoopAngleL+20)||get_integeral_state(&distance_integral)==2)
           &&y <watch.InLoopCirc
           &&!lineinfo[y+3].left_lost
           &&!lineinfo[y+2].left_lost
           &&!lineinfo[y+1].left_lost
           &&!lineinfo[y-3].left_lost
           &&!lineinfo[y-2].left_lost
           &&!lineinfo[y-1].left_lost
           &&lineinfo[y+1].left <= lineinfo[y].left
           &&lineinfo[y+2].left <= lineinfo[y].left
           &&lineinfo[y+3].left <= lineinfo[y].left
           &&lineinfo[y-1].left <= lineinfo[y].left
           &&lineinfo[y-2].left <= lineinfo[y].left
           &&lineinfo[y-3].left <= lineinfo[y].left
           //&&(watch.right_lost+watch.cross_lost)<5
            )
       { //入环点所在行
            watch.InLoopCirc = y;
            //beep(20);
            break;
       }
    }
}
//检测左环第二个角点
void left_ring_second_angle()
{
    if(watch.InLoop != 1&&watch.InLoop != 2)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
        if (watch.InLoopCirc<66
             &&y<watch.InLoopAngle2
             &&watch.InLoopAngle2==120
             &&get_integeral_state(&distance_integral)==2
           &&y > 60
           &&y < (loop_forward_far-2)
           &&y>watch.InLoopCirc
           &&lineinfo[y+1].left > 30
           &&(lineinfo[y+1].left-lineinfo[y].left)<=2
           &&(lineinfo[y].left-lineinfo[y-4].left)>lineinfo[y].left/2
           )
           {
               watch.InLoopAngle2 = y;
               watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].left;
               //if()
               //watch.InLoopCirc=0;
               break;
           }
    }
    if(watch.InLoopAngle2!=120
        &&watch.InLoopAngle2>50
        )
    {
        find_angle_left_down(&watch.InLoopAngle2_x,&watch.InLoopAngle2);
    }
}
//左环开始转向函数(watch.InLoop=2)
void left_ring_begin_turn()
{
    if(watch.InLoop!=1)return;//在循环之前跳出，节省时间
    if(get_integeral_state(&distance_integral)==2//路程积分完成
        &&watch.InLoop==1
        &&watch.InLoopAngle2<=80
    )
    {
        clear_distant_integeral();//清除路程积分变量
        watch.InLoop=2;
        //set_speed(setpara.loop_target_speed);
        //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
        //watch.fix_slope=(float)(lineinfo[watch.InLoopAngle2].left)/(115-watch.InLoopAngle2);
        begin_angle_integeral(260);
        // beep2(2,20);
    }
}
//检验小车是否完全入环
void left_ring_in_loop()
{
    if(watch.InLoop != 2)return;
    if( watch.InLoop == 2
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)>40//转过一定角度
        //&&watch.right_near_lost<40
        //&&lineinfo[watch.right_near_lost].right>180
        //&&(!lineinfo[40].right_lost)
        )
        {
            //watch.InLoopCirc = 0;
            watch.InLoop = 3;
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
            set_speed(setpara.loop_target_speed+3);
            // beep2(3,20);
        }
}
//小车角度积分完成，准备出环
void left_ring_prepare_out()
{
    if(watch.InLoop != 3)return;
    if( watch.InLoop == 3
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)>160
        &&lineinfo[69].right<152
        &&lineinfo[69].right>82)
   {
       watch.InLoop = 4;
       watch.OutLoop_turn_point_x=lineinfo[69].right;
    //    beep2(4,20);
   }
}
//检测出环时右角点位置
void left_ring_out_angle()
{
    if(watch.InLoop != 4)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if ((watch.InLoop == 4)&&y<80
                 //lineinfo[y].left_lost
                 &&lineinfo[y+1].right >= lineinfo[y].right
                 &&lineinfo[y+2].right >= lineinfo[y+1].right
                 &&lineinfo[y-1].right >= lineinfo[y].right
                 &&lineinfo[y-2].right >= lineinfo[y].right
/*                 &&lineinfo[y - 3].right > lineinfo[y - 1].right
                 &&lineinfo[y + 4].right > lineinfo[y + 2].right
                 &&lineinfo[y - 5].right > lineinfo[y - 3].right*/
                 &&lineinfo[y].right > 30
                 &&Grayscale[119-y-2][lineinfo[y].right]==255
)
             {
                 if(watch.OutLoopAngle1>y)
                 {
                     //watch.OutLoopRight = lineinfo[y].right;
                     watch.OutLoopAngle1 = y; //出环判断列
                     break;
                 }
             }
        }
}
//出左环右转(进入圆环与直线交界处)
void left_ring_out_loop_turn()
{
    if(watch.InLoop != 4)return;
    if(watch.InLoop == 4
       //&&(watch.cross+watch.right_lost)>30//右侧丢线过多，说明进入圆环与直线交界处
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2
       &&lineinfo[watch.OutLoopAngle1].right_lost==1
       &&watch.OutLoop==0
    )
    {
        clear_angle_integeral();
//        if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//        {
//            begin_angle_integeral(setpara.loop_angle_out);
//        }
//        else//如果是大环
//        {
//            begin_angle_integeral(setpara.big_loop_out);
//        }
        begin_distant_integeral(3000);//开启路程积分，此时要保持左转
        watch.OutLoop=1;
        // beep2(5,20);
    }
}
//右侧为直线时直行
void left_ring_out_loop()
{
    if(watch.InLoop != 4&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop == 4
            &&watch.OutLoop==1
            &&y<60
            &&lineinfo[y].right_lost==0
            &&lineinfo[y+10].right_lost==0
            &&lineinfo[y].right>10///////////////////
            &&lineinfo[y+10].right>10/////////////////
            &&(188-(lineinfo[y].right-lineinfo[y+10].right)*(115-y)/10)>40
            &&lineinfo[y].right>lineinfo[y+10].right
            &&get_integeral_state(&distance_integral)==2)
                )
            {
            clear_distant_integeral();
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//            {
                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//如果是大环
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
            clear_angle_integeral();
            // beep2(6,20);
            watch.InLoop =5;////不用陀螺仪，用摄像头自身提取赛道元素；
            }
        }
}
//检测出左环进入直线后左侧角点
void left_ring_straight_out_angle()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop==5)
          &&watch.OutLoop==1
          &&y<(watch.watch_lost-10)
          &&y<80
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120
          &&lineinfo[y].right>60
          && watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+1].right-lineinfo[y+2].right<5
          &&!lineinfo[y].left_lost
          &&!lineinfo[y+1].left_lost
          &&!lineinfo[y+2].left_lost)///////////
            {
                watch.OutLoopAngle2 = y;
            }
        }
}

//检测完全出环
void left_ring_complete_out()
{
    if(watch.InLoop != 5&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
    {
    if (watch.InLoop == 5
        &&get_integeral_state(&distance_integral)==2
        &&watch.OutLoopAngle2<70
        )
     {
         clear_all_flags();// 出环成功,清除所有标志
         out_element();
         //mycar.target_speed=setpara.com_target_speed;//恢复正常速度
         //change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复正常转向PID
        //  beep2(7,20);
     }
    }
}

//以下为右环代码

//检测右环第一个角点(Inloop=1)
//返回值：1：找到角点 0：无角点
void right_ring_first_angle()
{
    if(watch.InLoop != 0&&watch.InLoop!=6)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if (
          y < 75
         &&((lineinfo[y + 1].right_lost)||(lineinfo[y+1].right-lineinfo[y].right>=5*(lineinfo[y+2].right-lineinfo[y+1].right)))
         && !lineinfo[y - 3].right_lost
         && !lineinfo[y - 2].right_lost
         &&!lineinfo[y - 1].right_lost
         &&!lineinfo[y].right_lost
         &&!lineinfo[y + 5].left_lost
         &&!lineinfo[y + 3].left_lost
         &&!lineinfo[y + 2].left_lost
         &&!lineinfo[y + 1].left_lost
         &&!lineinfo[y].left_lost
         &&!lineinfo[y - 5].left_lost
         &&!lineinfo[y - 4].left_lost
         &&!lineinfo[y - 3].left_lost
         &&!lineinfo[y - 2].left_lost
         &&!lineinfo[y - 1].left_lost
         &&y<watch.InLoopAngleR
         &&lineinfo[y+4].right-lineinfo[y].right>10
    //     &&abs_m(lineinfo[y - 5].right,lineinfo[y - 4].right)<8
    //     &&abs_m(lineinfo[y - 6].right,lineinfo[y - 5].right)<8
         )
            { //左圆环的第一个角点所在行
                watch.InLoopAngleR = y;
                   //Element=Rifht_ring_confirm;
                if(Element==None)     //在当前无元素时进行以下操作，其他时候只找角点
                {
                    right_ring_confirm();       //进入右环二次确认
                }
            }
        }
}
//右环二次确认函数
void right_ring_confirm()
{
    uint8_t zebra_confirm,white_count1=0,white_count2=0,white_count3=0,left_lost=0,black_count=0;
        //right_ring_circular_arc();  //扫描是否存在右环上弧
        //left_ring_first_angle();   //扫描是否存在左环角点
        for(int y=loop_forward_near;y<95;y++)//逐行扫描
        {
            if((lineinfo[y].left-lineinfo[y+2].left>2)||(lineinfo[y+2].left-lineinfo[y].left>4))
                left_lost+=1;
//            if(lineinfo[y+2].left-lineinfo[y].left>20
//              &&lineinfo[y+3].left-lineinfo[y-1].left>20
//              &&lineinfo[y+4].left-lineinfo[y-2].left>20)
//               left_lost+=3;
        }
        if(left_lost<3)
        {
            {
//                for(int x=lineinfo[watch.InLoopAngleR-1].right;x<188;x++)
//                {
//                    if(Grayscale[119-watch.InLoopAngleR][x]==255)
//                        white_count1++;
//                    if(Grayscale[119-watch.InLoopAngleR-1][x]==255)
//                        white_count2++;
//                    if(Grayscale[119-watch.InLoopAngleR-2][x]==255)
//                        white_count3++;
//                }
                for(int y=watch.InLoopAngleR;y>loop_forward_near;y--)
                {
                    if(Grayscale[119-y][lineinfo[watch.InLoopAngleR].right]==0)
                       black_count++;
                }
                if(/*(white_count1>=10&&white_count2>=10&&white_count3>=10)&&*/black_count<10)
                {
                    //Element=Right_ring;        //正式进入右环元素
                    enter_element(Right_ring);
                    if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
                    {
                        set_speed(setpara.loop_target_speed);
                        mycar.pid_ctrl=0;
                        change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
                    }
//                    else//如果是大环
//                    {
//                        set_speed(setpara.big_loop_speed);
//                        //change_pid_para(&CAM_Turn,&setpara.big_loop_PID);
//                    }
                    begin_distant_integeral(6000);
                    watch.InLoop = 6;
                    // beep2(1,20);//蜂鸣器
                    return;
                }

            }
        }
        Element=None;
        watch.InLoopAngleR=120;
        watch.InLoopAngleL=120;
        return;

}
//右环开始转向函数(watch.InLoop=2)
void right_ring_begin_turn()
{
    if(watch.InLoop != 6)return;
    if(get_integeral_state(&distance_integral)==2//路程积分完成
        &&watch.InLoop==6
        &&(watch.InLoopAngle2<=80)
        )
        {
                clear_distant_integeral();//清除路程积分变量
                //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
                watch.InLoop=7;
                //set_speed(setpara.loop_target_speed);
                //watch.fix_slope=(float)(188-lineinfo[watch.InLoopAngle2].right)/(115-watch.InLoopAngle2);
                begin_angle_integeral(-260);
                // beep2(2,20);
        }

}
/*
if ((watch.InLoop == 1)
   &&y<watch.watch_lost-5
   &&y <watch.InLoopAngle2 &&y>watch.InLoopAngle
   &&y <watch.InLoopCirc
   &&!lineinfo[y+3].left_lost
   &&!lineinfo[y+2].left_lost
   &&!lineinfo[y+1].left_lost
   &&!lineinfo[y-3].left_lost
   &&!lineinfo[y-2].left_lost
   &&!lineinfo[y-1].left_lost
   &&lineinfo[y+1].left <= lineinfo[y].left
   &&lineinfo[y+2].left <= lineinfo[y+1].left
   &&lineinfo[y-1].left <= lineinfo[y].left
   &&lineinfo[y-2].left <= lineinfo[y-1].left*/
//检测右环入环第一个角点后的圆弧
void right_ring_circular_arc()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if (//(watch.InLoop == 6)&&y <watch.InLoopAngle2 &&
             (watch.InLoopAngleR<65||get_integeral_state(&distance_integral)==2)
         &&(y>(watch.InLoopAngleR+20)||get_integeral_state(&distance_integral)==2)
           &&y <watch.InLoopCirc
           &&!lineinfo[y+3].right_lost
           &&!lineinfo[y+2].right_lost
           &&!lineinfo[y+1].right_lost
           &&!lineinfo[y-3].right_lost
           &&!lineinfo[y-2].right_lost
           &&!lineinfo[y-1].right_lost
           &&lineinfo[y+1].right >= lineinfo[y].right
           &&lineinfo[y+2].right >= lineinfo[y+1].right
           &&lineinfo[y+3].right >= lineinfo[y].right

           &&lineinfo[y-1].right >= lineinfo[y].right
           &&lineinfo[y-2].right >= lineinfo[y-1].right
           &&lineinfo[y-3].right >= lineinfo[y].right

           )
           { //入环点所在行
                watch.InLoopCirc = y;
                break;
           }
        }
}

/*
if(watch.InLoopCirc<120
       &&y<watch.InLoopAngle2
       &&get_integeral_state(&distance_integral)==2
       &&y>watch.InLoopCirc
       &&lineinfo[y-1].left_lost&&!lineinfo[y].left_lost
       &&lineinfo[y].edge_store[1]>50
       &&lineinfo[y].edge_store[0]==0
       &&lineinfo[y+1].edge_store[1]<=lineinfo[y].edge_store[1]
       &&lineinfo[y+1].edge_store[2]>=lineinfo[y].edge_store[1])*/
//检测右环第二个角点
void right_ring_second_angle()
{
    if(watch.InLoop != 6&&watch.InLoop != 7)return;//在循环之前跳出，节省时间
    for(int y=loop_forward_near;y<loop_forward_far;y++)//最靠下的点，近距离时较为靠谱（第一判据）
    {
        if(watch.InLoopCirc<66
               &&y<watch.InLoopAngle2
               &&watch.InLoopAngle2==120
               &&get_integeral_state(&distance_integral)==2
               &&y > 60
               &&y < (loop_forward_far-2)
               &&y>watch.InLoopCirc
                        &&lineinfo[y+1].right <158
                        &&(lineinfo[y].right-lineinfo[y+1].right)<=2
                        &&(lineinfo[y-4].right-lineinfo[y].right)>(187-lineinfo[y].right)/2
                        )
                        {
                            watch.InLoopAngle2 = y;
                            watch.InLoopAngle2_x=lineinfo[watch.InLoopAngle2].right;
                            //if()
                            //watch.InLoopCirc=0;
                            break;
                        }
                 }
                 if(watch.InLoopAngle2!=120
                     &&watch.InLoopAngle2>50
                     )
                 {
                     find_angle_right_down(&watch.InLoopAngle2_x,&watch.InLoopAngle2);
                 }

//               &&lineinfo[y-1].right_lost&&!lineinfo[y].right_lost
//               &&lineinfo[y].edge_store[lineinfo[y].edge_count-2]<130
//               &&lineinfo[y].edge_store[lineinfo[y].edge_count-1]==187
//               &&lineinfo[y+1].edge_store[lineinfo[y+1].edge_count-2]>=lineinfo[y].edge_store[lineinfo[y].edge_count-2]
//               &&lineinfo[y+1].edge_store[lineinfo[y].edge_count-3]<=lineinfo[y].edge_store[lineinfo[y].edge_count-2])
//           )
//               {
//            watch.InLoopAngle2 = y;
//            watch.InLoopAngle2_x=lineinfo[y].edge_store[lineinfo[y].edge_count-3];
//            return;
//        }
//    }//检测左边线跳变沿，远距离时靠谱（第二判据）
//    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
//        {
//        if (watch.InLoopCirc<120
//             &&y<watch.InLoopAngle2
//             &&watch.InLoopAngle2==120
//             &&get_integeral_state(&distance_integral)==2
//             &&y > 60
//             &&lineinfo[watch.InLoopCirc].right_lost
//             &&y<watch.InLoopAngle2
//             &&y>watch.InLoopCirc
//             &&(lineinfo[y].right-lineinfo[y+1].right)<=2
//             &&(lineinfo[y-4].right-lineinfo[y].right)>(188-lineinfo[y].right)/2
//            )
//           {
//               watch.InLoopAngle2 = y;
//               watch.InLoopAngle2_x=lineinfo[y].right;
//               break;
//           }
//        }
//    if(watch.InLoopAngle2<120
//            &&watch.InLoopAngle2>50)
//    {
//        find_angle_right_down(watch.InLoopAngle2_x,watch.InLoopAngle2,&watch.InLoopAngle2_x,&watch.InLoopAngle2);
//    }
}

//检验小车是否完全入右环
void right_ring_in_loop()
{
    if(watch.InLoop != 7)return;

    if( watch.InLoop == 7
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-40//转过一定角度
        //&&watch.left_near_lost<40
        )
        {
            watch.InLoop = 8;
            //change_pid_para(&CAM_Turn,&setpara.loop_turn_PID);//将转向PID参数调为环内转向PID
            set_speed(setpara.loop_target_speed+3);
            // beep2(3,20);
        }
}

//小车角度积分完成，准备出环
void right_ring_prepare_out()
{
    if(watch.InLoop != 8)return;
    if( watch.InLoop == 8
        &&get_integeral_state(&angle_integral)==1
        &&get_integeral_data(&angle_integral)<-160
        &&lineinfo[69].left>35
        &&lineinfo[69].left<105)///////////////////////
   {
       watch.InLoop = 9;
       watch.OutLoop_turn_point_x=lineinfo[69].left;
    //    beep2(4,20);
   }
}
//

//检测出环时左角点位置
void right_ring_out_angle()
{
    if(watch.InLoop != 9)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if ((watch.InLoop == 9) &&y<80
                 //lineinfo[y].left_lost
                 &&lineinfo[y + 1].left <= lineinfo[y].left
                 &&lineinfo[y+2].left <= lineinfo[y+1].left
                 &&lineinfo[y-1].left <= lineinfo[y].left//
                 &&lineinfo[y-2].left <= lineinfo[y].left
                 &&lineinfo[y].left < 158
                 &&Grayscale[119-y-2][lineinfo[y].left]==255
                 &&y>30
                 )
             {
                 if(watch.OutLoopAngle1>y)
                 {
                 //watch.OutLoopLeft = lineinfo[y].left;
                     watch.OutLoopAngle1 = y; //出环判断列
                 }
             }
        }
}
//出右环右转
void right_ring_out_loop_turn()
{
    if(watch.InLoop == 9
       //&&(watch.cross+watch.left_lost)>30//右侧丢线过多，说明进入圆环与直线交界处
       &&watch.OutLoopAngle1<120
       &&get_integeral_state(&angle_integral)==2
       &&lineinfo[watch.OutLoopAngle1].left_lost==1
       &&watch.OutLoop==0)
    {
        if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
        {
            begin_distant_integeral(setpara.loop_out_distance);
        }
        else//如果是大环
        {
            begin_distant_integeral(setpara.big_loop_out_distance);
        }
        clear_angle_integeral();
        begin_distant_integeral(3000);//开启路程积分，此时要保持左转
        watch.OutLoop=1;
        // beep2(5,20);
    }
}

//左侧为直线时直行
void right_ring_out_loop()
{
    if(watch.InLoop != 9&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop == 9
            &&watch.OutLoop==1
            &&y<60
            &&lineinfo[y].left_lost==0
            &&lineinfo[y+10].left_lost==0
            &&lineinfo[y].left>10///////////////////
            &&lineinfo[y+10].left>10/////////////////
            &&((lineinfo[y+10].left-lineinfo[y].left)*(115-y)/10)<148
            &&lineinfo[y+10].left-lineinfo[y].left>0////////////////
            &&get_integeral_state(&distance_integral)==2)
                //||get_integeral_state(&angle_integral)==2
                )
            {
            clear_distant_integeral();
//            if(Element_rem.loop_data[Element_rem.loop_count]==0)//如果是小环
//            {
                begin_distant_integeral(setpara.loop_out_distance);
//            }
//            else//如果是大环
//            {
//                begin_distant_integeral(setpara.big_loop_out_distance);
//            }
            // beep2(6,20);
            watch.InLoop =10;////不用陀螺仪，用摄像头自身提取赛道元素；
            }
        }
}

//检测出右环进入直线后左侧角点
void right_ring_straight_out_angle()
{
    if(watch.InLoop != 10&&watch.OutLoop!=1)return;
    for(int y=loop_forward_near;y<loop_forward_far;y++)//逐行扫描
        {
        if((watch.InLoop==10)
          &&watch.OutLoop==1
          &&y<(watch.watch_lost-10)
          &&y<80
          &&y<watch.OutLoopAngle2
          &&lineinfo[y].left<120
          &&lineinfo[y].right>60
          && watch.zebra_flag == 0
          &&(lineinfo[y + 2].right-lineinfo[y + 2].left)<(lineinfo[y -1].right-lineinfo[y -1].left)-30
          &&(lineinfo[y + 1].right-lineinfo[y + 1].left)<(lineinfo[y -2].right-lineinfo[y -2].left)-30
          &&(lineinfo[y].right-lineinfo[y].left)<(lineinfo[y -3].right-lineinfo[y -3].left)-30
          &&lineinfo[y+2].left-lineinfo[y+1].left<5///////////////////
          &&!lineinfo[y].right_lost
          &&!lineinfo[y+1].right_lost
          &&!lineinfo[y+2].right_lost)
            {
                watch.OutLoopAngle2 = y;
            }
        }
}

//检测完全出环
void right_ring_complete_out()
{
    if (watch.InLoop == 10
        &&get_integeral_state(&distance_integral)==2
        &&watch.OutLoopAngle2<70
        )
     {
         //clear_all_flags();// 出环成功,清除所有标志
         out_element();
         //mycar.target_speed=setpara.com_target_speed;//恢复正常速度
         //change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复正常转向PID
        //  beep2(7,100);
     }
}
//向左下找角点
void find_angle_left_down(int*angle_x,int*angle_y)
{
    int x=*angle_x, y=*angle_y;
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x+1]!=255&&x<187)
    {
        x++;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}
//向右下找角点
void find_angle_right_down(int*angle_x,int*angle_y)
{
    int x=*angle_x, y=*angle_y;
    while(Grayscale[119-y][x]!=0&&y<110)
    {
        y++;
    }
    while(Grayscale[119-y][x-1]!=255&&x>0)
    {
        x--;
    }
    while(y>40)
    {
        if(Grayscale[119-y][x]==0)
        {

        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x=x+3;
        }
        else if(Grayscale[119-y][x-3]==0)
        {
            x=x-3;
        }
        else break;
        y--;
    }
    *angle_x=x;
    *angle_y=y;
}

