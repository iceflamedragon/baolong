/*
 * obstacle.c
 *
 *  Created on: 2023年7月5日
 *      Author: HUAWEI
 */
//避障思路：1.tof检测到障碍物距离小于0.6m时识别为路障（此处可有防误判条件）
//2.函数1：补线，左线为0，右线从左上向右下拉线；同时开启角度积分
//3.函数2：角度积分约30度后停止积分；此时补左右边线为竖直线，开启路程积分
//4.转向积分停止，进行路程积分；积分50cm左右（6000）
//5.关闭积分，开启第三段补线右转，并开启角度积分
//6.角度积分停止，小车此时应回到赛道中央恢复巡线模式
//吴双:
#include "obstacle.h"
//断路准备函数

//1.tof检测距离0.8m识别为路障
void obstacle_identification()
{
    // if(dl1b_distance_mm>0
    //    &&dl1b_distance_mm <= (setpara.obstacle_detect_distance)
    //    )
    // {
    //     set_speed(0);
    //     change_pid_para(&Speed_left,&setpara.stop_PID);
    //     change_pid_para(&Speed_right,&setpara.stop_PID);
    //     enter_element(obstacle);
    // }
}
void obstacle_stop()
{
    // if(mycar.present_speed==0)
    // {
    //     out_element();
    //     car_stop();
    // }
}




