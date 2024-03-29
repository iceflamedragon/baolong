/*
 * tof.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "tof.h"
//获取距离，单位mm
uint16 get_distance_mm()
{
    static uint16 distance;
    dl1a_get_distance();
    if(dl1a_finsh_flag == 1)
    {
        distance=dl1a_distance_mm;
        dl1a_finsh_flag = 0;
    }
    return distance;
}
//判断前方是否有障碍物  1：有  0：没有
uint8 avoid_obstacle(uint16 target_distance)
{
    uint16 distance,times;
    distance=get_distance_mm();
    if(distance<target_distance)
    {
        times++;
        if(times>=5)
            return 1;
    }
    else if(distance>target_distance)
    {
        times=0;
    }
    return 0;
}

