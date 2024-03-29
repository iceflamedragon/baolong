/*
 * tof.c
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */
#include "tof.h"
//��ȡ���룬��λmm
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
//�ж�ǰ���Ƿ����ϰ���  1����  0��û��
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

