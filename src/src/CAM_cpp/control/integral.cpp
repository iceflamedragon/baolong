/*
 * integeral.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "integral.hpp"
integeral_STRUCT angle_integral,//角度积分结构体
                distance_integral;//路程积分结构体
//开始角度积分，入口参数为目标角度
void begin_angle_integeral(int16_t target_angle)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_Thres=target_angle;
    angle_integral.integeral_flag=1;
}
//清除角度积分变量
void clear_angle_integeral(void)
{
    angle_integral.integeral_data=0;
    angle_integral.integeral_flag=0;
}
//角度积分函数，放在读取主函数角度积分
void angal_integeral(float mpu_now)
{
    static float mpu_first;//一开始的值
    static int count=0;
    if(angle_integral.integeral_flag==1)
    {   
        
        if(count==0)
       { 
        mpu_first=mpu_now;//找到初值
        count=1;
       }
        angle_integral.integeral_data=mpu_now-mpu_first;
        if(angle_integral.integeral_Thres>0)
        {
            if(angle_integral.integeral_data>angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //angle_integral.integeral_data=0;
                mpu_first=0;
                count=0;
            }
        }
        else
        {
            if(angle_integral.integeral_data<angle_integral.integeral_Thres)
            {
                angle_integral.integeral_flag=2;
                //indata.YawAngle=0;
                mpu_first=0;
                count=0;
            }
        }
    }
}
//开始路程积分函数，入口参数为目标积分距离
void begin_distant_integeral(int target_distant)
{
    distance_integral.integeral_flag=1;
    distance_integral.integeral_data=0;
    distance_integral.integeral_Thres=target_distant;
}
//清除路程积分变量
void clear_distant_integeral(void)
{
    distance_integral.integeral_flag=0;
    distance_integral.integeral_data=0;
}
//路程积分函数，放在读取主函数路程积分
void distant_integeral(float distance_now)  //路程积分
{
    static float distance_first;
    static int count=0;
    if(distance_integral.integeral_flag==1)//进行积分
    {
        if (count==0)
        { 
        distance_first=distance_now;//找到初值
        count=1;
        }
        distance_integral.integeral_data=distance_now-distance_first; //把现在的路程积分传入
        if(distance_integral.integeral_data>distance_integral.integeral_Thres)//达到设定路程，停止积分
        {
        //distance_integral.integeral_data=0;
         distance_integral.integeral_flag=2;
        distance_first=0;//找到初值
        count=0;
        }
    }distance_first=distance_now;
}
//获取积分状况
uint8_t get_integeral_state(integeral_STRUCT* integeral)
{
    return integeral->integeral_flag;
}
//获取积分数据
float get_integeral_data(integeral_STRUCT* integeral)
{
    return integeral->integeral_data;
}

