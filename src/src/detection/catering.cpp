#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2025; SaiShu.Lcc.; HC; https://bjsstech.com
 *                                   版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial transactions(开源学习,请勿商用).
 *            The code ADAPTS the corresponding hardware circuit board(代码适配百度Edgeboard-智能汽车赛事版),
 *            The specific details consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file catering.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 餐饮区
 * @version 0.1
 * @date 2025/03/03 09:53:17
 * @copyright  :Copyright (c) 2024
 *
 */

#include <fstream>
#include <iostream>
#include <cmath>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"
#include <chrono>
#include "../motion.cpp"

using namespace cv;
using namespace std;

class Catering
{
public:
    uint16_t counterSession = 0;    // 图像场次计数器
    uint16_t counterRec = 0;        // 汉堡标志检测计数器   
    bool stopped=false;             // 停车完成标志
    int stop_time=0;                // 记录停车瞬间的mycar.RUNTIME
    bool cateringEnable = false;    // 岔路区域使能标志
    bool burgerLeft = true;         // 汉堡在左侧
    int burgerY = 0;                // 汉堡高度
    int state=0;                    // 岔路状态
    bool found=0;                   // 找到角点
    
    //汉堡补线函数//
    void catering_linefix(){
        int16_t xl,xr;
        for (int y =forward_near; y <=watch.watch_lost; y++)
        {   
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
            if(burgerLeft)
            {
                xl=y/0.8;
                xr=lineinfo[y].right;
            }
            else if(!burgerLeft)
            {
                xr=187-y/0.8;
                xl=lineinfo[y].left;
            }
            cout<<"汉堡补线开始啦"<<endl;               
            if (burgerLeft)
            {
                persp_task(xl,lineinfo[y].right,y);//对补线结果进行逆透视变换和存储
            }
                           
            else
            {   
                persp_task(lineinfo[y].left,xr,y);//对补线结果进行逆透视变换和存储
            }    
        }  

    }

    bool process(Mat &image, vector<PredictResult> predict)
    {   
        if(!cateringEnable&&state==0) // 开始检测汉堡标志
        {   
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BURGER)
                {
                    burgerY = predict[i].y;   
                }
            }// 计算汉堡最高高度
            for (size_t i = 0; i < predict.size(); i++)
            {
                if (predict[i].type == LABEL_BURGER && predict[i].score > 0.4 && (predict[i].y ) < ROWSIMAGE * 0.75)
                {
                    counterRec++;
                    if (predict[i].x < COLSIMAGE / 2)   // 汉堡在左侧
                        burgerLeft = true;
                    else
                        burgerLeft = false;
                    break;
                }
            }
            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 3 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    cateringEnable = true; // 检测到汉堡标志
                    state=1;
                    begin_distant_integeral(1200);//开始距离积分
                    return true;
                }
                else if (counterSession >= 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                }
            }
            return false;
        }

        if(state==1)  // 开始补线
        {   
            catering_linefix();  
            set_speed(setpara.catering_speed);
            counterSession++;
            if(get_integeral_state(&distance_integral)==2) state=2;
        }

        if(state==2)  // 停车过程
        {   
            common_linefix();
            if (!stopped) //开始停车
            {
                car_stop();
                stop_time=mycar.RUNTIME;      
                stopped=true; 
            }
            else if(stopped&&mycar.RUNTIME-stop_time>=600)
            {
                mycar.car_running = 1;//重新启动
                stopped=false;
                state=3;
            }
        }

        if(state==3)  // 结束时的补线
        {
        for(int y=15;y<87;y++)//逐行扫描
        {
        if(lineinfo[y].left>=lineinfo[y+1].left&&
           lineinfo[y+1].left>=lineinfo[y+2].left&&
           lineinfo[y+2].left>=lineinfo[y+3].left&&
           lineinfo[y].left>=lineinfo[y-1].left&&
           lineinfo[y-1].left>=lineinfo[y-2].left&&
           lineinfo[y-2].left>=lineinfo[y-3].left&&
           y<watch.cross_LD_angle&&
           Grayscale[119-y-2][lineinfo[y].left]==255&&
           lineinfo[y-2].left>lineinfo[y-4].left&&
           lineinfo[y-1].left>lineinfo[y-3].left&&
           lineinfo[y].left>lineinfo[y-2].left&&
           lineinfo[y].left>lineinfo[y+2].left&&
           lineinfo[y+1].left>lineinfo[y+3].left&&!burgerLeft)
        {
            watch.cross_LD_angle=y;
            begin_distant_integeral(900);
            found=true;
            break;
        }
        
        if(lineinfo[y].right<=lineinfo[y+1].right&&
           lineinfo[y+1].right<=lineinfo[y+2].right&&
           lineinfo[y+2].right<=lineinfo[y+3].right&&
           lineinfo[y].right<=lineinfo[y-1].right&&
           lineinfo[y-1].right<=lineinfo[y-2].right&&
           lineinfo[y-2].right<=lineinfo[y-3].right&&
           y<watch.cross_RD_angle&&
           Grayscale[119-y-2][lineinfo[y].right]==255&&
           lineinfo[y-2].right<lineinfo[y-4].right&&
           lineinfo[y-1].right<lineinfo[y-3].right&&
           lineinfo[y].right<lineinfo[y-2].right&&
           lineinfo[y].right<lineinfo[y+2].right&&
           lineinfo[y+1].right<lineinfo[y+3].right&&burgerLeft)
        {
            watch.cross_RD_angle=y;
            begin_distant_integeral(900);
            found=true;
            break;
        }
        }

        if(burgerLeft)
        {
            for (int y =watch.cross_RD_angle-40; y <=watch.watch_lost; y++)
            {   
            int16_t xl,xr;    
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
            cout<<"汉堡补线开始啦"<<endl;     
            xl=y/0.8;
            xr=lineinfo[y].right;
            persp_task(xl,lineinfo[y].right,y);//对补线结果进行逆透视变换和存储
            } 
        }   
        else
        {
            for (int y =watch.cross_LD_angle-40; y <=watch.watch_lost; y++)
            { 
            int16_t xl,xr;  
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
            cout<<"汉堡补线开始啦"<<endl;     
            xr=187-y/0.8;
            xl=lineinfo[y].left;
            persp_task(lineinfo[y].left,xr,y);//对补线结果进行逆透视变换和存储
            }   
        }  
        
        if(state==3&&get_integeral_state(&distance_integral)==2&&found) state=4;
            
        } 

        if(state==4)  //退出岔路
        {
            counterRec = 0;
            counterSession = 0;
            cateringEnable = false;
            Element=None;
            state=0;
            watch.cross_LD_angle=120;
            watch.cross_LD_angle=120;
            return false;
        }

        return true;
        
    }
};

