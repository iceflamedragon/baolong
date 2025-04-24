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
 * @file layby.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 临时停车区
 * @version 0.1
 * @date 2025/03/04 20:29:04
 * @copyright  :Copyright (c) 2024
 * @note 具体功能模块:
 */

 #include <fstream>
 #include <iostream>
 #include <cmath>
 #include <opencv2/highgui.hpp>
 #include <opencv2/opencv.hpp>
 #include "../../include/common.hpp"
 #include "../../include/detection.hpp"
#include "../recognition/tracking.cpp"

using namespace cv;
using namespace std;

class Layby
{
public:

    bool stopEnable = false;        // 停车使能标志
    uint16_t Layby_distance_left0;
    uint16_t Layby_distance_left1;
    float Layby_slope_left;
    uint16_t Layby_distance_right0;
    uint16_t Layby_distance_right1;
    float Layby_slope_right;

    bool process(Mat &image, vector<PredictResult> predict)
    {
        if (!laybyEnable&&layby_state==0) // 检测标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                //if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY)  && predict[i].score > 0.6)  && predict[i].y  < ROWSIMAGE * 0.7)
                if ((predict[i].type == LABEL_BRIDGE  && predict[i].score > 0.6)  && predict[i].y  < ROWSIMAGE * 0.6)
                {
                    counterRec++;
                    if (predict[i].x < COLSIMAGE / 2)   // 标识牌在左侧
                        leftEnable = true;
                    else
                        leftEnable = false;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 4 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    laybyEnable = true; // 检测到标识牌子
                    layby_state=1;
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

        else // 处理
        {
            if(leftEnable)
            {
                if(layby_state==0)return false;
                if(layby_state==1)
                {
                    layby_linefix_left();
                    begin_distant_integeral(900);
                    layby_state=2;
                }
                else if(layby_state==2)
                {
                    layby_linefix_left();
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=3;
                    }
                }
                else if(layby_state==3)
                {
                    cout<<"开始临时停车"<<endl;
                    if (!stopEnable) //开始停车
                    {
                        cout<<"开始临时停车"<<endl;
                        car_stop();
                        stop_time=mycar.RUNTIME;      
                        stopEnable=true; 
                    }
                    else 
                    {
                        if(stopEnable&&mycar.RUNTIME-stop_time>=1000)
                        {
                    
                        mycar.car_running = 1;    //重新启动
                        stopEnable=false;
                        layby_state=0;
                       
                        return false;
                        }
                    }

                }
            }
            else if(!leftEnable)
            {
                if(layby_state==0)return false;
                if(layby_state==1)
                {
                    layby_linefix_right();
                    begin_distant_integeral(900);
                    layby_state=2;
                }
                else if(layby_state==2)
                {
                    layby_linefix_right();
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=3;
                    }
                }
                else if(layby_state==3)
                {
                    cout<<"开始临时停车"<<endl;
                    if (!stopEnable) //开始停车
                    {
                        cout<<"开始临时停车"<<endl;
                        car_stop();
                        stop_time=mycar.RUNTIME;      
                        stopEnable=true; 
                        
                    }
                    else if(stopEnable&&mycar.RUNTIME-stop_time>=1000)
                    {
                        mycar.car_running = 1;//重新启动
                        stopEnable=false;
                        Element=None;
                        layby_state=0;
                        cout<<"临时停车结束"<<endl;
                        return false;
                        out_element();
                        clear_distant_integeral();
                    }
                    
                }
            }
            return true;
        }
        return false;
    }

    bool process1(Mat &image, vector<PredictResult> predict)
    {
        if (!laybyEnable&&layby_state==0) // 检测标志
        {
            for (size_t i = 0; i < predict.size(); i++)
            {
                //if (((predict[i].type == LABEL_SCHOOL || predict[i].type == LABEL_COMPANY)  && predict[i].score > 0.6)  && predict[i].y  < ROWSIMAGE * 0.7)
                if ((predict[i].type == LABEL_BRIDGE  && predict[i].score > 0.6)  && predict[i].y  < ROWSIMAGE * 0.5)
                {
                    counterRec++;
                    if (predict[i].x < COLSIMAGE / 2)   // 标识牌在左侧
                        leftEnable = true;
                    else
                        leftEnable = false;
                    break;
                }
            }

            if (counterRec)
            {
                counterSession++;
                if (counterRec >= 4 && counterSession < 8)
                {
                    counterRec = 0;
                    counterSession = 0;
                    laybyEnable = true; // 检测到标识牌子
                    layby_state=1;
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

        else // 处理
        {
            if(leftEnable)
            {
                if(layby_state==0)return false;
                if(layby_state==1)
                {
                    layby_linefix_left();
                    begin_distant_integeral(Layby_distance_left0);
                    layby_state=2;
                }
                else if(layby_state==2)
                {
                    layby_linefix_left();
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=3;
                        clear_distant_integeral();

                        begin_distant_integeral(Layby_distance_left1);
                    }
                }
                else if(layby_state==3)
                {
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=4;
                    }
                }

                else if(layby_state==4)
                {
                    cout<<"开始临时停车"<<endl;
                    if (!stopEnable) //开始停车
                    {
                        cout<<"开始临时停车"<<endl;
                        car_stop();
                        stop_time=mycar.RUNTIME;      
                        stopEnable=true; 
                    }
                    else 
                    {
                        if(stopEnable&&mycar.RUNTIME-stop_time>=1000)
                        {
                    
                        mycar.car_running = 1;    //重新启动
                        stopEnable=false;
                        layby_state=0;
                        out_element();
                        clear_distant_integeral();
                        return false;
                        }
                    }

                }
            }
            else if(!leftEnable)
            {
                if(layby_state==0)return false;
                if(layby_state==1)
                {
                    layby_linefix_right();
                    begin_distant_integeral(Layby_distance_right0);
                    layby_state=2;
                }
                else if(layby_state==2)
                {
                    layby_linefix_right();
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=3;
                        clear_distant_integeral();
                        
                        begin_distant_integeral(Layby_distance_right1);
                    }
                }
                else if(layby_state==3)
                {
                    if(get_integeral_state(&distance_integral)==2)
                    {
                        layby_state=4;
                    }
                }
                else if(layby_state==4)
                {
                    cout<<"开始临时停车"<<endl;
                    if (!stopEnable) //开始停车
                    {
                        cout<<"开始临时停车"<<endl;
                        car_stop();
                        stop_time=mycar.RUNTIME;      
                        stopEnable=true; 
                        
                    }
                    else if(stopEnable&&mycar.RUNTIME-stop_time>=1000)
                    {
                        mycar.car_running = 1;//重新启动
                        stopEnable=false;
                        Element=None;
                        layby_state=0;
                        cout<<"临时停车结束"<<endl;
                       
                        out_element();
                        clear_distant_integeral();
                        return false;
                    }
                    
                }
            }
            return true;
        }
        return false;
    }


    void layby_linefix_left()
    {
        int16_t xl,xr;
        for (int y =forward_near; y <=watch.watch_lost; y++)
        {   
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
            
            xr=140-y/Layby_slope_left;
            xl=lineinfo[y].left;
            
            cout<<"左侧临时停车补线开始啦"<<endl;               
                 
            persp_task(lineinfo[y].left,xr,y);//对补线结果进行逆透视变换和存储 
        }

    }

    void layby_linefix_right()
    {
        int16_t xl,xr;
        for (int y =forward_near; y <=watch.watch_lost; y++)
        {   
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
          
            xl=y/Layby_slope_right;
            xr=lineinfo[y].right;
            
            cout<<"右侧临时停车补线开始啦"<<endl;               
            
            persp_task(xl,lineinfo[y].right,y);//对补线结果进行逆透视变换和存储
             
        }

    }

private:
    uint16_t counterSession = 0;    // 图像场次计数器
    uint16_t counterRec = 0;        // 标识牌检测计数器
    bool laybyEnable = false;       // 临时停车区域使能标志
    bool leftEnable = false;         // 标识牌在左侧
    int layby_state=0;               // 临时停车状态
    int stop_time=0;

    
};