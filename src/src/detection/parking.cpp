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
 * @file parking.cpp
 * @author HC (sasu@saishukeji.com)
 * @brief 充电停车场
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
 #include "../recognition/tracking.cpp"
 #include "../../include/detection.hpp"
 #include <stack>
 #include "../CAM_cpp/global.hpp"
using namespace cv;
using namespace std;

class Parking
{
public:
lineinfo_s linefind[ROWSIMAGE];
    /**
     * @brief 停车步骤
     *
     */
    enum ParkStep
    {
        none = 0, // 未知状态
        right_enable,
        right_turning,  // 入库转向
        right_stop,// 停车
        right_trackout,  // 出库 
        right_clear,
        left_enable, // 入库使能       
        left_turning,  // 入库转向     
        left_stop,
        left_trackout,
        left_clear   
    };
    int left_breakpoint,right_breakpoint;
    int lostline,balck_point0,balck_point1;
    int threshold_B_W=0;//判断第一个车库空不空0为空，1为不空
    bool parking0=false,parking1=false;//车库位置
    float t_slope_l=0,t_slope_r=0;//斜率t ,x=ty+b
    int blackx=80,blacky=65;
    int stop_time=0;//停止时间
    int total_steer_pwm=0,total_t=0;//总的pwm，和加的次数
    int ave_steer_pwm=0;
    bool stopped;
    int _lableImg[ROWSIMAGE][COLSIMAGE];
    ParkStep step = ParkStep::none; // 停车步骤

void process2( Mat &image, vector<PredictResult> predict)
{
     switch(step) 
     {
      case ParkStep::none:
      {
            left_breakpoint=Find_Left_Breakpoint(30,60);
            right_breakpoint=Find_Rightdown_Breakpoint(30,60);
            if(left_breakpoint&&!right_breakpoint)
            {
                lostline=Lostline(left_breakpoint-10,left_breakpoint+10,1);
                balck_point0=Conut_blackpoint(left_breakpoint,lineinfo[left_breakpoint].left,0);
                if(lostline<3&&balck_point0>20)//检查右侧丢线数量和左边黑色像素点数量
                   { step=ParkStep::left_enable;
                    enter_element(eparking);
                    }
            }
            if(!left_breakpoint&&right_breakpoint)
            {
                lostline=Lostline(right_breakpoint-10,right_breakpoint+10,1);
                balck_point0=Conut_blackpoint(right_breakpoint,lineinfo[right_breakpoint].right,187);
                if(lostline<3&&balck_point0>20)//检查左侧丢线数量和右边黑色像素点数量
                    {
                    step=ParkStep::right_enable;
                    enter_element(eparking);
                    }
            }
      }
      break;
     
      case ParkStep::right_enable:   
            threshold_B_W=Rect_Black_White(lineinfo[right_breakpoint].right-5,right_breakpoint+5,
                                          lineinfo[right_breakpoint].right+20,right_breakpoint+20,10,60,true);
            if(threshold_B_W<60)  //第一个库不为空
            {
                parking0=false;
                parking1=true;
                begin_distant_integeral(1000);
                step=ParkStep::right_turning;
                
          
            }

            else
            {
              cout<<"找到第一个库"<<endl;                                                                 
                //找到黑色T形下方点
                right_breakpoint=Find_Rightdown_Breakpoint(40,70);
                t_slope_r=regression(right_breakpoint,right_breakpoint-5,1);
                blackx=lineinfo[right_breakpoint].right-15;
                blacky=right_breakpoint+25;
                find_lowest_black_point(blackx,blacky,10,&blackx,&blacky);
                cout<<"blackx:"<<blackx<<"blacky:"<<blacky<<endl;
                parking0=true;
                parking1=false;
                step=ParkStep::right_turning;
                begin_distant_integeral(1000);

            }
      break;
        


      case ParkStep::right_turning:  
        if(parking0)//第一个库并且正在积分
           {              
            find_lowest_black_point(blackx,blacky,10,&blackx,&blacky);
            cout<<"开始补线"<<endl;
            cout<<"blackx"<<blackx<<"blacky:"<<blacky<<endl;
            
          for (int y =forward_near; y <=watch.watch_lost; y++)//补线
          {
              total_steer_pwm+=mycar.steer_pwm;
              total_t++;
            // cout<<"开始补线"<<endl;
            int16_t xl,xr;          // 补线后的结果
            float slopeTL,slopeTR; // 左右补线斜率
            if(blacky>30&&blacky<80)
            {
              total_steer_pwm+=mycar.steer_pwm;
              total_t++;
                slopeTL=(float)(blackx-lineinfo[0].left)/blacky;
                xl=slopeTL*y+lineinfo[0].left;
                xr=lineinfo[y].right;
                // cout<<"左斜线斜率"<<slopeTL<<"  "<<"左下角点x"<<xl<<endl;
                lineinfo[y].left_adjust=xl;
                lineinfo[y].right_adjust=xr;
                    //对补线后的结果进行逆透视变换
                
            }
            else if(blacky>80)
            {
            
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
        //记录补线后的结果
            lineinfo[y].left_adjust=xl;
            lineinfo[y].right_adjust=xr;
        //对补线后的结果进行逆透视变换
            
            }persp_task(xl,xr,y); 
           if(blacky<30||(get_integeral_state(&distance_integral)==2))//积分完成强制退出 
            {
            step=ParkStep::right_stop;//准备开始停车
            clear_distant_integeral();
            begin_distant_integeral(800);
            }       
        }
        
            
            }
        else if(parking1)
         {
              bool find_point;//只进去一次找点，
              int16_t xl,xr;
              float slopeTL,slopeTR; // 左右补线斜率
              if(get_integeral_data(&distance_integral)<500)
              {
              for (int y =forward_near; y <=watch.watch_lost; y++)
            {
              xl = lineinfo[y].left;
              xr = lineinfo[y].right;
              // 先拉一道实现封住出口,由于左边不丢线,故以右边为参考补左边线
              slopeTR=(float)(lineinfo[40].left-lineinfo[0].left)/40;
              watch.top_x=lineinfo[0].left+118*slopeTR;
              slopeTR=(float)(lineinfo[0].right-watch.top_x)/118;
              xr = watch.top_x+slopeTR*(118-y);
              persp_task(xl,xr,y);
            }
            }
            else 
            {
              static bool find_point==false;//只进去一次找点
              if(!find_point)
              {
              right_breakpoint=Find_Rightup_Breakpoint(50,80);
              find_lowest_black_point(lineinfo[right_breakpoint].right-5,right_breakpoint-10,10,&blackx,&blacky);
              }
              find_lowest_black_point(blackx,blacky,10,&blackx,&blacky);
                
          for (int y =forward_near; y <=watch.watch_lost; y++)//补线
          {
              total_steer_pwm+=mycar.steer_pwm;
              total_t++;
            // cout<<"开始补线"<<endl;
            int16_t xl,xr;          // 补线后的结果
            float slopeTL,slopeTR; // 左右补线斜率
            if(blacky>25&&blacky<80)
            {
              total_steer_pwm+=mycar.steer_pwm;
              total_t++;
                slopeTL=(float)(blackx-lineinfo[0].left)/blacky;
                xl=slopeTL*y+lineinfo[0].left;
                xr=lineinfo[y].right;
                // cout<<"左斜线斜率"<<slopeTL<<"  "<<"左下角点x"<<xl<<endl;
                lineinfo[y].left_adjust=xl;
                lineinfo[y].right_adjust=xr;
                //对补线后的结果进行逆透视变换
                
            }
            else if(blacky>80)
            {
            
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
        //记录补线后的结果
            lineinfo[y].left_adjust=xl;
            lineinfo[y].right_adjust=xr;
        //对补线后的结果进行逆透视变换    
            }persp_task(xl,xr,y); 
           if(blacky<25||(get_integeral_state(&distance_integral)==2))//积分完成强制退出 
            {
            step=ParkStep::right_stop;//准备开始停车
            clear_distant_integeral();
            begin_distant_integeral(800);
            } 
            }
         }


      break;    
      case ParkStep::right_stop:  
          {
            FindBottomRightWhiteTopPoint(50, 30, 130, 80, &blackx, &blacky);
            // icvprCcaBySeedFill2(120,50,80,45,&blackx,&blacky);
            for (int y =forward_near; y <=watch.watch_lost; y++)//补线
          {
            total_steer_pwm+=mycar.steer_pwm;
            total_t++;
            // cout<<"开始停车补线"<<endl;
            int16_t xl,xr;          // 补线后的结果
            float slopeTL,slopeTR; // 左右补线斜率
            if(blacky<20||get_integeral_state(&distance_integral)==2)//积分完成强制退出
            {
            step=ParkStep::right_trackout;//停车
              car_stop();
              clear_distant_integeral();
              stop_time=mycar.RUNTIME;      
          
            }
            else if(blacky>20)
            {
                slopeTL=(float)(blackx-lineinfo[0].left)/blacky;
                xl=slopeTL*y+lineinfo[0].left;
                xr=187;//直接贴边
                // cout<<"左斜线斜率"<<slopeTL<<"  "<<"左下角点x"<<xl<<endl;
                lineinfo[y].left_adjust=xl;
                lineinfo[y].right_adjust=xr;
                    //对补线后的结果进行逆透视变换
                persp_task(xl,xr,y);
            }
            
            else
            {            
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
        //记录补线后的结果
            lineinfo[y].left_adjust=xl;
            lineinfo[y].right_adjust=xr;
        //对补线后的结果进行逆透视变换
            
            }persp_task(xl,xr,y);
          }
          }break;
          
          //倒车退出  
      case ParkStep::right_trackout:
          {  
           if(mycar.RUNTIME-stop_time>=600&&stopped==false){
              mycar.car_running = 1;//重新启动、
              begin_angle_integeral(60);  
              stopped=true;            
              }
            if(get_integeral_state(&angle_integral)==1)  
           {  ave_steer_pwm=total_steer_pwm/total_t;
            mycar.steer_ctrl=0;    
            set_steerpwm(ave_steer_pwm);
            mycar.uart_servo = STEER_MID+ mycar.steer_pwm ;
            set_speed(-0.2);//倒车
            }
            else if(get_integeral_state(&angle_integral)==2)
            {
              clear_angle_integeral();
              mycar.steer_ctrl=1;
              mycar.speed_ctrl=1;
              step=ParkStep::right_clear;
              begin_distant_integeral(1200);
              
            }
          }  
      break;
      case ParkStep::right_clear:  
          {
          int16_t xl,xr;
          float slopeTL,slopeTR; // 左右补线斜率
          for (int y =forward_near; y <=watch.watch_lost; y++)
          {
            xl = lineinfo[y].left;
            xr = lineinfo[y].right;
           // 先拉一道实现封住出口,由于左边丢线右边不丢线,故以右边为参考补左边线
              slopeTR=(float)(lineinfo[40].left-lineinfo[0].left)/40;
              watch.top_x=lineinfo[0].left+118*slopeTR;
              slopeTR=(float)(lineinfo[0].right-watch.top_x)/118;
              xr = watch.top_x+slopeTR*(118-y);
           persp_task(xl,xr,y);
          }
          
          if(get_integeral_state(&distance_integral)==2)
          {
            clear_all_flags();
            step=ParkStep::none;
             left_breakpoint=0;
            right_breakpoint=0;
           lostline=0;balck_point0=0;balck_point1=0;;
           threshold_B_W=0;//判断第一个车库空不空0为空，1为不空
           parking0=false;parking1=false;//车库位置
           t_slope_l=0;t_slope_r=0;//斜率t ,x=ty+b
           blackx=ROWSIMAGE-1;
           blacky=COLSIMAGE-1;
           stop_time=0;//停止时间
           total_steer_pwm=0;
           total_t=0;//总的pwm，和加的次数
          }
          }
      break;



      /*左边*/  
      case ParkStep::left_enable:   
                


      break;      
            
      case ParkStep::left_turning:  
        if(parking0)
            left_breakpoint=Find_Left_Breakpoint(40,70);
      break;
      case ParkStep::left_stop:  
          {




            }
      break;
      case ParkStep::left_trackout: 
          {
            
            
          } 
      break;  
            
      case ParkStep::left_clear:  
          {
            
          }
      break;  
            
            
            
            
            
            
            }


}
    /************************************线性回归计算斜率************************************/

    /**
     * @brief 识别结果图像绘制
     *
     */
    void drawImage(Tracking track, Mat &image)
    {
        // 赛道边缘
        for (size_t i = 0; i < track.pointsEdgeLeft.size(); i++)
        {
            circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 1,
                   Scalar(0, 255, 0), -1); // 绿色点
        }
        for (size_t i = 0; i < track.pointsEdgeRight.size(); i++)
        {
            circle(image, Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
                   Scalar(0, 255, 255), -1); // 黄色点
        }
        
        if (step != ParkStep::none)
            putText(image, "[1] BATTERY - ENABLE", Point(COLSIMAGE / 2 - 30, 10), cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, CV_AA);
        
}
/************************************线性回归计算斜率************************************/
  /*
   *  @brief 最小二乘法拟合从上往下直线斜率
   * @param 输入点集,使用x=ty+b，求出的是t
   * @param startline 开始的行数
   * @param endline 结束的行数
   * @param left_or_right 0为左边，1为右边
   * @return 返回拟合的斜率
   */
  float regression(int startline, int endline,int left_or_right) {

    int i = 0, SumX = 0, SumY = 0, SumLines = 0;
    float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B, A;
    int t = 0;
    if (startline > endline) // 从下往上计算的，反了就互换一下
    {
      t = startline;
      startline = endline;
      endline = t;
    }
    if(startline<=0)startline=1;
    if(endline>watch.watch_lost)endline=watch.watch_lost;
    SumLines =
        endline - startline; // startline 为开始行， //endline 结束行 //SumLines
    if(!left_or_right) {
    for (i = startline; i < endline; i++) {
      SumY += i;
      SumX += lineinfo[i].left;  
    }}  
    else{
    for (i = startline; i < endline; i++) {
      SumY += i;
      SumX += lineinfo[i].right;  
        }
    }
     avrX = (float)SumX / SumLines;  // x的平均值（边缘坐标）
    avrY = (float)SumY / SumLines;  // y的平均值（行号）

    // 重新计算协方差和方差（关键修正）
    if (!left_or_right) {
        for (i = startline; i < endline; i++) {
            float x_i = lineinfo[i].left;
            float y_i = i;
            SumUp += (y_i - avrY) * (x_i - avrX);  // 协方差：Σ(y_i - y_avg)(x_i - x_avg)
            SumDown += (y_i - avrY) * (y_i - avrY);  // 方差：Σ(y_i - y_avg)^2
        }
    } else {
        for (i = startline; i < endline; i++) {
            float x_i = lineinfo[i].right;
            float y_i = i;
            SumUp += (y_i - avrY) * (x_i - avrX);
            SumDown += (y_i - avrY) * (y_i - avrY);
        }
    }

    B = (SumDown == 0) ? 0 : (SumUp / SumDown);  // 斜率 t
    return B;                   // 返回斜率
    }


 int Find_Left_Breakpoint(int start,int end) // 找四个角点，返回值是角点所在的行数
  {
    int i, t;
    int left_down_line = 0;
  if(watch.watch_lost<ROWSIMAGE*0.4) 
    {
      // cout<<"右侧大部分丢线"<<endl<<endl;
      return left_down_line;
    }
       if (start > end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (end >= watch.watch_lost) // 数组越界保护
      end =watch.watch_lost;
    if (start <= 5)
      start = 5;
   
    for (i = start; i <= end; i++) {
      if (left_down_line == 0 && // 只找第一个符合条件的点
          abs(lineinfo[i].left - lineinfo[i-1].left ) <=
              3 && // 角点的阈值可以更改
          abs(lineinfo[i-1].left - lineinfo[i-2].left ) <=
              3 &&
          abs(lineinfo[i-2].left - lineinfo[i-3].left ) <=3 &&
          abs(lineinfo[i-4].left - lineinfo[i-5].left ) <=3 && 
          abs(lineinfo[i+10].left - lineinfo[i].left) >=15&&
          abs(lineinfo[i+15].left - lineinfo[i].left) >=15) {
        // cout<<"左上角点有效行     "<<i<<endl;
        left_down_line = i; // 获取行数即可
        break;
      }
    }
    return left_down_line;
  }

int Find_Rightdown_Breakpoint(int start,int end)
  {
    int i, t;
    int right_down_line = 0;
  if(watch.watch_lost<ROWSIMAGE*0.4) 
    {
      // cout<<"右侧大部分丢线"<<endl<<endl;
      return right_down_line;
    }
       if (start > end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (end >= watch.watch_lost) // 数组越界保护
      end =watch.watch_lost;
    if (start <= 5)
      start = 5;
   
    for (i = start; i <= end; i++) {
      if (right_down_line == 0 && // 只找第一个符合条件的点，这个地方的条件改了第一个库有车不一定能进去
          abs(lineinfo[i].right - lineinfo[i-1].right ) <=
              3 && // 角点的阈值可以更改
          abs(lineinfo[i-1].right - lineinfo[i-2].right ) <=
              3 &&
          abs(lineinfo[i-2].right - lineinfo[i-3].right ) <=3 &&
          abs(lineinfo[i-4].right - lineinfo[i-5].right ) <=3 && 
          abs(lineinfo[i+1].right - lineinfo[i].right) >=5&&
          abs(lineinfo[i+2].right - lineinfo[i].right) >=8&&
          abs(lineinfo[i+3].right - lineinfo[i].right) >=10&&

          abs(lineinfo[i+3].right - lineinfo[i+2].right) >=2&&    //这几个点成阶梯式
          abs(lineinfo[i+3].right - lineinfo[i+2].right) <=8&&
          abs(lineinfo[i+4].right - lineinfo[i+3].right) <=8&&
          abs(lineinfo[i+4].right - lineinfo[i+3].right) >=2
          ) {
        // cout<<"左上角点有效行     "<<i<<endl;
        right_down_line = i; // 获取行数即可
        // cout<<"kaishipanduan"<<endl;
        break;
      }
    }
    return right_down_line;
  }  
  
int Find_Rightup_Breakpoint(int start,int end)
  {
    int i, t;
    int right_down_line = 0;
  if(watch.watch_lost<ROWSIMAGE*0.4) 
    {
      // cout<<"进入弯道"<<endl<<endl;
      return right_down_line;
    }
       if (start > end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (end >= watch.watch_lost) // 数组越界保护
      end =watch.watch_lost;
    if (start <= 5)
      start = 5;
   
    for (i = end; i >= start; i--) {  //从上往下找
      if (right_down_line == 0 && // 只找第一个符合条件的点，这个地方的条件改了第一个库有车不一定能进去
          abs(lineinfo[i].right - lineinfo[i+1].right ) <=
              3 && // 角点的阈值可以更改
          abs(lineinfo[i+1].right - lineinfo[i+2].right ) <=
              3 &&
          abs(lineinfo[i+2].right - lineinfo[i+3].right ) <=3 &&
          abs(lineinfo[i+4].right - lineinfo[i+5].right ) <=3 && 
          abs(lineinfo[i-5].right - lineinfo[i].right) >=10&&
          abs(lineinfo[i-10].right - lineinfo[i].right) >=15&&
          abs(lineinfo[i-12].right - lineinfo[i].right) >=15&&

          ) {
        // cout<<"左上角点有效行     "<<i<<endl;
        right_down_line = i; // 获取行数即可
        // cout<<"kaishipanduan"<<endl;
        break;
      }
    }
    return right_down_line;
  }  
  /*-------------------------------------------------------------------------------------------------------------------
  *@brief     检查左侧丢线数量
  *@param     起始点，终止行
  *@param     0是左，1是右
  *@return    点所在的扫线的行0到120，，单调就为0
  *Sample     Find_Right_Up_Point(int start,int end);
  *@note      小于2就位丢线
  -------------------------------------------------------------------------------------------------------------------*/
  int Lostline( int start,
                    int end,int left_or_right) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int lostline = 0;
       if (start > end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (end >= watch.watch_lost) // 数组越界保护
      end =watch.watch_lost;
    if (start <= 5)
      start = 5;
   
    for (i = start; i <= end; i++) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if(left_or_right==0&&lineinfo[i].left_lost)
      lostline++;
      else if(left_or_right==1&&lineinfo[i].right_lost)
      lostline++;
    }
    return lostline;
  }
/*
  @brief     检查两个直线点之间的黑色像素数量
  @param     起始点xy 结束点x     
  @return    返回像素数
*/
int Conut_blackpoint(int y, int start_x,int end_x)
{
    int blackpoint = 0,t=0;
    if (start_x > end_x) // 都是从下往上计算的，反了就互换一下
    {
      t = start_x;
      start_x = end_x;
      end_x = t;
    }
    for (int x = start_x; x <= end_x; x++)
    {
        if (Grayscale[119-y][x] < 5)
        {
            blackpoint++;
        }
    }
    return blackpoint;
}


/*
  @brief     检查矩形区域的黑色像素平均值
  @param     左下角为起始点xy 右上角为结束点xy
  @param     两个阈值，0到255 
  @param     是否只看计数    
  @return    0为基本全黑，1为黑白都有，
*/

int Rect_Black_White(int start_x,int start_y, int end_x,int end_y,int threshold_black,int threshold_white,bool if_threshold_num)
{
    if(end_x>=COLSIMAGE-1)end_x=COLSIMAGE-1;
    if(start_x<=0)start_x=0;
    int threashold = 0;
    int pointnum=0;
    for (int y = start_y; y <= end_y; y++)
    {
        for(int x = start_x; x <= end_x; x++)
           {threashold+=Grayscale[119-y][x];
           pointnum++;}
    }
    threashold=threashold/pointnum;
    if(if_threshold_num)return threashold;
    if(threashold<threshold_black)return 0;
    if(threashold>threshold_white)return 1;
}

/*
  @brief     持续追踪已找到的黑块
  @param     已找到的点的坐标x
  @param     已找到的点的坐标y 
  @param     找寻范围range
*/

void Tracking_Black_Point(int start_x,int start_y,int range)
{
    
    for (int y = 119-start_y-range; y <= start_y+range; y++)
    {
      for(int x = start_x-range; x <= start_x+range; x++)
      {
         if(Grayscale[119-y][x]==0&&((y-119-start_y-range)*(y-119-start_y-range)+(x-start_x-range)*(x-start_x-range)<(blacky-119-start_y-range)*(blacky-119-start_y-range)+(blackx-start_x-range)*(blackx-start_x-range)))
         {blackx=x;blacky=y;}
      }
    }
}
void find_lowest_black_point(int start_x, int start_y, int range, int* out_x, int* out_y) {
    int min_y = start_y - range;
    int max_y = start_y + range;
    
    // 边界检查
    if (max_y > 119) max_y = 119;
    if (min_y < 0) min_y = 0;

    // 从下往上遍历
    for (int y = min_y; y <= max_y; y++) {
        int min_x = start_x - range;
        int max_x = start_x + range;
        
        if (min_x < 0) min_x = 0;
        if (max_x >= COLSIMAGE) max_x = COLSIMAGE - 1;

        // 遍历当前行的x范围
        for (int x = min_x; x <= max_x; x++) {
            if (Grayscale[119 - y][x] == 0) { // 判断是否为黑点
                *out_x = x;
                *out_y = y;
                return; // 找到最下面的直接返回
            }
        }
    }

    // 未找到时返回初始坐标
    *out_x = start_x;
    *out_y = start_y;
}
//八邻域向左上找角点
void find_angle_left_up(int*angle_x,int*angle_y,int endy)
{
    int x=*angle_x, y=*angle_y,p=(int)*angle_y;
    if(Grayscale[119-y][x]==0&&Grayscale[119-(y-1)][x]==0&&Grayscale[119-(y+1)][x]==0&&Grayscale[119-y][x-1]==0&&Grayscale[119-y][x+1]==0) return ; 
    while(Grayscale[119-y][x]!=0&&y<endy)
    {
      if(Grayscale[119-y][x]==0&&Grayscale[119-(y-1)][x]==0&&Grayscale[119-(y+1)][x]==0&&Grayscale[119-y][x-1]==0&&Grayscale[119-y][x+1]==0) return ; 
        y++;
    }
    while(Grayscale[119-y][x+1]!=255&&x<187)
    {
      if(Grayscale[119-y][x]==0&&Grayscale[119-(y-1)][x]==0&&Grayscale[119-(y+1)][x]==0&&Grayscale[119-y][x-1]==0&&Grayscale[119-y][x+1]==0) return ; 
        x++;
    }
    while(y>20&&y<80&&x<187)
    {
        
        if(Grayscale[119-y][x]==255)
        {
            break;
        }
        else if(Grayscale[119-y][x-1]==255)
        {
            x--;
        }
        else if(Grayscale[119-y][x+1]==255)
        {
            x++;
        }
        else if(Grayscale[119-y][x-2]==255)
        {
            x=x-2;
        }
        else if(Grayscale[119-y][x+2]==255)
        {
            x=x+2;
        }
        else if(Grayscale[119-y][x-3]==255)
        {
            x=x-3;
        }
        else if(Grayscale[119-y][x+3]==255)
        {
            x=x+3;
        }
        else break;
        
    }
    *angle_x=x;
    *angle_y=y;
}
//八邻域向右上找角点
void find_angle_right_up(int*angle_x,int*angle_y,int endy)
{
    int x=*angle_x, y=*angle_y,p=(int)*angle_y;
    while(Grayscale[119-y][x]!=0&&y<endy)
    {
        y++;
    }
    while(Grayscale[119-y][x-1]!=255&&x>0)
    {
        x--;
    }
    while(y>20&&y<p+20&&x>0)
    {
        if(Grayscale[119-y][x]==0)
        {   
            
        }
        else if(Grayscale[119-y][x+3]==0)
        {
            x+=3;
        }
        else if(Grayscale[119-y][x+2]==0)
        {
            x+=2;
        }
        else if(Grayscale[119-y][x+1]==0)
        {
            x++;
        }
        else if(Grayscale[119-y][x-1]==0)
        {
            x--;
        }
        else if(Grayscale[119-y][x-2]==0)
        {
            x-=2;
        }
        else if(Grayscale[119-y][x-3]==0)
        {   
            x=x-3;
        }    
        else break;
        y++;
    } 
    *angle_x=x;
    *angle_y=y;
}
void FindBottomRightWhiteTopPoint(int start_x, int start_y, int end_x, int end_y, int* out_x, int* out_y)
{
    const int WHITE_THRESHOLD = 255;
    const int IMG_HEIGHT = 120; // 根据实际图像高度调整
    const int RIGHT_THRESHOLD = 50; // 右侧区域阈值
    bool found = false;
    
    // 初始化访问标记数组（使用vector避免栈溢出）
    std::vector<std::vector<bool>> visited(
        IMG_HEIGHT, 
        std::vector<bool>(COLSIMAGE, false)
    );

    // 转换坐标系后的扫描顺序：
    // 从转换后的左下角坐标系底部(y=0)向顶部(y=119)扫描
    for (int y_trans = 0; y_trans < IMG_HEIGHT; y_trans++) {
        // 优先扫描右侧区域（转换后坐标系的x较大侧）
        for (int x = COLSIMAGE - 1; x >= RIGHT_THRESHOLD; x--) {
            // 转换为原始数组索引
            // 跳过非搜索区域
            if (x < start_x || x > end_x || y_trans < start_y || y_trans > end_y)
                continue;

            if (!visited[y_trans][x] && Grayscale[119-y_trans][x] >= WHITE_THRESHOLD) {
                // 发现新连通域，进行BFS标记
                int current_max_y = y_trans; // 转换后坐标系的最大y值（最上方）
                int domain_rightmost = x;    // 连通域最右侧x坐标
                std::queue<std::pair<int, int>> q;
                q.push({x, y_trans});
                visited[y_trans][x] = true;

                while (!q.empty()) {
                    auto [cur_x, cur_oy] = q.front(); // cur_oy是数组原始行索引
                    q.pop();
                    
                    
                    // 更新最上边界（转换后坐标系y值越小表示越靠上）
                    if (y_trans > current_max_y) {
                        current_max_y = y_trans;
                    }

        // 四邻域扩展（转换后坐标系的移动方向）
            for (int dx = 1; dx >= -1; dx--) {
                if (dx == 0){
            for (int dy_oy = -1; dy_oy <= 1; dy_oy++) { // 原始y坐标变化量
                if (dx == 0 && dy_oy == 0) continue;
                            
                            int nx = cur_x + dx;
                            int ny_oy = cur_oy + dy_oy; // 新的原始y坐标
                                                        
                            // 边界检查（转换后坐标系）
                            if (nx >= start_x && nx <= end_x &&
                                ny_oy >= start_y && ny_oy <= end_y &&
                                ny_oy >= 0 && ny_oy < IMG_HEIGHT &&
                                !visited[ny_oy][nx] &&
                                Grayscale[119-ny_oy][nx] >= WHITE_THRESHOLD) {
                                
                                visited[ny_oy][nx] = true;
                                q.push({nx, ny_oy});

                                // 更新最右点
                                if (nx > domain_rightmost) domain_rightmost = nx;
                            }
                        }
                          }}
                }

                // 筛选条件：位于右侧区域且是最靠上的连通域
                    
                        
                        *out_x = domain_rightmost; // 取最右点x坐标
                        *out_y = current_max_y;    // 转换后坐标系的最上y值
                        found = true;
                    
        
    }
}
    }

    if (!found) 
    {
        *out_x = -1;
        *out_y = -1;
    }
    
}
/*
 *@brief     从右下角开始向上向左扫描矩形区域的最右侧顶点 
  *@param     右下角xy 矩形行列     
  *@return    
*/
void icvprCcaBySeedFill( cv::Mat& _lableImg,int R_D_x,int R_D_y, int cols,int rows)
{
	/*
	种子填充法的连通区域分析方法：
（1）扫描图像，直到当前像素点B(x,y) == 1：
a、将B(x,y)作为种子（像素位置），并赋予其一个label，然后将该种子相邻的所有前景像素都压入栈中；
b、弹出栈顶像素，赋予其相同的label，然后再将与该栈顶像素相邻的所有前景像素都压入栈中；
c、重复b步骤，直到栈为空；
此时，便找到了图像B中的一个连通区域，该区域内的像素值被标记为label；
（2）重复第（1）步，直到扫描结束；
扫描结束后，就可以得到图像B中所有的连通区域；
	*/

	int label = 1;  // start by 2

	for (int y = R_D_y-( rows - 1); y <R_D_y; y++)
	{
		for (int x = R_D_x; x >R_D_x-(cols - 1); x--)
		{
			if (Grayscale[119-y][x] == 255)
			{
				std::stack<std::pair<int, int>> neighborPixels;
				neighborPixels.push(std::pair<int, int>(y, x));     // 像素坐标: <y,x>
				++label;  //从一个新label开始
				while (!neighborPixels.empty())
				{
					// 栈中最上面的像素给予和与其连通的像素相同的label
					std::pair<int, int> curPixel = neighborPixels.top();
					int curX = curPixel.first;
					int curY = curPixel.second;
					_lableImg.at<int>(curX, curY) = label;

					// 弹出最上面的像素
					neighborPixels.pop();

					// 推及到四个邻居
					if (_lableImg.at<int>(curX, curY - 1) == 1)
					{// 左边的像素
						neighborPixels.push(std::pair<int, int>(curX, curY - 1));
					}
					if (_lableImg.at<int>(curX, curY + 1) == 1)
					{// 右边的像素
						neighborPixels.push(std::pair<int, int>(curX, curY + 1));
					}
					if (_lableImg.at<int>(curX - 1, curY) == 1)
					{// 上面的像素
						neighborPixels.push(std::pair<int, int>(curX - 1, curY));
					}
					if (_lableImg.at<int>(curX + 1, curY) == 1)
					{// 下面的像素
						neighborPixels.push(std::pair<int, int>(curX + 1, curY));
					}
				}
			}
		}
	}
}
/**
 * @brief 从指定点出发寻找连通域的最高点（图像坐标系）
 * @param start_x 起始点x坐标（图像坐标系）
 * @param start_y 起始点y坐标（图像坐标系，0=顶部）
 * @param up_y  限制范围上边y，比down_y大
 * @param down_y 限制范围下边y
 * @param out_x 最高点x坐标输出
 * @param out_y 最高点y坐标输出（图像坐标系）
 */
void icvprCcaBySeedFill2(int start_x, int start_y, int up_y,int down_y,int* out_x, int* out_y)
{
    const int IMG_HEIGHT = 120;
    const int WHITE = 255;
    
    // 转换到数组坐标系
    int arr_y = 119 - start_y;
    if(up_y<down_y)
    {
      int t=down_y;
      down_y=up_y;
      up_y=t;
    }
     up_y=119-up_y;        //调转对齐
     down_y=119-down_y;
    while (Grayscale[arr_y][start_x] != WHITE)//过滤小黑点
    {
      start_x=start_x-1;
    }
    // 合法性检查
    if (start_x < 0 || start_x >= COLSIMAGE || 
        start_y < 0 || start_y >= IMG_HEIGHT ||
        Grayscale[arr_y][start_x] != WHITE) {
        *out_x = -1;
        *out_y = -1;
        return;
    }

    std::vector<std::vector<bool>> visited(
        IMG_HEIGHT, 
        std::vector<bool>(COLSIMAGE, false)
    );

    std::queue<std::pair<int, int>> q;
    int min_img_y = down_y; // 图像坐标系最小y值（最高点）
    int max_x = start_x;

    // 初始化队列（存储数组坐标系坐标）
    q.push({start_x, arr_y});
    visited[arr_y][start_x] = true;

    // 四邻域方向（数组坐标系）
    const int dx[] = {-1, 1, 0, 0};
    const int dy[] = {0, 0, -1, 1}; 

    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        // 转换为图像坐标系判断最高点
        int img_y = 119 - y;
        if (img_y < min_img_y || (img_y == min_img_y && x > max_x)) {
            min_img_y = img_y;
            max_x = x;
        }

        // 四邻域扩展
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (nx >= 0 && nx < COLSIMAGE && 
                ny >= down_y && ny < up_y &&
                !visited[ny][nx] && 
                Grayscale[ny][nx] == WHITE) {
                
                visited[ny][nx] = true;
                q.push({nx, ny});
            }
        }
    }

    *out_x = max_x;
    *out_y = min_img_y;
}

void FindTopWhitePoint4(int rect_left, int rect_right,int rect_down, int rect_top, int* out_x, int* out_y) {
    const int WHITE = 255;
    bool found = false;
    int maxY = -1;        // 最高点Y坐标（左下角为原点）
    int targetX = -1;     // 对应X坐标

    // 创建访问标记矩阵（y范围：0~rect_top，x范围：rect_left~rect_right）
    vector<vector<bool>> visited(rect_top + 1, vector<bool>(rect_right  + 1, false));

    // 从下往上扫描每一行（y=0是底部）
    for (int y = rect_down; y <= rect_top; ++y) {
        // 在当前行内从右向左扫描
        for (int x = rect_right; x >= rect_left; --x) {
            // 坐标转换：Grayscale[119 - y][x] 对应图像坐标系
            if (Grayscale[119 - y][x] == WHITE && !visited[y][x ]) {
                queue<pair<int, int>> q;
                q.push({x, y});
                visited[y][x ] = true;
                int localMaxY = y;  // 当前连通域的最高点Y
                int localX = x;     // 对应的X坐标

                // 四邻域方向：上(0,-1)、下(0,1)、左(-1,0)、右(1,0)
                const int dx[] = {0, 0, -1, 1};
                const int dy[] = {-1, 1, 0, 0};

                while (!q.empty()) {
                    auto [curX, curY] = q.front();
                    q.pop();

                    // 更新当前连通域的最高点（优先右侧）
                    if (curY > localMaxY || (curY == localMaxY && curX > localX)) {
                        localMaxY = curY;
                        localX = curX;
                    }

                    // 四邻域遍历
                    for (int i = 0; i < 4; ++i) {
                        int nx = curX + dx[i];
                        int ny = curY + dy[i];  // 注意：ny是相对于左下角原点的y坐标

                        // 检查是否在目标矩形内
                        if (nx >= rect_left && nx <= rect_right &&
                            ny >= rect_down && ny <= rect_top) {
                            
                            // 转换为访问矩阵坐标
                            int visX = nx ;
                            int visY = ny;

                            if (!visited[visY][visX] && 
                                Grayscale[119 - ny][nx] == WHITE) {
                                visited[visY][visX] = true;
                                q.push({nx, ny});
                            }
                        }
                    }
                }

                // 更新全局最高点（优先取y更小的点，y相同时取右侧）
                if (localMaxY > maxY || (localMaxY == maxY && localX > targetX)) {
                    maxY = localMaxY;
                    targetX = localX;
                    found = true;
                }
            }
        }
    }

    if (found) {
        *out_x = targetX;
        *out_y = maxY;  // 注意：返回的y是相对于左下角原点的坐标
    } else {
        *out_x = -1;
        *out_y = -1;
    }
}
private:
    uint16_t counterSession = 0;  // 图像场次计数器
    uint16_t counterRec = 0;      // 加油站标志检测计数器
    bool garageFirst = true;      // 进入一号车库
    int lineY = 0;                // 直线高度
    bool startTurning = false;    // 开始转弯
    vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
    vector<vector<POINT>> pathsEdgeRight;
    Point ptA = Point(0, 0);      // 记录线段的两个端点
    Point ptB = Point(0, 0);
    int truningTime = 21;             // 转弯时间 21帧
    int stopTime = 40;                // 停车时间 40帧
    float swerveTime = 0.2;           // 转向时机 0.2 （转弯线出现在屏幕上方0.2处）
};