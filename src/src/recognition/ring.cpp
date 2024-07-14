#pragma once
/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file ring.cpp
 * @author Leo
 * @brief 环岛识别（基于track赛道识别后）
 * @version 0.1
 * @date 2022-02-28
 *
 * @copyright Copyright (c) 2022
 *
 * @note  环岛识别步骤（ringStep）：
 *          1：环岛识别（初始化）
 *          2：入环处理
 *          3：环中处理
 *          4：出环处理
 *          5：出环结束
 */
/*
几点问题：鲁棒性不好
1.countwide误判调参
2.延时的条件误判----将十字写死，不让误判
*/

#include "../../include/common.hpp"
#include "../../include/uart.hpp"
#include "../motion.cpp"
#include "crossroad.cpp"
#include "tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

extern shared_ptr<Uart> uart;
using namespace cv;
using namespace std;

class Ring {
private:
  float ring_p1;
  float ring_p2;
  float ring_d;
  float danger_p1;
  float danger_p2;
  float danger_d;
  float common_p1;
  float common_p2;
  float common_d;
  float common_i;
  Crossroad c1;
  float distance_now;
  float distance_in;       // 状态一的编码器值
  float distance_diff;     // 差值
  float distance_laterout; // 出环的编码器值
  float distance_final;

public: // int leftpoint;
  // int rightpoint;
  int flag_closeai;
  int flagbigringl=0;
  int flagbigringr=0;
  int flagpid=0;
  int left_breakpoint = 10;  // 左拐点行号
  int right_breakpoint = 10; // 右拐点行号
  int monotonicity_change_line[2]; // 单调性改变点坐标，[0]寸某行，[1]寸某列
  int monotonicity_right = 0; // 右侧严格单调,单调为0
  int monotonicity_left = 0;  // 左侧严格单调，单调为0
  int lostline_left = 0;      // 左右丢线数量
  int lostline_right = 0;
  int monotonicity_change_left_flag = 0;  // 不转折是0
  int monotonicity_change_right_flag = 0; // 不转折是0
  int continuity_change_right_flag = 0;   // 连续是0
  int continuity_change_left_flag = 0;    // 连续是0
  bool roundaboutArc=0;//圆弧标志位
  int center_sum_flag;///中心值积分标志
  double part_stdevEdgeCal_left=0; //左边缘部分斜率方差
  double part_stdevEdgeCal_right=0; //左边缘部分斜率方差
  int ringTypeTemp = 0;   //环的类型
  int flagjiao = 0; //

  void setmpu6050(float mpu6050_now_read) { mpu6050_now = mpu6050_now_read; }
  void setdistance(float distance) { distance_now = distance; };
  uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

#define  Center_Sum_None 0
#define  Center_Sum_Start 1
#define  Center_Sum_End 2
#define  Center_Sum_Reset 3
#define RingNone 0
#define RingLeft 1
#define RingRight 2
  /**
   * @brief 环岛识别初始化|复位
   *
   */
  // static int ru;
  void reset(void) {
    int ringType = RingLeft;            // 环岛类型
    RingStep ringStep = RingStep::None; // 环岛处理阶段
    int rowRepairLine = 0;              // 用于环补线的点（行号）
    int colRepairLine = 0;              // 用于环补线的点（列号）
    counterSpurroad = 0;                // 岔路计数器
    counterShield = 0;
  }

  bool ringEnable = false; // 判环标志
  void set_ring_pid(float ring_p1, float ring_p2, float ring_d,
                    Motion &motion) // 设置圆环pid
  {
    // cout << "原先的pid为" << motion.params.ring_p1 << endl
    //      << motion.params.ring_p2 << endl
    //      << motion.params.ring_d << endl;
    // motion.params.ring_p1 = ring_p1;

    // motion.params.ring_p2 = ring_p2;

    // motion.params.ring_d = ring_d;
     motion.params.runP1 = ring_p1;

     motion.params.runP2 = ring_p2;

     motion.params.turnD = ring_d;
    cout << "设置圆环的pid" << endl;
  }
  void save_common_pid(Motion &motion) {
    common_p1 = motion.params.runP1;
    common_p2 = motion.params.runP2;
    common_d = motion.params.turnD;
    common_i = motion.params.turnI;
  }
  //  p RingType ringTypeTemp = RingTye::RingNone; // 环岛类型：临时变量
  // void ringtype_init(void)
  // {
  // RingType ringTypeTemp = RingNone; // 环岛类型：临时变量
  // }
  /**
   * @brief 环岛识别与行径规划
   *
   * @param track 基础赛道识别结果
   * @param imagePath 赛道路径图像
   */

  bool process(Tracking &track, Mat &imagePath, Motion &motion) {
    // 暂时注释
    //  if (counterShield < 40) {
    //    counterShield++;
    //    return false;
    //  }

    /////借鉴的代码
    // static int time = 0;
    //  cout<<"标志位。。。"<<first_ring<<endl;
    //   以下是常规判断法

    // continuity_change_left_flag =
    //     Continuity_Change_Left(track, ROWSIMAGE - 1 - 5, 30); // 连续性判断
    // continuity_change_right_flag =
    //     Continuity_Change_Right(track, ROWSIMAGE - 1 - 5, 30);
    // monotonicity_change_right_flag =
    //     Monotonicity_Change_Right(track, ROWSIMAGE - 1 - 10, 30);
    // monotonicity_change_left_flag =
    //     Monotonicity_Change_Left(track, ROWSIMAGE - 1 - 10, 30);

    // bool ringEnable = false;                    // 判环标志
    //  RingType ringTypeTemp = RingNone; // 环岛类型：临时变量
    // if(ringStep==RingStep::Entering||ringStep==RingStep::Inside||ringStep==RingStep::Exiting)  flag_closeai=1;
    // else flag_closeai=0;

    int rowBreakpointLeft = 0;  // 边缘拐点起始行（左）
    int rowBreakpointRight = 0; // 边缘拐点起始行（右）
    int colEnterRing = 0;       // 入环点（图像列序号）
    int rowRepairRingside =
        track.widthBlock.size() - 1; // 环一侧，补线起点（行号）
    int rowRepairStraightside =
        track.widthBlock.size() - 1; // 直道侧，补线起点（行号）
    int rowYendStraightside =
        track.widthBlock.size() - 1; // 直道侧，延长补线终点（行号）
    _index = 0;
    _ringPoint = POINT(0, 0);
   
    // 算环用布线的候选点
    rowRepairLine = max(rowRepairLine - 5, 0); // 在那其中最下面的点
    if (ringStep == RingStep::Entering && !track.spurroad.empty()) {
      if (ringType == RingLeft && track.pointsEdgeLeft.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeLeft.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeLeft[j].x >= track.spurroad[0].x; // 在拐点之前
             j++) {
          if (track.pointsEdgeLeft[j].y >
                  track.pointsEdgeLeft[j - 10]
                      .y && // 比他上下10行都大，为补线候选点
              track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j + 10].y) {
            rowRepairLine = j;
            cout << "算出布线的候选点左" << endl;
            break;
          }
        }
      } else if (ringType == RingRight && track.pointsEdgeRight.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeRight.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeRight[j].x >= track.spurroad[0].x;
             j++) {
          if (track.pointsEdgeRight[j].y < track.pointsEdgeRight[j - 10].y &&
              track.pointsEdgeRight[j].y < track.pointsEdgeRight[j + 10].y) {
            rowRepairLine = j;
            cout << "算出布线的候选点右" << endl;
            break;
          }
        }
      }
    }

    // 搜索赛道左右边缘满足图像边沿的最高处，也就是有效行的开始从下往上
    for (int ii = 0; ii < track.pointsEdgeLeft.size(); ++ii) {
      rowBreakpointLeft = track.pointsEdgeLeft[ii].x;
      if (track.pointsEdgeLeft[ii].y > 2)
        break;
    }
    for (int ii = 0; ii < track.pointsEdgeRight.size(); ++ii) {
      rowBreakpointRight = track.pointsEdgeRight[ii].x;
      if (track.pointsEdgeRight[ii].y < COLSIMAGE - 3)
        break;
    }

    /////////// 判环,是否进入状态一
    // 环岛入口变宽区域行数,是变宽的过程

    if (ringStep == RingStep::None) {
      // 判断左入环
      part_stdevEdgeCal_left=Part_stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE,40,160);
      // cout<<"左环右侧拐点周围斜率方差"<<part_stdevEdgeCal_left<<endl;
      // cout<<"kaishipanduan9999999999"<<endl;s
      left_breakpoint = Find_Left_Breakpoint(track, 80, 140);
      right_breakpoint = Find_Right_Breakpoint(track, 80, 140); // 原来150
      if (left_breakpoint) // 有左下拐点
      {
        lostline_left =
            Lostline_Left(track, left_breakpoint + 40, left_breakpoint);
            cout<<"左环左侧丢线数量"<<lostline_left<<endl;
        continuity_change_right_flag =
            Continuity_Change_Right(track, left_breakpoint - 10,
                                    left_breakpoint + 10); // 拐点附近判断连续性
         cout<<"左环的右边连续性"<<continuity_change_right_flag<<endl;

        continuity_change_left_flag =
            Continuity_Change_Left(track, ROWSIMAGE - 1 - 5, 50);
           cout<<"左环左侧连续性"<< continuity_change_left_flag<<endl;
           cout<<"左环左边有效行数量"<<track.validRowsLeft<<endl;
           cout<<"左环右边有效行数量"<<track.validRowsRight<<endl;

        if (ringTypeTemp == RingNone) {
          if (lostline_left > 15 &&                 // 判断左入环
              continuity_change_left_flag != 0 &&  // 左边是不连续的
              continuity_change_right_flag == 0 &&   // 左环岛右边是连续的
              track.validRowsLeft >= 50 &&        // 左边有效行不能少
              // track.validRowsLeft <= 120 &&        // 左边有效行比较少
              track.validRowsRight >= 100         // 右边丢线较少
              &&track.stdevRight<50&&track.stdevLeft>150
          ) {
            
            monotonicity_right =
                Monotonicity_Right(track, left_breakpoint - 15,
                                   left_breakpoint + 15); // 原来此处为10
            part_stdevEdgeCal_left=Part_stdevEdgeCal(track.pointsEdgeLeft, ROWSIMAGE,left_breakpoint - 15,
                                   left_breakpoint + 15);
          
            cout<<"左环右侧严格单调性"<<monotonicity_right<<endl;
            if (monotonicity_right == 0) {                // 右边是单调的
              ringStep = RingStep::IsRing;
              ringTypeTemp = RingLeft; // 环岛类型：左入环
              ringEnable = true;
              std::cout << "判断出左入环" << endl;
              distance_in = distance_now;
              // mpu6050_later = mpu6050_now;
            } else {
              ringStep == RingStep::None;
              cout << "误判环" << endl;
            }
          }
        }
        // cout<<"左环右侧拐点周围斜率方差"<<part_stdevEdgeCal_left<<endl;
        ////判断右入环
      } else if (right_breakpoint) // 有右下拐点
      {
        cout << "圆环判断出右下拐点" << endl;
        lostline_right = Lostline_Right(track, right_breakpoint + 40, // 找丢线
                                        right_breakpoint);
                                        cout<<"右环丢线"<< lostline_right<<endl;
        continuity_change_left_flag = Continuity_Change_Left(
            track, right_breakpoint - 10, right_breakpoint+10); // 大部分左边线判断连续性
            cout<<"右环左边连续性"<<continuity_change_left_flag<<endl;
        continuity_change_right_flag =          // 右边肯定应该不连续
            Continuity_Change_Right(track, 170, 40);
            
        // monotonicity_change_line[0] = Monotonicity_Change_Right(
        //     track, right_breakpoint + 100,
        //     right_breakpoint); // 拐点上方应该有单调突变点
        //     cout<<"单调突变点"<<monotonicity_change_line[0]<<endl;
        if (ringTypeTemp == RingNone) {
          if ( // 左边是单调的          //判断右入环
              lostline_right > 15 &&              // 丢线的数量
              continuity_change_left_flag == 0 && // 右环岛是左线连续的
              continuity_change_right_flag != 0 && // 右环岛右边是连续的
              track.validRowsRight >= 50 &&    // 右边有效行不能太少
              // track.validRowsRight <= 140 &&   // 右边有效行比较少
              track.validRowsLeft >= 100       // 左边丢线较少
              &&track.stdevLeft<50&&track.stdevRight>150) // 右边有单调突变点&& monotonicity_change_line[0]
          {
            monotonicity_left = Monotonicity_Left(
                track, right_breakpoint - 10,
                right_breakpoint + 10); // 判断拐点附近的严格的单调性
            cout<<"右环左边严格单调"<<monotonicity_left<<endl;
            if (monotonicity_left == 0
                ) // 右单调突变点左侧也连续 !Monotonicity_Left(track, monotonicity_change_line[0] + 10, monotonicity_change_line[0] -  10)
                                  
                                     
            {
              // right_breakpoint=0;
              ringStep = RingStep::IsRing;
              ringTypeTemp = RingRight; // 环岛类型：右入环
              distance_in = distance_now;
              ringEnable = true;
              // mpu6050_later = mpu6050_now;
              std::cout << "判断出右入环" << endl;
              } else // 误判，归零
              {
               ringStep == RingStep::None;
               cout << "误判环" << endl;
            } // 右边是单调的
          }
        }

        ////判断右入环
      }
    }
    // if(ringStep == RingStep::IsRing)
    // {
    //   RoundaboutGetArc(track, 1, 20, 30, 160);//左入环
    //   RoundaboutGetArc(track, 2, 20, 30, 160);//右入环
    // }
    
    int countWide = 0; // 环岛入口变宽区域行数
    for (int i = 1; i < track.widthBlock.size(); ++i) {
      if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
          track.widthBlock[i].y > COLSIMAGE * 0.6 &&
          track.widthBlock[i].x > 30 &&
          (((track.stdevLeft > 120 && track.stdevRight < 50) ||
            (track.stdevRight > 120 &&
             track.stdevLeft < 50)) && // 将左右值分别调换
           ringStep == RingStep::Entering)) // 搜索突然变宽的路径行数   Entering
      {
        ++countWide;
        cout << "countwide" << countWide << endl;
      } else {
        countWide = 0;
      }
    }

   if (ringStep == RingStep::IsRing) // 状态2入环前判断
    {
      // cout<<"左入环"<<ringTypeTemp<<endl;
      if (ringTypeTemp == RingLeft) // 左入环  对于类型问题换个方式？ ringTypeTemp == RingLeft
      {

        cout << "要进行entering的判断了" << endl;
        monotonicity_change_line[0] = Monotonicity_Change_Right(track, 30, 160);
        roundaboutArc=RoundaboutGetArc(track, 1, 10, 30, 160);
        monotonicity_right=Monotonicity_Right(track, monotonicity_change_line[0] + 20,
                monotonicity_change_line[0] -20);
        cout << "圆弧的判断" << (int)RoundaboutGetArc(track, 1, 10, 30, 160)<< endl;
        cout << "右边线连续的判断"<< Monotonicity_Right(track, monotonicity_change_line[0] + 10,monotonicity_change_line[0] - 10)<< endl;
        
           ///入环补线
        int breakpoint_in=0;
         breakpoint_in = Find_Left_Breakpoint(track, 20, 140); // 找到拐点
        cout<<"breakpoint_in"<<breakpoint_in<<endl;
        if(breakpoint_in<140&&breakpoint_in>20)
        {K_Add_Boundry_Left(regression(track.pointsEdgeLeft, breakpoint_in -2,breakpoint_in -6),
                           track.pointsEdgeLeft[breakpoint_in -2].y,
                          breakpoint_in-2, 140,track); // 根据斜率作直线
          cout<<"开始左环入环直线补线！"<<endl;}


        if ( roundaboutArc&&!monotonicity_right) { // 当左边不单调点较低，或者左侧的斜率较大
          ringStep = RingStep::Entering;
          // flagpid=1;
          mpu6050_later = mpu6050_now;
          cout << "////////////////////////////////左入环前" << endl;
        }

        else // 没有检测到圆弧点
        {
          if ((distance_now - distance_in) > 1800) {  // 原先为3000
            ringStep = RingStep::None;
            ringTypeTemp = RingNone;
            distance_in = 0;
            cout << "在IsRing 误判" << endl;
          }
        }
      } else if (ringTypeTemp == RingRight ) // 右入环  //右入环  &&   (distance_now - distance_in) > 500
                
      {

        cout << "右入环Entering判断" << endl;
        monotonicity_change_line[0] =Monotonicity_Change_Left(track, 30, 160); // 寻找单调性改变点，x
        roundaboutArc=RoundaboutGetArc(track, 2, 10, 30, 160);
        monotonicity_left=Monotonicity_Left(track,  monotonicity_change_line[0] + 20,monotonicity_change_line[0] - 20);
        
        cout<<"右圆弧判断"<<RoundaboutGetArc(track, 2, 10, 30, 160)<<endl;
        cout<<"右圆环左边单调性"<<!Monotonicity_Left(track,  monotonicity_change_line[0] + 10,monotonicity_change_line[0] - 10)<<endl;// 左边单调
        
        
        ////////////先判断再补线,不影响判断
        int breakpoint_in=0;
         breakpoint_in = Find_Right_Breakpoint(track, 20, 140); // 找到拐点
        if(breakpoint_in<140&&breakpoint_in>20)
        {K_Add_Boundry_Right(regression(track.pointsEdgeRight, breakpoint_in -2,breakpoint_in -6),
                           track.pointsEdgeRight[breakpoint_in -2].y,
                          breakpoint_in-2, 140,track); // 根据斜率作直线
          cout<<"开始右环入环直线补线！"<<endl;
          }

       
        if (roundaboutArc&&!monotonicity_left) { //RoundaboutGetArc(track, 2, 10, 10, 180) && 右边上面有拐点 && right_breakpoint)
          ringStep = RingStep::Entering;
          // flagpid=1;
          mpu6050_later = mpu6050_now;
          cout << "////////////////////////////////右入环前" << endl;
        }
        else {
          if ((distance_now - distance_in) > 1800) {
            ringStep = RingStep::None;
            ringTypeTemp = RingNone;
            distance_in = 0;
            cout << "在IsRing 误判" << endl;
          }
        }
      }
    }
    // 状态三 入环判断
    if ((ringStep == RingStep::Entering)) /// 有
    {
      // save_common_pid(motion);
      // set_ring_pid(ring_p1, ring_p2, ring_d, motion);
      // left_breakpoint =
      // Find_Left_Breakpoint(track, 80, 160);
      // left_breakpoint = Find_Left_Breakpoint(track, 20, 80);
      continuity_change_right_flag =
          Continuity_Change_Right(track, left_breakpoint - 10,
                                  left_breakpoint + 10); // 拐点附近判断连续性

      distance_diff = distance_now - distance_in;
      cout << "入环路程差值" << distance_diff << endl; // distance的路程积分
      left_breakpoint = Find_Left_Breakpoint(track, 20, 160);
      right_breakpoint=Find_Right_Breakpoint(track, 20,
                           160);
      cout<<"入环右角点的i值" << right_breakpoint<<endl;                   
      cout << "入环左角点的i值" << left_breakpoint << endl;
      if ((left_breakpoint > 50 && left_breakpoint < 155) ||
          (right_breakpoint > 50 && right_breakpoint < 150)) {//原先为130  95
        flagjiao = 1;
      }
      if (!track.spurroad.empty() && (distance_diff > 1000) &&  //积分距离有点大---可能会导致晚了---判断入环时出问题
          flagjiao) //    &&left_breakpoint     left_breakpoint < 50
                    //    !continuity_change_right_flag   原来路程积分1300
      {
        // cout << "进入状态三,入环判断拐点" << endl;
        for (int i = 1; i < track.widthBlock.size(); ++i) {
          // cout << "进入状态三,入环判断" << endl;
          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 5].y) {
            //ringTypeTemp = RingLeft; // 环岛类型：左入环
            colEnterRing = track.pointsEdgeLeft[i - 5].y; // 入环点列号
            _ringPoint.x = track.pointsEdgeLeft[i - 5].x;
            _ringPoint.y = track.pointsEdgeLeft[i - 5].y;

            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeLeft[i].x; // 用于环补线的列号

          } else if (track.pointsEdgeRight[i].y >
                     track.pointsEdgeRight[i - 5].y) {
           // ringTypeTemp = RingRight; // 环岛类型：右入环
            colEnterRing = track.pointsEdgeRight[i - 5].y; // 入环点列号
            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeRight[i].x; // 用于环补线的列号
          }

          // 内圆检测
          if ((ringTypeTemp == RingLeft &&
               colEnterRing - track.pointsEdgeLeft[i].y >= 3) ||
              (ringTypeTemp == RingRight &&
               track.pointsEdgeRight[i].y - colEnterRing >= 3))

          {
            // 判断入环成功
            // cout << "内圆检测" << endl;
            // mpu6050_later = mpu6050_now;

            ringEnable = true;
            // ringStep = RingStep::Entering;
            ringType = ringTypeTemp;
            if (rowRepairStraightside == track.widthBlock.size() - 1) {
              rowRepairStraightside = i - countWide;
            }
          } else {
            countWide = 0;
          }

          /*if(ringStep == RingStep::Entering && ringEnable == false){
              ringEnable = true;
              rowRepairStraightside = rowRepairLine;
          }*/

          if (ringEnable == true && ringStep == RingStep::Entering) {
            if (ringTypeTemp == RingLeft) { // 祖传没有右入环
              if (track.pointsEdgeLeft[i].y <= 2 &&
                  i != track.widthBlock.size() - 1) {
                if (rowRepairRingside == track.widthBlock.size() - 1) {
                  rowRepairRingside = i;
                }
                rowYendStraightside = track.pointsEdgeLeft[i].x;
              } else if (rowRepairRingside != track.widthBlock.size() - 1 &&
                         distance_diff > 1300)

              {
                // save_common_pid(motion);
                //  motion.set_direction_pid(motion.params.ring_p1,motion.params.ring_p2, motion.params.ring_d); 
                // flagpid=1;
                int x = track.pointsEdgeLeft[rowRepairStraightside].x +
                        (rowYendStraightside -
                         track.pointsEdgeRight[rowRepairStraightside].x) *
                            5 / 4;
                int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
                         track.pointsEdgeRight[rowRepairStraightside].y) /
                        2;

                POINT startPoint =
                    track.pointsEdgeRight[rowRepairStraightside]; // 补线：起点
                POINT midPoint(x, y);                   // 补线：中点
                POINT endPoint(rowYendStraightside, 0); // 补线：终点
                std::cout << "左环开始补线" << endl;
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeLeft.resize(rowRepairRingside);
                track.pointsEdgeRight.resize(rowRepairStraightside);
                for (int kk = 0; kk < b_modify.size(); ++kk) {
                  track.pointsEdgeRight.emplace_back(b_modify[kk]);
                }
                break;
              }
            } else if (ringTypeTemp == RingRight) { // 祖传没有右入环 此处还没改
              if (track.pointsEdgeRight[i].y >= 318 &&
                  i != track.widthBlock.size() - 1) {
                if (rowRepairRingside == track.widthBlock.size() - 1) {
                  rowRepairRingside = i;
                }
                rowYendStraightside = track.pointsEdgeRight[i].x;
              } else if (rowRepairRingside != track.widthBlock.size() - 1 &&
                         distance_diff > 1300)

              {

                int x = track.pointsEdgeRight[rowRepairStraightside].x +
                        (rowYendStraightside -
                         track.pointsEdgeLeft[rowRepairStraightside].x) *
                            5 / 4;//原来不减少   之后减少了50
                int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
                         track.pointsEdgeRight[rowRepairStraightside].y) /
                        4;//对于y值 的处理?原来除以2  
                cout<<"x的值是"<<x<<endl;
                cout<<"y的值是"<<y<<endl;
                cout<<"补线i"<<rowRepairStraightside<<endl;
                cout<<"直线"<<rowYendStraightside<<endl;
                POINT startPoint =
                    track.pointsEdgeLeft[rowRepairStraightside]; // 补线：起点
                POINT midPoint(x, y);                   // 补线：中点
                POINT endPoint(rowYendStraightside, 300); // 补线：终点  原来y为320  不减少   -20，250
                std::cout << "右环开始补线" << endl;
                vector<POINT> input = {startPoint, midPoint, endPoint};
                vector<POINT> b_modify = Bezier(0.01, input);
                track.pointsEdgeLeft.resize(rowRepairRingside);
                track.pointsEdgeRight.resize(rowRepairStraightside);
                for (int kk = 0; kk < b_modify.size(); ++kk) {
                  track.pointsEdgeLeft.emplace_back(b_modify[kk]);
                }
                break;
              }
            }
          }
        }
      }
    }
    if(ringStep == RingStep::Entering)  //&&distance_now - distance_in>1400
    {
      cout<<"设置了flagpid"<<endl<<endl<<endl;
        save_common_pid(motion);
      //  motion.set_direction_pid(motion.params.ring_p1,motion.params.ring_p2, motion.params.ring_d); 
        // flagpid=1;


    }
    /*********************状态三判断结束 */
    // int tmp_ttttt = 0;
    // if (ringEnable == false && ringStep == RingStep::Entering) {
    //   // 本场没判出环，且没有分叉
    //   if (!track.spurroad.empty() &&
    //       rowRepairLine < track.pointsEdgeRight.size() - 1 &&
    //       rowBreakpointRight > ROWSIMAGE / 2) {

    //       rowRepairStraightside = rowRepairLine;
    //       cout << "没判断出环布线" << endl;//出环补线
    //       if (ringType == RingLeft) {
    //       tmp_ttttt = 1;
    //       for (int i = rowRepairLine; i < track.pointsEdgeLeft.size() - 1;
    //            i++) {
    //         if (track.pointsEdgeLeft[i].y <= 2 &&
    //             i != track.widthBlock.size() - 1) {
    //           rowRepairRingside = i;
    //           break;
    //           // rowYendStraightside = track.pointsEdgeLeft[i].x;
    //         }
    //       }

    //       for (int i = rowRepairRingside; i < track.pointsEdgeLeft.size() - 1;
    //            i++) {
    //         if (track.pointsEdgeLeft[i].y <= 2 &&
    //             i != track.widthBlock.size() - 1) {
    //           rowYendStraightside = track.pointsEdgeLeft[i].x;
    //         } else if (rowRepairRingside != track.widthBlock.size() - 1) {
    //           int x = track.pointsEdgeLeft[rowRepairStraightside].x +
    //                   (rowYendStraightside -
    //                    track.pointsEdgeRight[rowRepairStraightside].x) *
    //                       5 / 4;
    //           int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
    //                    track.pointsEdgeRight[rowRepairStraightside].y) /
    //                   2;

    //           POINT startPoint =
    //               track.pointsEdgeRight[rowRepairStraightside]; // 补线：起点
    //           POINT midPoint(x, y);                   // 补线：中点
    //           POINT endPoint(rowYendStraightside, 0); // 补线：终点

    //           // for (int i = 0; i < track.spurroad.size(); i++)
    //           // {
    //           //     if (track.spurroad[i].y < startPoint.y &&
    //           //     track.spurroad[i].x < startPoint.x)
    //           //         endPoint = track.spurroad[i];
    //           //     break;
    //           // }

    //           vector<POINT> input = {startPoint, midPoint, endPoint};
    //           vector<POINT> b_modify = Bezier(0.02, input);
    //           track.pointsEdgeLeft.resize(rowRepairRingside);
    //           track.pointsEdgeRight.resize(rowRepairStraightside);

    //           for (int kk = 0; kk < b_modify.size(); ++kk) {
    //             track.pointsEdgeRight.emplace_back(b_modify[kk]);
    //           }
    //           break;
    //         }
    //       }
    //     }
    //   }

      

      
    //   //

    //   /*else {
    //     if (ringType == RingLeft &&
    //         track.pointsEdgeRight.size() > 1) {
    //       tmp_ttttt = 2;
    //       cout<<"没判断出环，有分叉"<<endl;
    //       int x_end = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
    //       for (int kkk =
    //                track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
    //            kkk <
    //            track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x + 50;
    //            kkk++) {
    //         if (imagePath.at<Vec3b>(kkk, 0)[2] > 0) {
    //           x_end = kkk;
    //           break;
    //         }
    //       }

    //       POINT startPoint(ROWSIMAGE - 10, COLSIMAGE - 1); // 补线：起点
    //       POINT endPoint(x_end, 0);                        // 补线：终点

    //       // for (int i = 0; i < track.spurroad.size(); i++)
    //       // {
    //       //     if (track.spurroad[i].y < startPoint.y && track.spurroad[i].x <
    //       //     startPoint.x)
    //       //         endPoint = track.spurroad[i];
    //       //     break;
    //       // }
    //       POINT midPoint =
    //           POINT((startPoint.x + endPoint.x) * 0.5,
    //                 (startPoint.y + endPoint.y) * 0.5); // 补线：中点
    //       vector<POINT> input = {startPoint, midPoint, endPoint};
    //       vector<POINT> b_modify = Bezier(0.02, input);
    //       track.pointsEdgeRight.resize(0);
    //       track.pointsEdgeLeft.resize(0);
    //       for (int kk = 0; kk < b_modify.size(); ++kk) {
    //         track.pointsEdgeRight.emplace_back(b_modify[kk]);
    //       }
    //     }///原来没注释掉------对于两种状态分析
    //   }*/
    // }
    if(ringStep == RingStep::Inside) cout<<"距离差值"<<distance_now-distance_in<<endl;
    // 状态4  环中，正常巡线
    if (ringStep == RingStep::Entering &&abs(mpu6050_now - mpu6050_later) >=30&&distance_now-distance_in<2100) {//原先为60度  && abs(mpu6050_now - mpu6050_later) >= 30  原来是2250
      ringStep = RingStep::Inside; // 纯粹陀螺仪积分到一定值就正常巡线
      // set_ring_pid(ring_p1, ring_p2, ring_d, motion);
       flagpid=1;
      cout<<"到小环了"<<endl<<endl<<endl;
      motion.set_direction_pid(motion.params.ring_p1s,motion.params.ring_p2s, motion.params.ring_ds,0);
    }
    else if  (ringStep == RingStep::Entering &&abs(mpu6050_now - mpu6050_later) >=30&&distance_now-distance_in>2100) {//原先为60度  && abs(mpu6050_now - mpu6050_later) >= 30
      ringStep = RingStep::Inside; // 纯粹陀螺仪积分到一定值就正常巡
      flagpid=1;
      cout<<"到大环了"<<endl<<endl<<endl;
      cout<<"两个状态间距离差值"<<distance_now-distance_in<<endl;
      //  if(distance_now-distance_in>2250)//rightpoint>110||leftpoint>110   //小环--2100   大环----2446
      //  {
      motion.set_direction_pid(motion.params.ring_p1b,motion.params.ring_p2b, motion.params.ring_db,0); //大环pid
      if(ringTypeTemp == RingRight)
      flagbigringr=1;//遇到大环时打开
      else if(ringTypeTemp == RingLeft)flagbigringl=1;
      // }
      // else 
      // {
      //   motion.set_direction_pid(motion.params.ring_p1s,motion.params.ring_p2s, motion.params.ring_ds); //小环pid
      // }
      cout << "已经到环中了" << endl;
    }
    // 出环补线
    if (ringStep == RingStep::Inside) {
      if(abs(mpu6050_now - mpu6050_later)>100&&abs(mpu6050_now - mpu6050_later)<220)//找固定打角
      {center_sum_flag=Center_Sum_Start; }
      else if(abs(mpu6050_now - mpu6050_later)>220)
      {center_sum_flag=Center_Sum_End;}

      // if (ringType == RingLeft) {
      //   int rowBreakRight = 0; // 右边缘横坐标连续性(行号)
      //   for (int i = 0; i < track.pointsEdgeRight.size(); i += 3) {
      //     if (track.pointsEdgeRight[i].y <=
      //         track.pointsEdgeRight[rowBreakRight].y) {
      //       rowBreakRight = i;
      //       continue;
      //     }
      //     if (i > rowBreakRight &&
      //         track.pointsEdgeRight[i].y -
      //                 track.pointsEdgeRight[rowBreakRight].y >
      //             5) {
      //       rowBreakpointRight = rowBreakRight;
      //       break; // 寻找到出环口：出环补线
      //     }
      //   }
      //   track.pointsEdgeLeft.resize(0); // 单边控制
      //   int acute_angle_flag = 0;
      //   if (!track.pointsEdgeRight.empty() &&
      //       track.pointsEdgeRight[rowBreakRight].y < COLSIMAGE / 4) {
      //     //cout << "前80列不用补线" << endl;
      //     track.pointsEdgeRight.resize(rowBreakRight); // 前80列不需要补线
      //   } 
      //   else if (track.pointsEdgeRight.size() - rowBreakRight >
      //              0) { // 下方两个同时输出，说明是此处制约时间  原来为20
      //                   // 可以将此处写死
      //     float slopeTop = 0;    // 斜率：分歧点上半部分
      //     float slopeButtom = 0; // 斜率：分歧点下半部分
      //     if (track.pointsEdgeRight[rowBreakRight].x !=
      //         track.pointsEdgeRight[0].x) {
      //       slopeButtom = (track.pointsEdgeRight[rowBreakRight].y -
      //                      track.pointsEdgeRight[0].y) *
      //                     100 /
      //                     (track.pointsEdgeRight[rowBreakRight].x -
      //                      track.pointsEdgeRight[0].x);
      //     }
      //     if (track.pointsEdgeRight[rowBreakRight].x !=
      //         track.pointsEdgeRight[rowBreakRight + 20].x) {
      //       slopeTop = (track.pointsEdgeRight[rowBreakRight + 20].y -
      //                   track.pointsEdgeRight[rowBreakRight].y) *
      //                  100 /
      //                  (track.pointsEdgeRight[rowBreakRight + 20].x -
      //                   track.pointsEdgeRight[rowBreakRight].x);
      //     }
      //     cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
      //          << endl;
      //     // if (slopeButtom * slopeTop <= 0) {
      //     //   cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
      //     //        << endl
      //     //        << endl;
      //     //   rowBreakpointLeft = track.pointsEdgeRight[track.validRowsLeft].x;
      //     //   cout << "左转出环补线1" << endl << endl << endl;
      //     //   // set_ring_pid(common_p1, common_p2, common_d, motion);
      //     //   // cout << "切换为普通pid" << endl;
      //     //   POINT p_end(rowBreakpointLeft - 60,
      //     //               300); // 补线终点为左边有效行顶点 //原先为不减，现在减少
      //     //   POINT p_mid(
      //     //       (track.pointsEdgeRight[rowBreakRight].x + rowBreakpointLeft) *
      //     //           3 / 8,
      //     //       track.pointsEdgeRight[rowBreakRight].y /
      //     //           8); // 将其减小--拉的更偏内些？  原来除以2
      //     //   POINT p_start(track.pointsEdgeRight[rowBreakRight].x,
      //     //                 track.pointsEdgeRight[rowBreakRight].y - 20);
      //     //   vector<POINT> input = {
      //     //       p_start, p_mid,
      //     //       p_end}; // track.pointsEdgeRight[rowBreakRight]原先为start
      //     //   vector<POINT> b_modify = Bezier(0.01, input);
      //     //   track.pointsEdgeRight.resize(rowBreakRight);
      //     //   for (int kk = 0; kk < b_modify.size(); ++kk) {
      //     //     track.pointsEdgeRight.emplace_back(b_modify[kk]);
      //     //   }
      //     // }
      //   }
      //   //  else if (track.pointsEdgeRight.size() - rowBreakRight <= 20 &&
      //   //             Find_Left_Breakpoint(track, 100, 180)) {
      //   //   cout << "出环补线2222" << endl << endl << endl;
      //   //   _index = 2;
      //   //   POINT p_end(rowBreakpointLeft, 0);
      //   //   POINT p_start(max(rowBreakpointRight, ROWSIMAGE - 80), COLSIMAGE);
      //   //   POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) / 4, COLSIMAGE / 2);
      //   //   vector<POINT> input = {p_start, p_mid, p_end};
      //   //   vector<POINT> b_modify = Bezier(0.01, input);
      //   //   track.pointsEdgeRight.resize(0);
      //   //   for (int kk = 0; kk < b_modify.size(); ++kk) {
      //   //     track.pointsEdgeRight.emplace_back(b_modify[kk]);
      //   //   }
      //   // }
      // } else if(ringType == RingRight){
      //       int rowBreakLeft = 0; // 右边缘横坐标连续性(行号)
      //   for (int i = 0; i < track.pointsEdgeLeft.size(); i += 3) {
      //     if (track.pointsEdgeLeft[i].y >=
      //         track.pointsEdgeLeft[rowBreakLeft].y) {
      //       rowBreakLeft = i;
      //       continue;
      //     }
      //     if (i > rowBreakLeft &&
      //         track.pointsEdgeLeft[rowBreakLeft].y-
      //         track.pointsEdgeLeft[i].y >5
         
      //             ) {
      //       rowBreakpointLeft = rowBreakLeft;
      //       break; // 寻找到出环口：出环补线
      //     }
      //   }
      //   track.pointsEdgeRight.resize(0); // 单边控制
      //   int acute_angle_flag = 0;
      //   if (!track.pointsEdgeLeft.empty() &&
      //       track.pointsEdgeLeft[rowBreakLeft].y > COLSIMAGE*3 / 4) {
      //     cout << "前80列不用补线" << endl;
      //     track.pointsEdgeLeft.resize(rowBreakLeft); // 前80列不需要补线
      //       }
      //   // } else if (track.pointsEdgeLeft.size() - rowBreakLeft >
      //   //            0) { // 下方两个同时输出，说明是此处制约时间  原来为20
      //   //                 // 可以将此处写死
      //   //   float slopeTop = 0;    // 斜率：分歧点上半部分
      //   //   float slopeButtom = 0; // 斜率：分歧点下半部分
      //   //   if (track.pointsEdgeLeft[rowBreakLeft].x !=
      //   //       track.pointsEdgeLeft[0].x) {
      //   //     slopeButtom = (
      //   //                    track.pointsEdgeLeft[0].y-track.pointsEdgeLeft[rowBreakLeft].y ) *
      //   //                   100 /
      //   //                   (track.pointsEdgeLeft[rowBreakLeft].x -
      //   //                    track.pointsEdgeLeft[0].x);
      //   //   }
      //   //   if (track.pointsEdgeLeft[rowBreakLeft].x !=
      //   //       track.pointsEdgeLeft[rowBreakLeft + 20].x) {
      //   //     slopeTop = (track.pointsEdgeLeft[rowBreakLeft].y-track.pointsEdgeLeft[rowBreakLeft + 20].y 
      //   //                 ) *
      //   //                100 /
      //   //                (track.pointsEdgeLeft[rowBreakLeft + 20].x -
      //   //                 track.pointsEdgeLeft[rowBreakLeft].x);
      //   //   }
      //   //   cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
      //   //        << endl;
      //   //   if (slopeButtom * slopeTop <= 0) {
      //   //     cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
      //   //          << endl
      //   //          << endl;
      //   //     rowBreakpointRight = track.pointsEdgeLeft[track.validRowsRight].x;
      //   //     cout << "右环出环补线1" << endl << endl << endl;
      //   //     // set_ring_pid(common_p1, common_p2, common_d, motion);
      //   //     // cout << "切换为普通pid" << endl;
      //   //     POINT p_end(rowBreakpointRight ,
      //   //                 340); // 补线终点为左边有效行顶点 //原先为不减，现在减少
      //   //     POINT p_mid(
      //   //         (track.pointsEdgeLeft[rowBreakLeft].x + rowBreakpointRight) *
      //   //             3 / 15,//原来除以8
      //   //         track.pointsEdgeLeft[rowBreakLeft].y /
      //   //             2); // 将其减小--拉的更偏内些？  原来除以2  之后除以4
      //   //     POINT p_start(track.pointsEdgeLeft[rowBreakLeft].x,
      //   //                   track.pointsEdgeLeft[rowBreakLeft].y );
      //   //     vector<POINT> input = {
      //   //         p_start, p_mid,
      //   //         p_end}; // track.pointsEdgeRight[rowBreakRight]原先为start
      //   //     vector<POINT> b_modify = Bezier(0.01, input);
      //   //     track.pointsEdgeLeft.resize(rowBreakLeft);
      //   //     for (int kk = 0; kk < b_modify.size(); ++kk) {
      //   //       track.pointsEdgeLeft.emplace_back(b_modify[kk]);
      //   //     }
      //   //   }
      //   // } else if (track.pointsEdgeLeft.size() - rowBreakLeft <= 20 &&
      //   //            Find_Left_Breakpoint(track, 100, 180)) {
      //   //   cout << "出环补线2222" << endl << endl << endl;
      //   //   _index = 2;
      //   //   POINT p_end(rowBreakpointLeft, 320);
      //   //   POINT p_start(max(rowBreakpointLeft,  80), 0);
      //   //   POINT p_mid((ROWSIMAGE - 50 + rowBreakpointRight) / 4, COLSIMAGE / 2);
      //   //   vector<POINT> input = {p_start, p_mid, p_end};
      //   //   vector<POINT> b_modify = Bezier(0.01, input);
      //   //   track.pointsEdgeLeft.resize(0);
      //   //   for (int kk = 0; kk < b_modify.size(); ++kk) {
      //   //     track.pointsEdgeLeft.emplace_back(b_modify[kk]);
      //   //   }
      //   // }
        
      // }

      if (abs(mpu6050_now - mpu6050_later )> 300) // 判断mpu6050然后再出环  280
      {
        // motion.set_direction_pid(common_p1, common_p2, common_d);
        cout << "切换为普通pid" << endl;
        // if (max(rowBreakpointLeft, rowBreakpointRight) < ROWSIMAGE / 2) {
        ringStep = RingStep::Exiting;
         flagbigringl=0;
         flagbigringr=0;
        flagpid=0;
        flagjiao = 0;
        center_sum_flag=Center_Sum_Reset;
        // distance_laterout = distance_now; // 开始路程积分
        cout << "开始路程积分" << endl;
        distance_final = distance_now;
        // }
      }
    }
    // 出环完成
    // 出环完成
    // else if(ringStep == RingStep::Exiting&&distance_now - distance_final>1000)ringStep == RingStep::None;
    else if (ringStep == RingStep::Exiting) {
      cout << "到达exiting了" << endl << endl << endl << endl;
      // 左出环补直线
      if (ringType == RingLeft) {
        int breakpoint_out;

         breakpoint_out = Find_Left_Breakpoint(track, 40, 180); // 找到拐点
        if(breakpoint_out<170&&breakpoint_out>40)
        {K_Add_Boundry_Left(regression(track.pointsEdgeLeft, breakpoint_out + 7,
                                      breakpoint_out + 2),
                           track.pointsEdgeLeft[breakpoint_out + 1].y,
                          breakpoint_out + 1,
                           30,track); // 根据斜率作直线
                           cout<<"开始左环出环直线补线！"<<endl;}
        
  // cout<<"dsjaiofjeoisdjfiodsjiofjadsiofjdsioa"<<   breakpoint_out<<endl<<endl<<endl;       // POINT p_end(rowBreakpointLeft, 0);
  //           // POINT p_start(ROWSIMAGE - 50, COLSIMAGE - 1);
            // POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) * 3 / 8,
            //             COLSIMAGE / 2);
            // vector<POINT> input = {p_start, p_mid, p_end};
            // vector<POINT> b_modify = Bezier(0.01, input);
            // track.pointsEdgeRight.resize(0);
            // track.pointsEdgeLeft.resize(0);

            //////////判断finish
        //     for (int kk = 0; kk < b_modify.size(); ++kk) {
        //   track.pointsEdgeRight.emplace_back(b_modify[kk]);
        // }
        if (distance_now - distance_final>1000||(distance_now - distance_final>700&&!Find_Left_Breakpoint(track, 40, 130))) {
          cout << "到达finish" << endl;
          ringStep = RingStep::Finish;
        }
      }

      // 右出环补直线
      if (ringType == RingRight) {
        int breakpoint_out;

         breakpoint_out = Find_Right_Breakpoint(track, 40, 180); // 找到拐点
        if(breakpoint_out<170&&breakpoint_out>40)


       { K_Add_Boundry_Right(regression(track.pointsEdgeRight,
                                      breakpoint_out + 10, breakpoint_out + 3),
                           track.pointsEdgeRight[breakpoint_out + 1].y,
                         breakpoint_out + 1,
                           30,track) ;// 根据斜率作直线
}
            // POINT p_end(rowBreakpointRight, 320);
            // POINT p_start(ROWSIMAGE - 50, 1);
            // POINT p_mid((ROWSIMAGE - 50 + rowBreakpointRight) * 3 / 8,
            //             COLSIMAGE / 2);
            // vector<POINT> input = {p_start, p_mid, p_end};
            // vector<POINT> b_modify = Bezier(0.01, input);
            // track.pointsEdgeRight.resize(0);
            // track.pointsEdgeLeft.resize(0);

            //////////判断finish
        //     for (int kk = 0; kk < b_modify.size(); ++kk) {
        //   track.pointsEdgeLeft.emplace_back(b_modify[kk]);
        // }
        if (distance_now - distance_final>1000||(distance_now - distance_final>700&&!Find_Right_Breakpoint(track, 40, 130))) {
          cout << "到达finish" << endl;
          ringStep = RingStep::Finish;
        }
      }
    }

    // 出环，切回正常循迹        //这里应该依靠路程积分彻底出环
    if (ringStep == RingStep::Finish) {
      motion.set_direction_pid(common_p1, common_p2, common_d,common_i);
      cout << "距离差值   " << distance_now - distance_final << endl;
      // if (track.pointsEdgeLeft.size() > 30 &&
      //     track.pointsEdgeRight.size() > 30 &&
      //     abs(track.pointsEdgeRight.size() - track.pointsEdgeLeft.size() <
      //         track.pointsEdgeRight.size() / 3) &&
      //     track.spurroad.empty() ||
      //     (distance_now - distance_final >= 200)) {
             // 结束出环补线  之前为1500
        ringStep = RingStep::None;
        center_sum_flag=Center_Sum_None;
           flagpid=0;
    left_breakpoint = 0;  // 左拐点行号
    right_breakpoint = 0; // 右拐点行号
    monotonicity_change_line[0]=0;
    monotonicity_change_line[1]=0;// 单调性改变点坐标，[0]寸某行，[1]寸某列
    monotonicity_right = 0; // 右侧单调
    monotonicity_left = 0;  // 左侧单调
    lostline_left = 0;      // 左右丢线数量
    lostline_right = 0;
    monotonicity_change_left_flag = 0;  // 不转折是0
    monotonicity_change_right_flag = 0; // 不转折是0
    continuity_change_right_flag = 0;   // 连续是0
    continuity_change_left_flag = 0;    // 连续是0
    //  first_ring = 0;
    //  left = 0;
    //  right = 0;
    ringTypeTemp = 0;
    flagjiao = 0; //
    cout<<"圆环完成"<<endl;
    //  leftpoint=0;
    //  rightpoint=0;
  
    reset();
   
          // }
    }
   

    if (track.spurroad.empty())
      counterSpurroad++;
    else
      counterSpurroad = 0;

    // 返回识别结果
    if (ringStep == RingStep::None)
      return false;
    else
      return true;
  }

  /**
   * @brief 绘制环岛识别图像
   *
   * @param ringImage 需要叠加显示的图像
   */
  void drawImage(Tracking track, Mat &ringImage) {
    for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(ringImage,
             Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x), 2,
             Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(ringImage,
             Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 2,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    for (int i = 0; i < track.spurroad.size(); i++) {
      circle(ringImage, Point(track.spurroad[i].y, track.spurroad[i].x), 5,
             Scalar(0, 0, 255), -1); // 红色拐点
    }
    // circle(ringImage,
    //        Point(track.pointsEdgeLeft[left_breakpoint].y,
    //              track.pointsEdgeLeft[left_breakpoint].x),
    //        5, Scalar(255, 152, 0), -1); // 我们自己的拐点
    putText(ringImage,
            to_string(_ringStep) + " " + to_string(_ringEnable) + " " +
                to_string(_tmp_ttttt),
            Point(COLSIMAGE - 80, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, 16);

    putText(ringImage, to_string(_index), Point(80, ROWSIMAGE - 20),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 0, 255), 1, 16);

    putText(ringImage,
            to_string(track.validRowsRight) + " | " +
                to_string(track.stdevRight),
            Point(COLSIMAGE - 100, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, 16);
    putText(ringImage,
            to_string(track.validRowsLeft) + " | " + to_string(track.stdevLeft),
            Point(30, ROWSIMAGE - 50), FONT_HERSHEY_TRIPLEX, 0.3,
            Scalar(0, 0, 255), 1, 16);

    switch (ringStep) {
    case First:
      putText(ringImage, "ring_First", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    case IsRing:
      putText(ringImage, "ring_IsRing", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    case Entering:
      putText(ringImage, "ring_Entering", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    case Inside:
      putText(ringImage, "ring_Inside", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    case Exiting:
      putText(ringImage, "ring_Exiting", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    case Finish:
      putText(ringImage, "ring_Finish", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      break;
    }
    circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0),
           -1); // 红色点
  }

  /*----------------------------------------自己写的函数---------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0，点所在的扫线的行0到220，
  Sample     Find_Right_Breakpoint(int start,int end);
  @note      角点检测阈值可根据实际值更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Find_Right_Breakpoint(Tracking &track, int start,
                            int end) // 找四个角点，返回值是角点所在的行数
  {
    int i, t;
    int right_down_line = 0;
    if (track.validRowsRight <=
        0.05 * ROWSIMAGE) // 大部分都丢线，没有拐点判断的意义
      return right_down_line;
    if (start < end) {
      t = start;
      start = end;
      end = t;
    }
    if (start >=
        ROWSIMAGE - 1 - 5) // 下面5行数据不稳定，不能作为边界点来判断，舍弃
      start = ROWSIMAGE - 1 - 5;
    if (end <= track.rowCutBottom)
      end = track.rowCutBottom;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) {
      if (right_down_line == 0 && // 只找第一个符合条件的点
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i + 1].y) <=
              3 && // 角点的阈值可以更改
          abs(track.pointsEdgeRight[i + 1].y -
              track.pointsEdgeRight[i + 2].y) <= 3 &&
          abs(track.pointsEdgeRight[i + 2].y -
              track.pointsEdgeRight[i + 3].y) <= 3 &&
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 2].y) >=
              20 &&
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 3].y) >=
              20 &&
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 4].y) >=
              20) {
        right_down_line = i; // 获取行数即可
        break;
      }
    }
    return right_down_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     左角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0，点所在的扫线的行0到220，
  Sample     Find_Right_Breakpoint(int start,int end);
  @note      角点检测阈值可根据实际值更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Find_Left_Breakpoint(Tracking &track, int start,
                           int end) // 找四个角点，返回值是角点所在的行数
  {
    int i, t;
    int right_down_line = 0;
    // cout<<"kaishi   "<<start<<"   jieshu"<<end<<endl;
    if (track.validRowsLeft <=
        0.05 * ROWSIMAGE) // 大部分都丢线，没有拐点判断的意义
    {
      // cout<<"右侧大部分丢线"<<endl<<endl;
      return right_down_line;
    }
    if (start < end) {
      t = start;
      start = end;
      end = t;
    }
    if (start >=
        ROWSIMAGE - 1 - 5) // 下面5行数据不稳定，不能作为边界点来判断，舍弃
      start = ROWSIMAGE - 1 - 5;
    if (end <= track.rowCutBottom)
      end = track.rowCutBottom;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) {
      if (right_down_line == 0 && // 只找第一个符合条件的点
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 1].y) <=
              3 && // 角点的阈值可以更改
          abs(track.pointsEdgeLeft[i + 1].y - track.pointsEdgeLeft[i + 2].y) <=
              3 &&
          abs(track.pointsEdgeLeft[i + 2].y - track.pointsEdgeLeft[i + 3].y) <=
              3 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 2].y) >=
              20 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 3].y) >=
              20 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 4].y) >=
              20) {
        // cout<<"左上角点有效行     "<<i<<endl;
        right_down_line = i; // 获取行数即可
        // cout<<"kaishipanduan"<<endl;
        break;
      }
    }
    return right_down_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     右赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数点所在的扫线的行0到220，
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Continuity_Change_Right(Tracking &track, int start, int end) {
    int i;
    int t;
    int continuity_change_flag = 0;
    if (track.validRowsRight <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没必要判断了，也就是左右的有效行数太少
      return 1;
    if (start >= ROWSIMAGE - 5) // 数组越界保护
      start = ROWSIMAGE - 5;
    if (end <= 5)
      end = 5;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }

    for (i = start; i >= end; i--) {
      if (abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 1].y) >=
          3) // 连续性阈值是5，可更改
      {
        cout << "右赛道不连续" << endl;
        continuity_change_flag = i;
        break;
      }
    }
    return continuity_change_flag;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     左赛道连续性检测
  @param     起始点，终止点
  @return    连续返回0，不连续返回断线出行数点所在的扫线的行0到220，
  Sample     continuity_change_flag=Continuity_Change_Right(int start,int end)
  @note      连续性的阈值设置为5，可更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Continuity_Change_Left(Tracking &track, int start, int end) {
    int i;
    int t;
    int continuity_change_flag = 0;
    if (track.validRowsLeft <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没必要判断了，也就是左右的有效行数太少
      return 1;
    if (start >= ROWSIMAGE - 5) // 数组越界保护
      start = ROWSIMAGE - 5;
    if (end <= 5)
      end = 5;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }

    for (i = start; i >= end; i--) {
      if (abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 1].y) >=
          3) // 连续性阈值是5，可更改
      {
        continuity_change_flag = i;
        break;
      }
    }
    return continuity_change_flag;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测
  @param     起始点，终止行
  @return    点所在的扫线的行0到220，，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
  -------------------------------------------------------------------------------------------------------------------*/
  int Monotonicity_Change_Right(
      Tracking &track, int start,
      int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int monotonicity_change_line = 0;

    if (track.validRowsRight <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没有单调性判断的意义
      return monotonicity_change_line;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    // if (start <= end)
    //   return monotonicity_change_line;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if (track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 8].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 8].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 7].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 7].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 6].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 6].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 4].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 4].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 2].y &&
          track.pointsEdgeRight[i].y ==
              track.pointsEdgeRight[i - 2]
                  .y) { // 一堆数据一样，显然不能作为单调转折点
        continue;
      } else if (
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 8].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 8].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 7].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 7].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 6].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 6].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 4].y &&
          track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i - 4].y &&
          track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i + 2].y &&
          track.pointsEdgeRight[i].y <=
              track.pointsEdgeRight[i - 2]
                  .y) { // 就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
        monotonicity_change_line = i;
        break;
      }
    }
    return monotonicity_change_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测左
  @param     起始点，终止行
  @return    点所在的扫线的行0到220，找不到返回0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      前5后5它最大（最小），那他就是角点
  -------------------------------------------------------------------------------------------------------------------*/
  int Monotonicity_Change_Left(
      Tracking &track, int start,
      int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int monotonicity_change_line = 0;
     int num = 0;
    int temp = 0;
    // cout<<"调用中心值函数开始值"<<start<<"结束值"<<end<<endl;
    if (track.validRowsLeft <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没有单调性判断的意义
    {
      return monotonicity_change_line;
      cout << "第一次return" << endl;
    }
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    // if (start <= end)
    // {
    //   return monotonicity_change_line;
    //   cout<<"第er次return2222222"<<endl;
    // }
    // if(track.spurroad.empty()&&)
    // {
    // cout<<"未发现岔路点qyq"<<endl;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
     // cout << "进入做单调判断" << endl;
      if (track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 7].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 7].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 6].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 6].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 5].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 5].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 4].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 4].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 2].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 2].y) {
       // cout << "左边缘点数值都一样了"
             //<< endl; // 一堆数据一样，显然不能作为单调转折点
             num++;
        continue;
      } else if (
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 7].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 7].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 8].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 8].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 6].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 6].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 4].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 4].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 2].y &&
          track.pointsEdgeLeft[i].y >=
              track.pointsEdgeLeft[i - 2]
                  .y) { // 就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
                        // //  点少了？
        monotonicity_change_line = i;
        //cout << "左边缘点坐标  " << i << endl;
        //cout << "找到中继点辣" << endl;
        break;
      }
      // }
    }
   if (num > (start - end) * 9 / 10)
      return 0; // 认为是单调的
    else
      return monotonicity_change_line; // 相应的单调性改变的点的i
  }
  /*-------------------------------------------------------------------------------------------------------------------
    @brief     严格单调右
    @param     起始点，终止行
    @return    点所在的扫线的行0到220，，单调就为0
    Sample     Find_Right_Up_Point(int start,int end);
    @note      上面的点小于下面的点，但是不能小太多
    -------------------------------------------------------------------------------------------------------------------*/
  int Monotonicity_Right(Tracking &track, int start,
                         int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int monotonicity_change_line = 0;
    int num = 0;
    int temp = 0;

    if (track.validRowsRight <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没有单调性判断的意义
      return monotonicity_change_line;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if (((track.pointsEdgeRight[i + 5].y - track.pointsEdgeRight[i + 4].y) <=
           -1) &&
          ((track.pointsEdgeRight[i + 4].y - track.pointsEdgeRight[i + 3].y) <=
           -1) &&
          ((track.pointsEdgeRight[i + 3].y - track.pointsEdgeRight[i + 2].y) <=
           -1) &&
          ((track.pointsEdgeRight[i + 2].y - track.pointsEdgeRight[i + 1].y) <=
           -1) &&
          ((track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 1].y) <=
           -1) &&
          ((track.pointsEdgeRight[i - 1].y - track.pointsEdgeRight[i - 2].y) <=
           -1) &&
          ((track.pointsEdgeRight[i - 3].y - track.pointsEdgeRight[i - 4].y) <=
           -1) &&
          ((track.pointsEdgeRight[i - 4].y - track.pointsEdgeRight[i - 5].y) <=
           -1)) { // 一堆数据一样，显然不能作为单调转折点
        num++;
        continue;
      } else if (
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 5].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 5].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 4].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 4].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 3].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i - 3].y &&
          track.pointsEdgeRight[i].y < track.pointsEdgeRight[i + 2].y &&
          track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i - 2].y &&
          track.pointsEdgeRight[i].y <= track.pointsEdgeRight[i + 1].y &&
          track.pointsEdgeRight[i].y <=
              track.pointsEdgeRight[i - 1]
                  .y) { // 就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
        monotonicity_change_line = i;
        // temp=i;
        //  break;
      }
    }
    if (num > (start - end) * 9 / 10)
      return 0; // 认为是单调的
    else
      return monotonicity_change_line; // 相应的单调性改变的点的i
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     严格单调左
  @param     起始点，终止行
  @return    点所在的扫线的行0到220，，单调就为0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      上面的点大于下面的点，但是不能小太多
  -------------------------------------------------------------------------------------------------------------------*/
  int Monotonicity_Left(Tracking &track, int start,
                        int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int monotonicity_change_line = 0;

    if (track.validRowsLeft <=
        0.1 * ROWSIMAGE) // 大部分都丢线，没有单调性判断的意义
      return monotonicity_change_line;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if (((track.pointsEdgeLeft[i + 5].y - track.pointsEdgeLeft[i + 4].y) >=
           1) &&
          ((track.pointsEdgeLeft[i + 4].y - track.pointsEdgeLeft[i + 3].y) >=
           1) &&
          ((track.pointsEdgeLeft[i + 3].y - track.pointsEdgeLeft[i + 2].y) >=
           1) &&
          ((track.pointsEdgeLeft[i + 2].y - track.pointsEdgeLeft[i + 1].y) >=
           1) &&
          ((track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 1].y) >= 1) &&
          ((track.pointsEdgeLeft[i - 1].y - track.pointsEdgeLeft[i - 2].y) >=
           1) &&
          ((track.pointsEdgeLeft[i - 3].y - track.pointsEdgeLeft[i - 4].y) >=
           1) &&
          ((track.pointsEdgeLeft[i - 4].y - track.pointsEdgeLeft[i - 5].y) >=
           1)) { // 一堆数据一样，显然不能作为单调转折点
        continue;
      } else if (
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 5].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 5].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 4].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 4].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 3].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 3].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 2].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 2].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 1].y &&
          track.pointsEdgeLeft[i].y >=
              track.pointsEdgeLeft[i - 1]
                  .y) { // 就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
        monotonicity_change_line = i;
        break;
      }
    }
    return monotonicity_change_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
 @brief     检查右侧丢线数量
 @param     起始点，终止行
 @return    点所在的扫线的行，返回丢线的行
 Sample     Find_Right_Up_Point(int start,int end);
 @note      上面的点大于下面的点，但是不能小太多
 -------------------------------------------------------------------------------------------------------------------*/
  int Lostline_Right(Tracking &track, int start,
                     int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int lostline_right = 0;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if (track.pointsEdgeRight[i].y >= COLSIMAGE - 1)
        lostline_right++;
        // cout<<"右边丢线函数"<<endl;
    }
    return lostline_right;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     检查左侧丢线数量
  @param     起始点，终止行
  @return    点所在的扫线的行0到180，，单调就为0
  Sample     Find_Right_Up_Point(int start,int end);
  @note      小于2就位丢线
  -------------------------------------------------------------------------------------------------------------------*/
  int Lostline_Left(Tracking &track, int start,
                    int end) // 单调性改变，返回值是单调性改变点所在的行数
  {
    int i;
    int t = 0;
    int lostline_left = 0;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    if (start >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start = ROWSIMAGE - 1 - 5;
    if (end <= 5)
      end = 5;
    for (i = start; i >= end; i--) // 会读取前5后5数据，所以前面对输入范围有要求
    {
      if (track.pointsEdgeLeft[i].y <= 2)
        lostline_left++;
    }
    return lostline_left;
  }
  /*!
   * @brief    判断左右边线是否存在弧形
   * 输出的 index 圆弧的顶点位置
   * @param    track ： 二值图像信息
   * @param    status     ： 1：左边线  2：右边线
   * @param    num        ： 圆弧的大小 用点数表示  （连续N个增  连续N个减）
   * @return   1 有弧线    0  没弧线
   */
  int RoundaboutGetArc(Tracking &track, int status, int num,
                           int start, int end) {
    int i = 0, t = 0;
    float inc = 0, dec = 0, n = 0;
    if (start < end) // 都是从下往上计算的，反了就互换一下
    {
      t = start;
      start = end;
      end = t;
    }
    switch (status) {
    case 1:
      for (i = start; i > end; i--) {
        // 没有丢线
        if (track.pointsEdgeLeft[i].y >= 2 &&
            track.pointsEdgeLeft[i + 1].y >= 2) {
          if (track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 1].y) {
            n++;
            continue;
          }
          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i + 1].y) {//
            inc++;
              inc+=n;
            n = 0;
          } else {
            dec++;
           dec+=n;
            n = 0;
           
          }
    // cout<<"左圆弧点递增数inc"<<inc<<"        ////左圆弧点递减数dec"<<dec<<endl;
          /* 有弧线 */
          if (inc > num && dec > (num/2)) {
            cout << "成功判断弧线" << endl;
            return 1;
          }
        } else {
          inc = 0;
          dec = 0;
          n = 0;
        }
      }
      // leftpoint=inc;
      break;

    case 2:
      for (i = start; i > end; i--) {
        if (track.pointsEdgeRight[i].y < ROWSIMAGE - 1 &&
            track.pointsEdgeRight[i + 1].y < ROWSIMAGE - 1) {
          if (track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 1].y) {
            n++;
            continue;
          }
          if (track.pointsEdgeRight[i].y > track.pointsEdgeRight[i + 1].y) {
            inc++;
              inc+=n;
          
            n = 0;
            
          } else {
            dec++;
            dec+=n;
            n = 0;
            
          }
          // cout<<"右圆弧点递增数inc"<<inc<<"        ////右圆弧点递减数dec"<<dec<<endl;
          /* 有弧线 */
          if (inc > num && dec > (num/2)) {
            // *index = i + num;
            return 1;
          }
        } else {
          inc = 0;
          dec = 0;
          n = 0;
        }
      }
      // rightpoint=dec;
      break;
    }
    
    return 0;
  }
/**
   * @brief 边缘斜率计算部分
   *
   * @param v_edge
   * @param img_height
   * @param start   
   * @param end
   * @return double
   */
  double Part_stdevEdgeCal(vector<POINT> &v_edge, int img_height,int start_line,int end_line) {
      int i = 0, t = 0;
    if (start_line < end_line) // 都是从下往上计算的，反了就互换一下
    {
      t = start_line;
      start_line = end_line;
      end_line = t;
    }    
    if (start_line >= ROWSIMAGE - 1 - 5) // 数组越界保护
      start_line = ROWSIMAGE - 1 - 5;
    if (end_line <= 5)
      end_line = 5;
    if (v_edge.size() < img_height / 4) {
      return 1000;
    }
    vector<int> v_slope;
    int step =2 ; ///这是间隔// v_edge.size()/10;
    for (int i = end_line; i < start_line; i += step) {
      if (v_edge[i].x - v_edge[i - step].x)
        v_slope.push_back((v_edge[i].y - v_edge[i - step].y) * 4 /
                          (v_edge[i].x - v_edge[i - step].x));
    }
    if (v_slope.size() > 1) {
      double sum = accumulate(begin(v_slope), end(v_slope), 0.0);
      double mean = sum / v_slope.size(); // 均值
      double accum = 0.0;
      for_each(begin(v_slope), end(v_slope),
               [&](const double d) { accum += (d - mean) * (d - mean); });

      return sqrt(accum / (v_slope.size() - 1)); // 方差
    } else
      return 0;
  }
      /************************************线性回归计算斜率************************************/
/*
 *  @brief 最小二乘法拟合直线斜率
 * @param 输入点集
 * @param startline 开始的行数
 * @param endline 结束的行数
 * @return 返回拟合的斜率
 */
float regression(vector<POINT> &v_edge, int startline, int endline) {

  int i = 0, SumX = 0, SumY = 0, SumLines = 0;
  float SumUp = 0, SumDown = 0, avrX = 0, avrY = 0, B, A;
  int t = 0;
  if (startline > endline) // 从下往上计算的，反了就互换一下
  {
    t = startline;
    startline = endline;
    endline = t;
  }
  SumLines =
      endline - startline; // startline 为开始行， //endline 结束行 //SumLines

  for (i = startline; i < endline; i++) {
    SumX += i;
    SumY += v_edge[i].y; // 这里Middle_black为存放中线的数组
  }
  avrX = SumX / SumLines; // X的平均值
  avrY = SumY / SumLines; // Y的平均值
  SumUp = 0;
  SumDown = 0;
  for (i = startline; i < endline; i++) {
    SumUp += (v_edge[i].y - avrY) * (i - avrX);
    SumDown += (v_edge[i].y - avrY)*(v_edge[i].y - avrY)  ;
  }
  if (SumDown == 0)
    B = 0;
  else
    B = (SumUp / SumDown);
  A = (SumY - B * SumX) / SumLines; // 截距
  return B;                         // 返回斜率
}

/*-------------------------------------------------------------------------------------------------------------------
@brief     通过斜率，左边线定点补线--
@param     k       输入斜率
           startY  输入起始点纵坐标
           startI  输入起始点i
           endI    结束点i
@return    null
Sample     K_Add_Boundry_Left(float k,int startY,startI,int endI);
@note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Left(float k, int startY, int startI, int endI,Tracking &track) {
  int i = 0, t = 0;
  if (startY >= COLSIMAGE) // 减去切行
    startY = COLSIMAGE;
  else if (startY <= 0)
    startY = 0;
  if (endI >= track.pointsEdgeLeft.size() - 1)
    endI = track.pointsEdgeLeft.size() - 1;
  else if (endI <= 0)
    endI = 0;
    // track.pointsEdgeLeft.resize(track.pointsEdgeLeft.size() + 1);
  if (startI < endI) //--操作，start需要大
  {for (i = startI; i <= endI; i++) 
    track.pointsEdgeLeft[i].y =
        (int)((i - startI) / k + startY); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    if (track.pointsEdgeLeft[i].y >= COLSIMAGE - 1) {
      track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
    } else if (track.pointsEdgeLeft[i].y <= 0) {
      track.pointsEdgeLeft[i].y= 0;
    }
  }
  
  // 这里有bug，下方循环--循环，需要start要大，只进行y的互换，但是没有进行x的互换
  // 建议进行判断，如果start更小，那就进行++访问
  // 这里修改各位自行操作
  else 
  {for (i = startI; i >= endI; i--) {
    track.pointsEdgeLeft[i].y =
        (int)((i - startI) / k + startY); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    if (track.pointsEdgeLeft[i].y >= COLSIMAGE - 1) {
      track.pointsEdgeLeft[i].y = COLSIMAGE - 1;
    } else if (track.pointsEdgeLeft[i].y <= 0) {
      track.pointsEdgeLeft[i].y= 0;
    }
  }}
}
/*-------------------------------------------------------------------------------------------------------------------
 @brief     通过斜率，右边线定点补线--
 @param     k       输入斜率
            startY  输入起始点纵坐标
            startI  输入起始点横坐标
            endI    结束点纵坐标
 @return    null
 Sample     K_Add_Boundry_Right(float k,int startY,startI,int endI);
 @note      补得线直接贴在边线上
-------------------------------------------------------------------------------------------------------------------*/
void K_Add_Boundry_Right(float k, int startY, int startI, int endI,Tracking &track) {
  int i = 0, t = 0;
  if (startY >= COLSIMAGE) // 减去切行
    startY = COLSIMAGE;
  else if (startY <= 0)
    startY = 0;
  if (endI >= track.pointsEdgeRight.size() - 1)
    endI = track.pointsEdgeRight.size() - 1;
  else if (endI <= 0)
    endI = 0;
  track.pointsEdgeRight.resize(track.pointsEdgeRight.size() + 1);
  if (startI < endI) //--操作，start需要大
  {
      for (i = startI; i <= endI; i++) {
    track.pointsEdgeRight[i].y =
        (int)((i - startI) / k + startY); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    if (track.pointsEdgeRight[i].y >= COLSIMAGE - 1) {
      track.pointsEdgeRight[i].y = COLSIMAGE - 1;
    } else if (track.pointsEdgeRight[i].y <= 0) {
      track.pointsEdgeRight[i].y = 0;
    }
  }
  }
  // 这里有bug，下方循环--循环，需要start要大，只进行y的互换，但是没有进行x的互换
  // 建议进行判断，如果start更小，那就进行++访问
  // 这里修改各位自行操作
  else
  {for (i = startI; i >= endI; i--) {
    track.pointsEdgeRight[i].y =
        (int)((i - startI) / k + startY); //(y-y1)=k(x-x1)变形，x=(y-y1)/k+x1
    if (track.pointsEdgeRight[i].y >= COLSIMAGE - 1) {
      track.pointsEdgeRight[i].y = COLSIMAGE - 1;
    } else if (track.pointsEdgeRight[i].y <= 0) {
      track.pointsEdgeRight[i].y = 0;
    }
  }}
}
private:
  float mpu6050_later;
  float mpu6050_now;

  uint16_t counterSpurroad = 0; // 岔路计数器
  // 临时测试用参数
  int _ringStep;
  int _ringEnable;
  int _tmp_ttttt;
  int _index = 0;
  POINT _ringPoint = POINT(0, 0);

  /**
   * @brief 环岛类型
   *
   */

  /**
   * @brief 环岛运行步骤/阶段
   *
   */
  enum RingStep {
    None = 0, // 未知类型
    First,    // 状态一
    IsRing,   // 状态二，入环之前在判断
    Entering, // 状态三入环
    Inside,   // 状态四环中
    Exiting,  // 状态五出环
    Finish    // 环任务结束
  };
  // enum RingType22 {
  //   RingNone1 = 0, // 未知类型
  //   RingLeft1,     // 左入环岛
  //   RingRight1     // 右入环岛
  // };
  int ringType = RingLeft;            // 环岛类型
  RingStep ringStep = RingStep::None; // 环岛处理阶段
  int rowRepairLine = 0;              // 用于环补线的点（行号）
  int colRepairLine = 0;              // 用于环补线的点（列号）
};
