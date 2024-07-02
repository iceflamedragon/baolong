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
  float p1;
  float p2;
  float d;
  Crossroad c1;

public:
  void setmpu6050(float mpu6050_now_read) { mpu6050_now = mpu6050_now_read; }
  uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

  /**
   * @brief 环岛识别初始化|复位
   *
   */
  static int ru;
  void reset(void) {
    RingType ringType = RingType::RingLeft; // 环岛类型
    RingStep ringStep = RingStep::None;     // 环岛处理阶段
    int rowRepairLine = 0;                  // 用于环补线的点（行号）
    int colRepairLine = 0;                  // 用于环补线的点（列号）
    counterSpurroad = 0;                    // 岔路计数器
    counterShield = 0;
  }
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
    p1 = motion.params.ring_p1;
    p2 = motion.params.ring_p2;
    d = motion.params.ring_d;
    bool ringEnable = false;                    // 判环标志
    RingType ringTypeTemp = RingType::RingNone; // 环岛类型：临时变量
    int rowBreakpointLeft = 0; // 边缘线拐点起始行（左），这是从边缘线上提取的
    int rowBreakpointRight = 0; // 边缘线拐点起始行（右）这是从边缘线上提取的
    int colEnterRing = 0; // 入环点（图像列序号）
    int rowRepairRingside =
        track.widthBlock.size() - 1; // 环一侧，补线起点（行号）
    int rowRepairStraightside =
        track.widthBlock.size() - 1; // 直道侧，补线起点（行号）
    int rowYendStraightside =
        track.widthBlock.size() - 1; // 直道侧，延长补线终点（行号）
    _index = 0;
    _ringPoint = POINT(0, 0);

    // 算环用布线的候选点
    rowRepairLine = max(rowRepairLine - 5, 0);
    if (ringStep == RingStep::Entering && !track.spurroad.empty()) {
      if (ringType == RingType::RingLeft && track.pointsEdgeLeft.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeLeft.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeLeft[j].x >= track.spurroad[0].x;
             j++) {
          if (track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j - 10].y &&
              track.pointsEdgeLeft[j].y > track.pointsEdgeLeft[j + 10].y) {
            rowRepairLine = j;
            break;
          }
        }
      } else if (ringType == RingType::RingRight &&
                 track.pointsEdgeRight.size() > 20) {
        for (int j = max(rowRepairLine - 30, 10);
             j < track.pointsEdgeRight.size() - 10 && j < rowRepairLine + 30 &&
             track.pointsEdgeRight[j].x >= track.spurroad[0].x;
             j++) {
          if (track.pointsEdgeRight[j].y < track.pointsEdgeRight[j - 10].y &&
              track.pointsEdgeRight[j].y < track.pointsEdgeRight[j + 10].y) {
            rowRepairLine = j;
            break;
          }
        }
      }
    }

    // 搜索赛道左右边缘满足图像边沿的最高处
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
    int xielv = 0;
    // 判环
    static int countWide = 0; // 环岛入口变宽区域行数,是变宽的过程
    static int flag = 0;
    static int time = 0;     // 出环延时
    static int delay1 = 0;   // 入环延时
    static int flaghuan = 0; // 出环的时候别再第二次进环
    static int flagcha = 0;  // 识别到岔路的数量
                             // static int comein=0;
    for (int i = 1; i < track.widthBlock.size();
         ++i) { //////////////////i从图像下面开始增加
      if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
          track.widthBlock[i].y > COLSIMAGE * 0.5 // 赛道的宽度足够大
          && track.widthBlock[i].x > 30 &&
          ((track.stdevLeft > 80 &&
            track.stdevRight < 50) || // 第一个值开始是大于100
           ringStep == RingStep::Entering)) {
        xielv++;
        cout << "xielv1" << xielv << endl;
      } ///////////这里可以加个else if写右入环
      else if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
               track.widthBlock[i].y > COLSIMAGE * 0.5 // 赛道的宽度足够大
               && track.widthBlock[i].x > 30 &&
               ((track.stdevLeft < 50 && track.stdevRight > 100) ||
                ringStep == RingStep::Entering)) {
        xielv++;
        cout << "xielv" << xielv << endl;
      }

      if (xielv > 3) { // 检测赛道不断变窄
        if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
            track.widthBlock[i].y >
                COLSIMAGE * 0.5 // 赛道的宽度足够大   i取图中前方的位置
            && track.widthBlock[i].x > 30 && ///////////////进入的距离判断
            ((track.stdevLeft > 80 &&
              track.stdevRight <
                  50) || //////////////////原来第一个条件是大于100
             ringStep ==
                 RingStep::
                     Entering)) // 搜索突然变宽的路径行数，根据两边的斜率判断
        {
          ++countWide;
          cout << "countwide" << countWide
               << endl; // 从左边看从120--80  只要判断为圆环就进入环的状态了
        } /* else {
           countWide = 0;
         }*/
      }
      // [1] 入环判断
      // cout<<"countwide"<<countWide<<endl;
      /*if(!track.spurroad.empty())
      {
        flagcha++;
      }*/
      if (countWide >
          50) // 上方大于100时这里是大于20
              // 放宽条件，为了识别到圆环，识别到之后根据岔路点进行延时
      {
        flag = 1; // 内存爆掉---找中间量，立flag；
      }

      // if (c1.)//CrossroadType::crossroadType == CrossroadType::CrossroadLeft
      if (!track.spurroad.empty() && flag == 1 && track.stdevRight < 30 &&
          track.stdevLeft >
              30) // 防止十字处误判为圆环---将右斜率改变+将延时的条件改变
      {
        delay1++;
        cout << "第一次延时" << delay1
             << endl; // 延时1500刚刚好   此处延时的目的是碰到岔路之后的延时
                      // 原先为60
      }

      // static int omeicn=countWide;
      //  static int loca=0;//之后要在程序中置0，一般判断为入环后可以维持状态
      /*if(!track.spurroad.empty())
      {

      loca++;
      cout<<"有岔路岔路岔路"<<loca<<endl;//在第一个路口加到180左右
      }*/

      // cout<<"loca"<<loca<<endl;
      // cout<<"countwide"<<countWide<<endl;
      if ((ringStep == RingStep::None || ringStep == RingStep::Entering) &&
          (flag == 1 && delay1 > 350 && flaghuan == 0 &&
           !track.spurroad.empty() &&
           track.stdevRight <
               30)) { // 判断第一次  之后的两个条件出现的位置不匹配  if ((
                      // ringStep == RingStep::None ||ringStep ==
                      // RingStep::Entering) &&////////
        ////(countWide >=3 && !track.spurroad.empty())) {//判断第一次
        /// 之后的两个条件出现的位置不匹配
        /// 加的斜率方差的条件((210<track.stdevLeft&&track.stdevLeft
        ///< 240)||(140<track.stdevLeft&&track.stdevLeft
        ///< 180))||(40<track.stdevLeft&&track.stdevLeft <80) )&&
        // loca=0;
        countWide = 0;
        // flagcha=0;
        cout << "第一次判断成功" << endl;
        /*if (ringTypeTemp == RingType::RingNone) // 环岛方向判定    原先为条件
        {
//第一次入环判断
          int tmp_flag = 0;
          for (int j = 0; j < track.spurroad.size(); j++) {
            if (track.spurroad[j].x < track.pointsEdgeLeft[i - 5].x) {
              tmp_flag = 1;
            }
          }
          if (tmp_flag == 0) {

            countWide = 0;//原来为0
            cout<<"因为tmpflag所以continue"<<endl;//如果入环时间太早会触发continue使得无法入环
            continue;
          }

      }*/
        cout << "圆环第一次判断成功" << endl; // if( !track.spurroad.empty()){
        if (1) { // 此处为啥出问题？？?   track.pointsEdgeLeft[i].y <
                 // track.pointsEdgeLeft[i - 5].y

          flaghuan = 1; // 出环的时候别再第二次进环
          ringTypeTemp = RingType::RingLeft; // 环岛类型：左入环
          cout << "进入左环" << endl;
          colEnterRing = track.pointsEdgeLeft[i - 5].y; // 入环点列号
          _ringPoint.x = track.pointsEdgeLeft[i - 5].x;
          _ringPoint.y = track.pointsEdgeLeft[i - 5].y;

          rowRepairLine = i;                         // 用于环补线的行号
          colRepairLine = track.pointsEdgeLeft[i].x; // 用于环补线的列号
          xielv = 0;                                 // 清零标志位
        } else if (track.pointsEdgeRight[i].y >
                   track.pointsEdgeRight[i - 5].y) {
          ringTypeTemp = RingType::RingRight; // 环岛类型：右入环
          colEnterRing = track.pointsEdgeRight[i - 5].y; // 入环点列号
          rowRepairLine = i; // 用于环补线的行号
          colRepairLine = track.pointsEdgeRight[i].x; // 用于环补线的列号
          xielv = 0;                                  // 清零标志位
        }

        // 内圆检测
        if (ringTypeTemp ==
            RingType::RingLeft) { //(ringTypeTemp == RingType::RingLeft &&
          // colEnterRing - track.pointsEdgeLeft[i].y >= 0) ||
          // //原来为大于等于三，改为0
          //(ringTypeTemp == RingType::RingRight &&
          // track.pointsEdgeRight[i].y - colEnterRing >= 3)
          // 判断入环成功
          cout << "要入环辣" << endl;
          motion.set_direction_pid(p1, p2, d); // 调整圆环pid
          mpu6050_later = mpu6050_now;         // 检测mpu6050
          ringEnable = true;
          ringStep = RingStep::Entering;
          ringType = ringTypeTemp;
          if (rowRepairStraightside == track.widthBlock.size() - 1) {
            rowRepairStraightside = i - countWide; // 原先为减countWide
          }
        } else {
          countWide = 0;
        }
      }

      if ((ringStep == RingStep::None ||
           ringStep == RingStep::Entering) && ////////
          countWide > 15000) // 判断第二次    第二个判断成功入环，大于15
      {
        cout << "第二次判断" << endl;
        if (max(rowBreakpointLeft, rowBreakpointRight) < ROWSIMAGE) {
          cout << "发现拐点" << endl;
          // 第二次判断

          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 5].y) {

            ringTypeTemp = RingType::RingLeft; // 环岛类型：左入环
            colEnterRing = track.pointsEdgeLeft[i - 5].y; // 入环点列号
            _ringPoint.x = track.pointsEdgeLeft[i - 5].x;
            _ringPoint.y = track.pointsEdgeLeft[i - 5].y;

            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeLeft[i].x; // 用于环补线的列号
            xielv = 0;                                 // 清零标志位
            cout << "判断为RingLeft111111111111111111111111111111" << endl;
          } else if (track.pointsEdgeRight[i].y >
                     track.pointsEdgeRight[i - 5].y) {
            ringTypeTemp = RingType::RingRight; // 环岛类型：右入环
            colEnterRing = track.pointsEdgeRight[i - 5].y; // 入环点列号
            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeRight[i].x; // 用于环补线的列号
            xielv = 0;                                  // 清零标志位
            cout << "RingRight" << endl;
          }
          // 内圆检测
          if ((ringTypeTemp == RingType::RingLeft &&
               colEnterRing - track.pointsEdgeLeft[i].y >= 3) ||
              (ringTypeTemp == RingType::RingRight &&
               track.pointsEdgeRight[i].y - colEnterRing >= 3)) {
            // 判断入环成功
            motion.set_direction_pid(p1, p2, d); // 调整圆环pid
            mpu6050_later = mpu6050_now;         // 检测mpu6050
            ringEnable = true;
            cout << "要入环辣2222222222" << endl;
            ringStep = RingStep::Entering;
            ringType = ringTypeTemp;
            if (rowRepairStraightside == track.widthBlock.size() - 1) {
              rowRepairStraightside = i - countWide;
            }
          } else {
            countWide = 0;
          }
        }
      }
      /*if(ringStep == RingStep::Entering && ringEnable == false){
          ringEnable = true;
          rowRepairStraightside = rowRepairLine;
      }*/

      if (ringEnable == true && ringStep == RingStep::Entering) {
        if (ringTypeTemp == RingType::RingLeft) {
          if (track.pointsEdgeLeft[i].y <= 2 &&
              i != track.widthBlock.size() - 1) {
            if (rowRepairRingside == track.widthBlock.size() - 1) {
              rowRepairRingside = i;
            }
            rowYendStraightside = track.pointsEdgeLeft[i].x;
          } else if (rowRepairRingside != track.widthBlock.size() - 1) {

            int x = track.pointsEdgeLeft[rowRepairStraightside].x +
                    (rowYendStraightside -
                     track.pointsEdgeRight[rowRepairStraightside].x) *
                        5 / 4;
            int y = (track.pointsEdgeLeft[rowRepairStraightside].y +
                     track.pointsEdgeRight[rowRepairStraightside].y) /
                    2;

            POINT startPoint =
                track.pointsEdgeRight[rowRepairStraightside]; // 补线：起点
            POINT midPoint(x, y);                             // 补线：中点
            POINT endPoint(rowYendStraightside, 0);           // 补线：终点
            cout << "开始补线" << endl;
            vector<POINT> input = {startPoint, midPoint, endPoint};
            vector<POINT> b_modify = Bezier(0.01, input);
            track.pointsEdgeLeft.resize(rowRepairRingside);
            track.pointsEdgeRight.resize(rowRepairStraightside);
            for (int kk = 0; kk < b_modify.size(); ++kk) {
              track.pointsEdgeRight.emplace_back(b_modify[kk]);
            }
            break;
          }
        }
      }
    }

    int tmp_ttttt = 0;
    if (ringEnable == false && ringStep == RingStep::Entering) {
      // 本场没判出环，且没有分叉
      if (!track.spurroad.empty() &&
          rowRepairLine < track.pointsEdgeRight.size() - 1 &&
          rowBreakpointRight > ROWSIMAGE / 2) {

        rowRepairStraightside = rowRepairLine;

        if (ringType == RingType::RingLeft) {
          tmp_ttttt = 1;
          for (int i = rowRepairLine; i < track.pointsEdgeLeft.size() - 1;
               i++) {
            if (track.pointsEdgeLeft[i].y <= 2 &&
                i != track.widthBlock.size() - 1) {
              rowRepairRingside = i;
              break;
              // rowYendStraightside = track.pointsEdgeLeft[i].x;
            }
          }

          for (int i = rowRepairRingside; i < track.pointsEdgeLeft.size() - 1;
               i++) {
            if (track.pointsEdgeLeft[i].y <= 2 &&
                i != track.widthBlock.size() - 1) {
              rowYendStraightside = track.pointsEdgeLeft[i].x;
            } else if (rowRepairRingside != track.widthBlock.size() - 1) {
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

              // for (int i = 0; i < track.spurroad.size(); i++)
              // {
              //     if (track.spurroad[i].y < startPoint.y &&
              //     track.spurroad[i].x < startPoint.x)
              //         endPoint = track.spurroad[i];
              //     break;
              // }

              vector<POINT> input = {startPoint, midPoint, endPoint};
              vector<POINT> b_modify = Bezier(0.02, input);
              track.pointsEdgeLeft.resize(rowRepairRingside);
              track.pointsEdgeRight.resize(rowRepairStraightside);

              for (int kk = 0; kk < b_modify.size(); ++kk) {
                track.pointsEdgeRight.emplace_back(b_modify[kk]);
              }
              break;
            }
          }
        }
      }
      // 本场没判出环，有分叉
      else {
        if (ringType == RingType::RingLeft &&
            track.pointsEdgeRight.size() > 1) {
          tmp_ttttt = 2;
          int x_end = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
          for (int kkk =
                   track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x;
               kkk <
               track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x + 50;
               kkk++) {
            if (imagePath.at<Vec3b>(kkk, 0)[2] > 0) {
              x_end = kkk;
              break;
            }
          }

          POINT startPoint(ROWSIMAGE - 10, COLSIMAGE - 1); // 补线：起点
          POINT endPoint(x_end, 0);                        // 补线：终点

          // for (int i = 0; i < track.spurroad.size(); i++)
          // {
          //     if (track.spurroad[i].y < startPoint.y && track.spurroad[i].x <
          //     startPoint.x)
          //         endPoint = track.spurroad[i];
          //     break;
          // }
          POINT midPoint =
              POINT((startPoint.x + endPoint.x) * 0.5,
                    (startPoint.y + endPoint.y) * 0.5); // 补线：中点
          vector<POINT> input = {startPoint, midPoint, endPoint};
          vector<POINT> b_modify = Bezier(0.02, input);
          track.pointsEdgeRight.resize(0);
          track.pointsEdgeLeft.resize(0);
          for (int kk = 0; kk < b_modify.size(); ++kk) {
            track.pointsEdgeRight.emplace_back(b_modify[kk]);
          }
        }
      }
    }
    // 环中
    if (ringStep == RingStep::Entering && track.spurroad.empty() &&
        counterSpurroad >= 3) {
      ringStep = RingStep::Inside;
      cout << "目前状态Inside22222222" << endl;
    }
    // 出环补线
    if (ringStep == RingStep::Inside) {
      if (ringType == RingType::RingLeft) {
        int rowBreakRight = 0; // 右边缘横坐标连续性(行号)
        for (int i = 0; i < track.pointsEdgeRight.size(); i += 3) {
          if (track.pointsEdgeRight[i].y <=
              track.pointsEdgeRight[rowBreakRight].y) {
            rowBreakRight = i;
            continue;
          }
          if (i > rowBreakRight &&
              track.pointsEdgeRight[i].y -
                      track.pointsEdgeRight[rowBreakRight].y >
                  5) {
            rowBreakpointRight = rowBreakRight;
            break; // 寻找到出环口：出环补线
          }
        }
        track.pointsEdgeLeft.resize(0); // 单边控制
        int acute_angle_flag = 0;
        if (!track.pointsEdgeRight.empty() &&
            track.pointsEdgeRight[rowBreakRight].y < COLSIMAGE / 4) {
          track.pointsEdgeRight.resize(rowBreakRight); // 前80列不需要补线
        } else if (track.pointsEdgeRight.size() - rowBreakRight > 20) {
          float slopeTop = 0;    // 斜率：分歧点上半部分
          float slopeButtom = 0; // 斜率：分歧点下半部分
          if (track.pointsEdgeRight[rowBreakRight].x !=
              track.pointsEdgeRight[0].x) {
            slopeButtom = (track.pointsEdgeRight[rowBreakRight].y -
                           track.pointsEdgeRight[0].y) *
                          100 /
                          (track.pointsEdgeRight[rowBreakRight].x -
                           track.pointsEdgeRight[0].x);
          }
          if (track.pointsEdgeRight[rowBreakRight].x !=
              track.pointsEdgeRight[rowBreakRight + 20].x) {
            slopeTop = (track.pointsEdgeRight[rowBreakRight + 20].y -
                        track.pointsEdgeRight[rowBreakRight].y) *
                       100 /
                       (track.pointsEdgeRight[rowBreakRight + 20].x -
                        track.pointsEdgeRight[rowBreakRight].x);
          }

          if (slopeButtom * slopeTop <= 0) {
            rowBreakpointLeft = track.pointsEdgeRight[track.validRowsLeft].x;
            POINT p_end(rowBreakpointLeft, 0); // 补线终点为左边有效行顶点
            POINT p_mid(
                (track.pointsEdgeRight[rowBreakRight].x + rowBreakpointLeft) *
                    3 / 8,
                track.pointsEdgeRight[rowBreakRight].y / 2);
            vector<POINT> input = {track.pointsEdgeRight[rowBreakRight], p_mid,
                                   p_end};
            vector<POINT> b_modify = Bezier(0.01, input);
            track.pointsEdgeRight.resize(rowBreakRight);
            for (int kk = 0; kk < b_modify.size(); ++kk) {
              track.pointsEdgeRight.emplace_back(b_modify[kk]);
            }
          }
        } else if (track.pointsEdgeRight.size() - rowBreakRight <= 20) {
          _index = 2;
          POINT p_end(rowBreakpointLeft, 0);
          POINT p_start(max(rowBreakpointRight, ROWSIMAGE - 80), COLSIMAGE);
          POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) / 4, COLSIMAGE / 2);
          vector<POINT> input = {p_start, p_mid, p_end};
          vector<POINT> b_modify = Bezier(0.01, input);
          track.pointsEdgeRight.resize(0);
          for (int kk = 0; kk < b_modify.size(); ++kk) {
            track.pointsEdgeRight.emplace_back(b_modify[kk]);
          }
        }
      } else {
        ;
      }
      // uart->mpu6050_receiveCheck();
      // mpu6050_later = mpu6050;
      // cout << "mpu6050" << mpu6050_later - mpu6050 << endl;
      if (max(rowBreakpointLeft, rowBreakpointRight) <
          ROWSIMAGE * 3 / 4) { // 拐点在屏幕四份之三一下
        {
          ringStep = RingStep::Exiting;
          cout << "识别到拐点，出环" << endl;
        }
      }

      if (mpu6050_now - mpu6050_later > 300) // 判断mpu6050然后再出环
      {
        ringStep = RingStep::Exiting;
        cout << "mpu太大了" << endl;
      }
    }
    // 出环完成

    else if (ringStep == RingStep::Exiting) {
      if (ringType == RingType::RingLeft &&
          rowBreakpointLeft < ROWSIMAGE * 3 / 4) { // 只能左出环    原来都为2
        cout << "出环最终阶段" << endl << endl << endl << endl;
        POINT p_end(rowBreakpointLeft, 0);
        POINT p_start(ROWSIMAGE - 50, COLSIMAGE - 30); // 原先为减一
        POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) * 3 / 8,
                    COLSIMAGE / 3); // 原先为除以2
        vector<POINT> input = {p_start, p_mid, p_end};
        vector<POINT> b_modify = Bezier(0.01, input);
        track.pointsEdgeRight.resize(0);
        track.pointsEdgeLeft.resize(0);
        for (int kk = 0; kk < b_modify.size(); ++kk) {
          track.pointsEdgeRight.emplace_back(b_modify[kk]);
        }
        time++;
        cout << "累加次数判断" << time << endl;
        /* if (rowBreakpointRight > ROWSIMAGE*3/4 ) {  //||mpu6050_now -
         mpu6050_later > 360 ringStep = RingStep::Finish;
           cout<<"真的出环热"<<endl;
         }*/
      }
      if (time > 35) { //||mpu6050_now - mpu6050_later > 360 rowBreakpointRight
                       //> ROWSIMAGE*9/10 &&
        time = 0;
        ringStep = RingStep::Finish;
        cout << "真的出环热66666666" << endl;
      }
    }

    // //清掉边界的edge点
    // vector<POINT> v_temp, v_temp2;
    // for (int jj = 0; jj < track.pointsEdgeLeft.size(); ++jj)
    // {
    //     if (track.pointsEdgeLeft[jj].y > 2)
    //     {
    //         v_temp.push_back(track.pointsEdgeLeft[jj]);
    //     }
    //     else
    //     {
    //         if (jj > track.pointsEdgeLeft.size() * 9 / 10)
    //         {
    //             break;
    //         }
    //     }

    //     if (track.pointsEdgeLeft[jj].y > COLSIMAGE * 9 / 10 && jj <
    //     track.pointsEdgeLeft.size() - 5)
    //     {
    //         break;
    //     }
    // }
    // track.pointsEdgeLeft = v_temp;
    // if (track.pointsEdgeLeft.size() < 5)
    // {
    //     track.pointsEdgeLeft.resize(0);
    // }

    // for (int jj = 0; jj < track.pointsEdgeRight.size(); ++jj)
    // {
    //     if (track.pointsEdgeRight[jj].y < COLSIMAGE - 3)
    //     {
    //         v_temp2.push_back(track.pointsEdgeRight[jj]);
    //     }
    //     else
    //     {
    //         if (jj > track.pointsEdgeRight.size() * 9 / 10)
    //         {
    //             break;
    //         }
    //     }
    //     if (track.pointsEdgeRight[jj].y < COLSIMAGE / 10 && jj <
    //     track.pointsEdgeRight.size() - 5)
    //     {
    //         break;
    //     }
    // }
    // track.pointsEdgeRight = v_temp2;
    // if (track.pointsEdgeRight.size() < 5)
    // {
    //     track.pointsEdgeRight.resize(0);
    // }

    // 出环，切回正常循迹
    if (ringStep == RingStep::Finish) {
      if (track.pointsEdgeLeft.size() > 30 &&
          track.pointsEdgeRight.size() > 30 &&
          abs(track.pointsEdgeRight.size() - track.pointsEdgeLeft.size() <
              track.pointsEdgeRight.size() / 3) &&
          track.spurroad.empty()) {
        ringStep = RingStep::None;
        motion.reset_direction_pid(); // 切回巡线的pid   需要有圆环pid
      }
    }

    if (track.spurroad.empty())
      counterSpurroad++;
    else
      counterSpurroad = 0;

    //--------------------------------临时测试----------------------------------
    // _ringStep = ringStep;
    // _ringEnable = ringEnable;
    // _tmp_ttttt = tmp_ttttt;

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
             Scalar(0, 0, 255), -1); // 红色点
    }

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

    putText(ringImage, "[7] RING - ENABLE", Point(COLSIMAGE / 2 - 30, 10),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
    circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0),
           -1); // 红色点
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
  enum RingType {
    RingNone = 0, // 未知类型
    RingLeft,     // 左入环岛
    RingRight     // 右入环岛
  };

  /**
   * @brief 环岛运行步骤/阶段
   *
   */
  enum RingStep {
    None = 0, // 未知类型
    Entering, // 入环
    Inside,   // 环中
    Exiting,  // 出环
    Finish    // 环任务结束
  };

  RingType ringType = RingType::RingLeft; // 环岛类型
  RingStep ringStep = RingStep::None;     // 环岛处理阶段
  int rowRepairLine = 0;                  // 用于环补线的点（行号）
  int colRepairLine = 0;                  // 用于环补线的点（列号）
};
