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
  float common_p1;
  float common_p2;
  float common_d;
  Crossroad c1;
  float distance_now;
  float distance_in;       // 状态一的编码器值
  float distance_diff;     // 差值
  float distance_laterout; // 出环的编码器值
  float distance_final;

public:
  int Left_Down_breakpoint = 0;  // 左下拐点行号
  int Right_Down_breakpoint = 0; // 左下拐点行号
  int monotonicity_change_line[2]; // 单调性改变点坐标，[0]寸某行，[1]寸某列
  int monotonicity_right = 0; // 右侧单调
  int monotonicity_left = 0;  // 左侧单调
  int lostline_left = 0;      // 左右丢线数量
  int lostline_right = 0;
  int monotonicity_change_left_flag = 0;  // 不转折是0
  int monotonicity_change_right_flag = 0; // 不转折是0
  int continuity_change_right_flag = 0;   // 连续是0
  int continuity_change_left_flag = 0;    // 连续是0
  void setmpu6050(float mpu6050_now_read) { mpu6050_now = mpu6050_now_read; }
  void setdistance(float distance) { distance_now = distance; };
  uint16_t counterShield = 0; // 环岛检测屏蔽计数器：屏蔽车库误检测

#define RingNone 0
#define RingLeft 1
#define RingRight 2
  /**
   * @brief 环岛识别初始化|复位
   *
   */
  static int ru;
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
    cout << "原先的pid为" << motion.params.ring_p1 << endl
         << motion.params.ring_p2 << endl
         << motion.params.ring_d << endl;
    motion.params.ring_p1 = ring_p1;

    motion.params.ring_p2 = ring_p2;

    motion.params.ring_d = ring_d;
    cout << "设置新的pid" << endl;
  }
  void save_common_pid(Motion &motion) {
    common_p1 = motion.params.ring_p1;
    common_p2 = motion.params.ring_p2;
    common_d = motion.params.ring_d;
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
  int first_ring = 0;
  int left = 0;
  int right = 0;
  int ringTypeTemp = 0;
  bool process(Tracking &track, Mat &imagePath, Motion &motion) {
    // 暂时注释
    //  if (counterShield < 40) {
    //    counterShield++;
    //    return false;
    //  }

    /////借鉴的代码
    if (!first_ring) {
      // cout<<"666666666666"<<endl;
      ringTypeTemp = RingNone;
      first_ring = 1;
    }
    static int time = 0;
    // cout<<"标志位。。。"<<first_ring<<endl;
    //  以下是常规判断法

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
      // cout<<"kaishipanduan9999999999"<<endl;
      Left_Down_breakpoint = Find_Left_Down_Point(track, 80, 120);
      Right_Down_breakpoint = Find_Right_Down_Point(track, 80, 120); // 原来150
      if (Left_Down_breakpoint) // 有左下拐点
      {
        // time++;
        // 加了一点延时
        // cout<<"圆环判断出左下拐点"<<endl;
        lostline_left = Lostline_Left(track, Left_Down_breakpoint + 10,
                                      Left_Down_breakpoint);
        continuity_change_right_flag = Continuity_Change_Right(
            track, Left_Down_breakpoint - 10,
            Left_Down_breakpoint + 10); // 拐点附近判断连续性
        continuity_change_left_flag =
            Continuity_Change_Left(track, ROWSIMAGE - 1 - 5, 50);
        if (ringTypeTemp == RingNone) {
          if (lostline_left > 8 &&                 // 判断左入环
              continuity_change_left_flag != 0 &&  // 左边是不连续的
              continuity_change_right_flag == 0 && // 左环岛右边是连续的
              track.validRowsLeft >= 10 &&         // 左边有效行不能少
              track.validRowsLeft <= 100 &&        // 左边有效行比较少
              track.validRowsRight >= 140          // 右边丢线较少
          ) {
            monotonicity_right =
                Monotonicity_Right(track, Left_Down_breakpoint - 30,
                                   Left_Down_breakpoint + 30); // 原来此处为10
            if (monotonicity_right == 0) { // 右边是单调的
              ringStep = RingStep::Entering;
              ringTypeTemp = RingLeft; // 环岛类型：左入环
              left = 1;
              ringEnable = true;
              std::cout << "判断出左入环" << endl;
              distance_in = distance_now;
              mpu6050_later = mpu6050_now;
            } else {
              ringStep == RingStep::None;
              cout << "误判环" << endl;
            }
          }
        }

        ////判断右入环
      } else if (Right_Down_breakpoint) // 有右下拐点
      {
        // cout<<"圆环判断出右下拐点"<<endl;
        continuity_change_left_flag = Continuity_Change_Left(
            track, Right_Down_breakpoint - 10,
            Right_Down_breakpoint + 10); // 拐点附近判断连续性
        continuity_change_left_flag =
            Continuity_Change_Left(track, ROWSIMAGE - 1 - 5, 50);
        if (ringTypeTemp == RingNone) {
          if ( // 左边是单调的          //判断左入环
              continuity_change_left_flag != 0 &&  // 右边边是不连续的
              continuity_change_right_flag == 0 && // 右环岛左边是连续的
              track.validRowsRight >= 10 &&  // 右边有效行不能太少
              track.validRowsRight <= 100 && // 右边有效行比较少
              track.validRowsLeft >= 140)    // 左边丢线较少
          {
            // monotonicity_change_right_flag ==
            //       0 &&
            // 找左下角点
            // if (left_down_guai[0] >=
            //    30) // 条件1很松，在这里判断拐点，位置不对，则是误判，跳出
            //{
            ringStep = RingStep::Entering;
            ringTypeTemp = RingRight; // 环岛类型：右入环
            left = 1;
            ringEnable = true;
            mpu6050_later = mpu6050_now;
            std::cout << "判断出右入环" << endl;
            //} else // 误判，归零
            //{
            //  ringStep == RingStep::None;
            //  cout << "误判环" << endl;
          }
        }

        ////判断右入环
      }
    }
    int countWide = 0; // 环岛入口变宽区域行数
    for (int i = 1; i < track.widthBlock.size(); ++i) {
      if (track.widthBlock[i].y > track.widthBlock[i - 1].y &&
          track.widthBlock[i].y > COLSIMAGE * 0.6 &&
          track.widthBlock[i].x > 30 &&
          ((track.stdevLeft > 120 && track.stdevRight < 50) &&
           ringStep == RingStep::IsRing)) // 搜索突然变宽的路径行数   Entering
      {
        ++countWide;
        cout << "countwide" << countWide << endl;
      } else {
        countWide = 0;
      }
    }

    if (ringStep == RingStep::IsRing) // 状态2入环前判断
    {
      // cout<<"状态222222"<<endl;
      // cout<<"左入环"<<ringTypeTemp<<endl;
      // cout<<"左入环"<<left<<endl;
      continuity_change_left_flag = Continuity_Change_Left(
          track, Right_Down_breakpoint - 10,
          Right_Down_breakpoint + 10); // 拐点附近判断连续性
      continuity_change_left_flag =
          Continuity_Change_Left(track, ROWSIMAGE - 1 - 5, 50);
      if (ringTypeTemp ==
          RingLeft) // 左入环  对于类型问题换个方式？ ringTypeTemp == RingLeft
      {
        monotonicity_change_line[0] =
            Monotonicity_Change_Left(track, 100, 160); // 寻找单调性改变点，i
        cout << "monotonicity_change_line//////////////////////////////"
             << monotonicity_change_line[0] << endl;
        monotonicity_change_line[1] =
            track.pointsEdgeLeft[monotonicity_change_line[0]].y; // y
        if (monotonicity_change_line[0] >
            140) { // 当左边不单调点较低，或者左侧的斜率较大
          ringStep = RingStep::Entering;
          // mpu6050_later = mpu6050_now;
          cout << "入环前////////////////////////////////" << endl;
        }
      } else if (ringTypeTemp == RingRight) // 右入环  //右入环
      {
        cout << "右右右右" << endl;
        monotonicity_change_line[0] =
            Monotonicity_Change_Right(track, 10, 80); // 寻找单调性改变点，x
        monotonicity_change_line[1] =
            track.pointsEdgeLeft[monotonicity_change_line[0]].y; // y
        if (monotonicity_change_line[0] < 70) {
          ringStep = RingStep::Entering;
          // mpu6050_later = mpu6050_now;
          cout << "入环前//////////////////////////////////" << endl;
        } // 当右边不单调点较低，或者斜率较大
      }
    }

    // 状态三 入环判断
    if ((ringStep == RingStep::Entering)) /// 有
    {
      save_common_pid(motion);
      set_ring_pid(ring_p1, ring_p2, ring_d, motion);
      // Left_Down_breakpoint =
      Find_Left_Down_Point(track, 80, 160);
      // time++;
      // cout<<time<<endl<<endl;
      //  if(time>1000)//&&!track.spurroad.empty()&&Left_Down_breakpoint<50
      //   {
      Left_Down_breakpoint = Find_Left_Down_Point(track, 20, 80);
      continuity_change_right_flag = Continuity_Change_Right(
          track, Left_Down_breakpoint - 10,
          Left_Down_breakpoint + 10); // 拐点附近判断连续性

      distance_diff = distance_now - distance_in;
      cout << "入环路程差值" << distance_diff << endl; // distance的路程积分
      if (!track.spurroad.empty() &&
          (distance_diff >
           1600)) // Left_Down_breakpoint < 50   !continuity_change_right_flag
      {
        cout << "进入状态三,入环判断拐点" << endl;
        for (int i = 1; i < track.widthBlock.size(); ++i) {
          cout << "进入状态三,入环判断" << endl;
          if (track.pointsEdgeLeft[i].y < track.pointsEdgeLeft[i - 5].y) {
            ringTypeTemp = RingLeft; // 环岛类型：左入环
            colEnterRing = track.pointsEdgeLeft[i - 5].y; // 入环点列号
            _ringPoint.x = track.pointsEdgeLeft[i - 5].x;
            _ringPoint.y = track.pointsEdgeLeft[i - 5].y;

            rowRepairLine = i; // 用于环补线的行号
            colRepairLine = track.pointsEdgeLeft[i].x; // 用于环补线的列号

          } else if (track.pointsEdgeRight[i].y >
                     track.pointsEdgeRight[i - 5].y) {
            ringTypeTemp = RingRight; // 环岛类型：右入环
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
            cout << "内圆检测" << endl;
            mpu6050_later = mpu6050_now;

            ringEnable = true;
            ringStep = RingStep::Entering;
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

          /*if(ringStep == RingStep::Entering && ringEnable == false){
              ringEnable = true;
              rowRepairStraightside = rowRepairLine;
          }*/

          if (ringEnable == true && ringStep == RingStep::Entering) {
            if (ringTypeTemp == RingLeft) {
              if (track.pointsEdgeLeft[i].y <= 2 &&
                  i != track.widthBlock.size() - 1) {
                if (rowRepairRingside == track.widthBlock.size() - 1) {
                  rowRepairRingside = i;
                }
                rowYendStraightside = track.pointsEdgeLeft[i].x;
              } else if (rowRepairRingside != track.widthBlock.size() - 1 &&
                         distance_diff > 1600)

              {

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
                std::cout << "开始补线" << endl;
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
      }
    }
    /*********************状态三判断结束 */
    int tmp_ttttt = 0;
    if (ringEnable == false && ringStep == RingStep::Entering) {
      // 本场没判出环，且没有分叉
      if (!track.spurroad.empty() &&
          rowRepairLine < track.pointsEdgeRight.size() - 1 &&
          rowBreakpointRight > ROWSIMAGE / 2) {

        rowRepairStraightside = rowRepairLine;
        cout << "没判断出环布线" << endl;
        if (ringType == RingLeft) {
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
      //

      /*else {
        if (ringType == RingLeft &&
            track.pointsEdgeRight.size() > 1) {
          tmp_ttttt = 2;
          cout<<"没判断出环，有分叉"<<endl;
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
        }///原来没注释掉------对于两种状态分析
      }*/
    }
    // 状态4  环中，正常巡线
    if (ringStep == RingStep::Entering && mpu6050_now - mpu6050_later >= 60) {
      ringStep = RingStep::Inside; // 纯粹陀螺仪积分到一定值就正常巡线
      cout << "已经到环中了" << endl;
    }
    // 出环补线
    if (ringStep == RingStep::Inside) {
      if (ringType == RingLeft) {
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
          cout << "前80列不用补线" << endl;
          track.pointsEdgeRight.resize(rowBreakRight); // 前80列不需要补线
        } else if (track.pointsEdgeRight.size() - rowBreakRight >
                   0) { // 下方两个同时输出，说明是此处制约时间  原来为20
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
          cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
               << endl;
          if (slopeButtom * slopeTop <= 0) {
            cout << "下半部分  " << slopeButtom << "   上半部分" << slopeTop
                 << endl
                 << endl;
            rowBreakpointLeft = track.pointsEdgeRight[track.validRowsLeft].x;
            cout << "出环补线1" << endl << endl << endl;
            set_ring_pid(common_p1, common_p2, common_d, motion);
            cout << "切换为普通pid" << endl;
            POINT p_end(rowBreakpointLeft - 40,
                        0); // 补线终点为左边有效行顶点 //原先为不减，现在减少
            POINT p_mid(
                (track.pointsEdgeRight[rowBreakRight].x + rowBreakpointLeft) *
                    3 / 8,
                track.pointsEdgeRight[rowBreakRight].y /
                    4); // 将其减小--拉的更偏内些？  原来除以2
            vector<POINT> input = {track.pointsEdgeRight[rowBreakRight], p_mid,
                                   p_end};
            vector<POINT> b_modify = Bezier(0.01, input);
            track.pointsEdgeRight.resize(rowBreakRight);
            for (int kk = 0; kk < b_modify.size(); ++kk) {
              track.pointsEdgeRight.emplace_back(b_modify[kk]);
            }
          }
        } else if (track.pointsEdgeRight.size() - rowBreakRight <= 20 &&
                   Find_Left_Down_Point(track, 100, 180)) {
          cout << "出环补线2222" << endl << endl << endl;
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

      if (mpu6050_now - mpu6050_later > 280) // 判断mpu6050然后再出环
      {
        // if (max(rowBreakpointLeft, rowBreakpointRight) < ROWSIMAGE / 2) {
        ringStep = RingStep::Exiting;
        // distance_laterout = distance_now; // 开始路程积分
        cout << "开始路程积分" << endl;
        distance_final = distance_now;
        // }
      }
    }
    // 出环完成
    // 出环完成
    else if (ringStep == RingStep::Exiting) {
      cout << "到达exiting了" << endl << endl << endl << endl;
      if (ringType == RingLeft && rowBreakpointLeft < ROWSIMAGE / 2) {
        POINT p_end(rowBreakpointLeft, 0);
        POINT p_start(ROWSIMAGE - 50, COLSIMAGE - 1);
        POINT p_mid((ROWSIMAGE - 50 + rowBreakpointLeft) * 3 / 8,
                    COLSIMAGE / 2);
        vector<POINT> input = {p_start, p_mid, p_end};
        vector<POINT> b_modify = Bezier(0.01, input);
        track.pointsEdgeRight.resize(0);
        track.pointsEdgeLeft.resize(0);

        for (int kk = 0; kk < b_modify.size(); ++kk) {
          track.pointsEdgeRight.emplace_back(b_modify[kk]);
        }
        if (rowBreakpointRight > ROWSIMAGE / 2) {
          cout << "到达finish" << endl;
          ringStep = RingStep::Finish;
        }
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

    // 出环，切回正常循迹        //这里应该依靠路程积分彻底出环
    if (ringStep == RingStep::Finish) {
      cout << "距离差值   " << distance_now - distance_final << endl;
      if (track.pointsEdgeLeft.size() > 30 &&
          track.pointsEdgeRight.size() > 30 &&
          abs(track.pointsEdgeRight.size() - track.pointsEdgeLeft.size() <
              track.pointsEdgeRight.size() / 3) &&
          track.spurroad.empty() &&
          (distance_now - distance_final >= 1200)) { // 结束出环补线  之前为1500
        ringStep = RingStep::None;
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
             Scalar(0, 0, 255), -1); // 红色拐点
    }
    // circle(ringImage,
    //        Point(track.pointsEdgeLeft[Left_Down_breakpoint].y,
    //              track.pointsEdgeLeft[Left_Down_breakpoint].x),
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

    putText(ringImage, "[7] RING - ENABLE", Point(COLSIMAGE / 2 - 30, 10),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
    circle(ringImage, Point(_ringPoint.y, _ringPoint.x), 4, Scalar(255, 0, 0),
           -1); // 红色点
  }

  /*----------------------------------------自己写的函数---------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     右下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0，点所在的扫线的行0到180，
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Find_Right_Down_Point(Tracking &track, int start,
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
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 2].y) <=
              10 &&
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 3].y) <=
              10 &&
          abs(track.pointsEdgeRight[i].y - track.pointsEdgeRight[i - 4].y) <=
              10) {
        right_down_line = i; // 获取行数即可
        break;
      }
    }
    return right_down_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     左下角点检测
  @param     起始点，终止点
  @return    返回角点所在的行数，找不到返回0，点所在的扫线的行0到180，
  Sample     Find_Right_Down_Point(int start,int end);
  @note      角点检测阈值可根据实际值更改
  -------------------------------------------------------------------------------------------------------------------*/
  int Find_Left_Down_Point(Tracking &track, int start,
                           int end) // 找四个角点，返回值是角点所在的行数
  {
    int i, t;
    int right_down_line = 0;
    // cout<<"kaishi   "<<start<<"   jieshu"<<end<<endl;
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
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i + 1].y) <=
              3 && // 角点的阈值可以更改
          abs(track.pointsEdgeLeft[i + 1].y - track.pointsEdgeLeft[i + 2].y) <=
              3 &&
          abs(track.pointsEdgeLeft[i + 2].y - track.pointsEdgeLeft[i + 3].y) <=
              3 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 2].y) >=
              10 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 3].y) >=
              10 &&
          abs(track.pointsEdgeLeft[i].y - track.pointsEdgeLeft[i - 4].y) >=
              10) {
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
  @return    连续返回0，不连续返回断线出行数点所在的扫线的行0到180，
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
  @return    连续返回0，不连续返回断线出行数点所在的扫线的行0到180，
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
  @return    点所在的扫线的行0到180，，找不到返回0
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
      if (track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 5].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 5].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 4].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 4].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 3].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 3].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 2].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i - 2].y &&
          track.pointsEdgeRight[i].y == track.pointsEdgeRight[i + 1].y &&
          track.pointsEdgeRight[i].y ==
              track.pointsEdgeRight[i - 1]
                  .y) { // 一堆数据一样，显然不能作为单调转折点
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
        break;
      }
    }
    return monotonicity_change_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
  @brief     单调性突变检测左
  @param     起始点，终止行
  @return    点所在的扫线的行0到180，找不到返回0
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
      cout << "进入做单调判断" << endl;
      if (track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 25].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 25].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 20].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 20].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 15].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 10].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 10].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 5].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i + 5].y &&
          track.pointsEdgeLeft[i].y == track.pointsEdgeLeft[i - 15].y) {
        cout << "左边缘点数值都一样了"
             << endl; // 一堆数据一样，显然不能作为单调转折点
        continue;
      } else if (
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 25].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 25].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i + 20].y &&
          track.pointsEdgeLeft[i].y > track.pointsEdgeLeft[i - 20].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 15].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 15].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 10].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i - 10].y &&
          track.pointsEdgeLeft[i].y >= track.pointsEdgeLeft[i + 5].y &&
          track.pointsEdgeLeft[i].y >=
              track.pointsEdgeLeft[i - 5]
                  .y) { // 就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
                        // //  点少了？
        monotonicity_change_line = i;
        cout << "左边缘点坐标  " << i << endl;
        cout << "找到中继点辣" << endl;
        break;
      }
      // }
    }
    return monotonicity_change_line;
  }
  /*-------------------------------------------------------------------------------------------------------------------
    @brief     单调右
    @param     起始点，终止行
    @return    点所在的扫线的行0到180，，单调就为0
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
  @brief     单调左
  @param     起始点，终止行
  @return    点所在的扫线的行0到180，，单调就为0
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
 @return    点所在的扫线的行0到180，，单调就为0
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
