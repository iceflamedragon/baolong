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
 * @file danger.cpp
 * @author Leo
 * @brief 危险区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-09
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测
#include "../motion.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/**
 * @brief 危险区AI识别与路径规划类
 *
 */
class Danger {

public:
  /**
   * @brief 危险区AI识别与路径规划处理
   *
   * @param track 赛道识别结果
   * @param predict AI检测结果
   * @return true
   * @return false
   */
 bool set_AI_detection(void){return is_ai_detection;}
  bool is_ai_detection=true;//是否开启AI标志
  int cone_num;
  int flagleft;
  int flagright;
  int time;
  int block_x;
  int flag_cone_first;
  int cone_temp;
  float distance_out;
  float distance_now;
  int danger_out;
  void save_common_pid(Motion &motion) {
    common_p1 = motion.params.runP1;
    common_p2 = motion.params.runP2;
    common_d = motion.params.turnD;
    common_i = motion.params.turnI;
  }

  void setdistance(float distance) { distance_now = distance; }; 
  bool process(Tracking &track, vector<PredictResult> predict, Motion &motion) {
    if(distance_now-distance_out>500&&danger_out)is_ai_detection=false;
    enable = false; // 场景检测使能标志
    if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 ||
        track.pointsEdgeRight.size() < ROWSIMAGE / 2)
      return enable;

    vector<PredictResult> resultsObs; // 锥桶AI检测数据
    for (int i = 0; i < predict.size(); i++) {
      if ((predict[i].type == LABEL_CONE || predict[i].type == LABEL_BLOCK) &&
          (predict[i].y + predict[i].height) >
              ROWSIMAGE * 0.2) // AI标志距离计算   更加灵活 原先为0.4
        resultsObs.push_back(predict[i]);
    }

    if (resultsObs.size() <= 0)
      return enable;

    // 选取距离最近的锥桶，面积最大的就最近
    int areaMax = 0; // 框面积
    int index = 0;   // 目标序号
    for (int i = 0; i < resultsObs.size(); i++) {
      int area = resultsObs[i].width * resultsObs[i].height;
      if (resultsObs[i].type == LABEL_CONE) {
        cone_num++;
        if (area >= areaMax) {
          index = i;
          areaMax = area;
        }
      }
    }
    cone_temp = cone_num;
    if (cone_num == 2)
      flag_cone_first = 1;
    if (cone_num == 1 && cone_temp == 2) {
      flagleft = 0;
      flagright = 0;
      flag_cone_first = 0;
    }
    cone_num = 0;
    resultObs = resultsObs[index];
    enable = true; // 场景检测使能标志

    // 障碍物方向判定（左/右）
    int row = track.pointsEdgeLeft.size() -
              (resultsObs[index].y + resultsObs[index].height -
               track.rowCutUp); ////这个计算有问题？切行？
    // cout << "障碍物所在row" << row << endl; //
    // row导致障碍物在不同侧---判断不出不同侧   以下面为0到上面的
    if (row < 0) // 无需规划路径
      return enable;

    int disLeft = resultsObs[index].x + resultsObs[index].width -
                  track.pointsEdgeLeft[row].y;
    int disRight = track.pointsEdgeRight[row].y - resultsObs[index].x;
    // if(resultsObs[index].type != LABEL_BLOCK)
    // {
    //     time++;
    //   if(time>10)
    //   {
    //   flagright=0;
    //   flagleft=0;
    //   }
    // }

    if (resultsObs[index].x + resultsObs[index].width >
            track.pointsEdgeLeft[row].y &&
        track.pointsEdgeRight[row].y > resultsObs[index].x &&
        disLeft <=
            disRight) //[1] 障碍物靠左&&resultsObs[index].type == LABEL_CONE
    {
      cout << "两种障碍物在左侧" << endl;
      // cout<<"障碍物的x坐标"<<resultsObs[index].x<<endl;
      // cout<<"障碍物的y坐标"<<resultsObs[index].y<<endl;
      save_common_pid(motion);
      motion.set_direction_pid(motion.params.danger_p1, motion.params.danger_p2,
                               motion.params.danger_d,0);

      if (resultsObs[index].type == LABEL_CONE) {
        if (cone_temp == 1) {
          flagleft = 1;
        }

        vector<POINT> points(4); // 三阶贝塞尔曲线

        cout << "第0个点的y坐标" << track.pointsEdgeLeft[row / 2].y + 20
             << endl;
        points[0] =
            track.pointsEdgeLeft
                [row /
                 2]; // points[0] = track.pointsEdgeLeft[row / 2]  row / 2
                     // {track.pointsEdgeLeft[150].x,track.pointsEdgeLeft[150].y+20}
        points[1] = {
            resultsObs[index].y + resultsObs[index].height, // 原来加了20
            resultsObs[index].x + resultsObs[index].width + 80}; // 原来为70
        points[2] = {(resultsObs[index].y + resultsObs[index].height +
                      resultsObs[index].y) /
                         2, // 原先为除以2   之后调为乘0.8
                     resultsObs[index].x + resultsObs[index].width +
                         80}; // 第二个位置仍然为y，
        if (resultsObs[index].y >
            track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1].x) {
          cout << "第三个点为第一种情况" << endl;
          points[3] = track.pointsEdgeLeft[track.pointsEdgeLeft.size() - 1];
        } else {
          cout << "第三个点为第二种情况" << endl;
          points[3] = {resultsObs[index].y,
                       resultsObs[index].x + resultsObs[index].width};
        }
        track.pointsEdgeLeft.resize((size_t)row / 2);  // 删除错误路线
        track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
        vector<POINT> repair = Bezier(0.01, points);   // 重新规划车道线
        vector<POINT> keys;
        keys.resize(repair.size());
        // vector<POINT> keys;
        // keys.size()=repair.size();
        for (int i = 0; i < repair.size(); i++) {
          track.pointsEdgeLeft.push_back(repair[i]);
          keys[i].x = repair[i].x; // 第一种思路，直接平移
          keys[i].y = repair[i].y + 120;
          track.pointsEdgeRight.push_back(keys[i]);
        }
        //  Danger_pointsEdgeRight(track.pointsEdgeLeft);
        //  track.pointsEdgeRight =
        //         predictEdgeRight(points); // 由左边缘补偿右边缘
      }

      for (int j = 0; j < resultsObs.size(); j++) {
        if (resultsObs[j].type ==
            LABEL_BLOCK) // 黑色路障特殊处理&&resultsObs[index].x>200&&resultsObs[index].y>75
        {

          block_x = resultsObs[j].x;
          cout << "黑色路障在左侧" << endl << endl;
          curtailTracking(track, true,
                          motion); // 缩减优化车道线（双车道→单车道）
          distance_out=distance_now;
          danger_out=1;
        }
      }
    } else if (resultsObs[index].x + resultsObs[index].width >
                   track.pointsEdgeLeft[row].y &&
               track.pointsEdgeRight[row].y > resultsObs[index].x &&
               disLeft > disRight) //[2] 障碍物靠右&&resultsObs[index].type ==
                                   // LABEL_CONE
    {
      cout << "两种障碍物在右侧" << endl;
      // cout<<"障碍物的x坐标"<<resultsObs[index].x<<endl;
      // cout<<"障碍物的y坐标"<<resultsObs[index].y<<endl;
      save_common_pid(motion);
      motion.set_direction_pid(motion.params.danger_p1, motion.params.danger_p2,
                               motion.params.danger_d,0);
      if (resultsObs[index].type == LABEL_CONE) {
        if (cone_temp == 1) {
          flagright = 1;
        }
        vector<POINT> points(4); // 三阶贝塞尔曲线
        points[0] = track.pointsEdgeRight[row / 2];
        points[1] = {resultsObs[index].y + resultsObs[index].height +
                         10, // 原来减少20
                     resultsObs[index].x - resultsObs[index].width - 50};
        points[2] = {(resultsObs[index].y + resultsObs[index].height +
                      resultsObs[index].y) /
                         2,
                     resultsObs[index].x - resultsObs[index].width - 50};
        if (resultsObs[index].y >
            track.pointsEdgeRight[track.pointsEdgeRight.size() - 1].x)
          points[3] = track.pointsEdgeRight[track.pointsEdgeRight.size() - 1];
        else
          points[3] = {resultsObs[index].y, resultsObs[index].x};

        track.pointsEdgeRight.resize((size_t)row / 2); // 删除错误路线
        track.pointsEdgeLeft.resize((size_t)row / 2);  // 删除错误路线
        vector<POINT> repair = Bezier(0.01, points);   // 重新规划车道线
        vector<POINT> keys;
        keys.resize(repair.size());
        for (int i = 0; i < repair.size(); i++) {
          track.pointsEdgeRight.push_back(repair[i]);
          keys[i].x = repair[i].x;       // 第一种思路，直接平移
          keys[i].y = repair[i].y - 135; // 原先为120  比较好过的为140
          track.pointsEdgeLeft.push_back(keys[i]);
        }
      }
      for (int j = 0; j < resultsObs.size(); j++) {
        if (resultsObs[j].type ==
            LABEL_BLOCK) // 黑色路障特殊处理&&resultsObs[index].x>200&&resultsObs[index].y>75
        {
         
          block_x = resultsObs[j].x;
          cout << "黑色路障在右侧" << endl << endl;
          curtailTracking(track, true,
                          motion); // 缩减优化车道线（双车道→单车道）
          distance_out=distance_now;
          danger_out=1;
        }
      }
    }

    return enable;
  } // 没写回正

  /**
   * @brief 图像绘制禁行区识别结果
   *
   * @param img 需要叠加显示的图像
   */
  void drawImage(Mat &img) {
    if (enable) {
      putText(img, "[2] DANGER - ENABLE", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
      cv::Rect rect(resultObs.x, resultObs.y, resultObs.width,
                    resultObs.height);
      cv::rectangle(img, rect, cv::Scalar(0, 0, 255), 1);
    }
  }

private:
  bool enable = false;     // 场景检测使能标志
  PredictResult resultObs; // 避障目标锥桶
  float common_p1;
  float common_p2;
  float common_d;
  float common_i;
  // /**
  //  * @brief 在俯视域由左边缘预测右边缘
  //  *
  //  * @param Danger_pointsEdgeRight
  //  * @return vector<POINT>
  //  */
  // vector<POINT> Danger_pointsEdgeRight(vector<POINT> &pointsEdgeLeft) {
  //   int offset = 120; // 右边缘平移尺度
  //   vector<POINT> pointsEdgeRight;
  //   if (pointsEdgeLeft.size() < 3)
  //     return pointsEdgeRight;

  //   // Start
  //   Point2d startIpm = ipm.homography(
  //       Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
  //   Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
  //   Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
  //   POINT startPoint = POINT(startIipm.y, startIipm.x);

  //   // Middle
  //   Point2d middleIpm = ipm.homography(
  //       Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
  //               pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
  //   prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
  //   Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
  //   POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

  //   // End
  //   Point2d endIpm = ipm.homography(
  //       Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
  //               pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
  //   prefictRight = Point2d(endIpm.x + offset, endIpm.y);
  //   Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
  //   POINT endPoint = POINT(endtIipm.y, endtIipm.x);

  //   // 补线
  //   vector<POINT> input = {startPoint, midPoint, endPoint};
  //   vector<POINT> repair = Bezier(0.05, input);

  //   for (int i = 0; i < repair.size(); i++) {
  //     if (repair[i].x >= ROWSIMAGE)
  //       repair[i].x = ROWSIMAGE - 1;

  //     else if (repair[i].x < 0)
  //       repair[i].x = 0;

  //     else if (repair[i].y >= COLSIMAGE)
  //       repair[i].y = COLSIMAGE - 1;
  //     else if (repair[i].y < 0)
  //       repair[i].y = 0;

  //     pointsEdgeRight.push_back(repair[i]);
  //   }

  //   return pointsEdgeRight;
  // }

  /**
   * @brief 缩减优化车道线（双车道→单车道）
   *
   * @param track
   * @param left
   */
  void curtailTracking(Tracking &track, bool left, Motion &motion) {
    if (!left) // 向左侧缩进  改变补线问题  // 此时相当于实际在左侧
    {
      motion.set_direction_pid(common_p1, common_p2, common_d,common_i);
      cout << "黑色路障相当于在左侧时右侧的补线" << endl;
      if (track.pointsEdgeRight.size() > track.pointsEdgeLeft.size())
        track.pointsEdgeRight.resize(track.pointsEdgeLeft.size());

      for (int i = block_x + 20; i < track.pointsEdgeRight.size(); i++) {
        track.pointsEdgeRight[i].y =
            (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2 -
            15; // 重新规划巡线  减少10还可以
      }
    } else // 向右侧缩进  障碍物在右侧
    {
      motion.set_direction_pid(common_p1, common_p2, common_d,common_i);
      cout << "黑色障碍物在右侧" << endl << endl;
      if (track.pointsEdgeRight.size() < track.pointsEdgeLeft.size())
        track.pointsEdgeLeft.resize(track.pointsEdgeRight.size());

      for (int i = block_x + 20; i < track.pointsEdgeLeft.size(); i++) {
        track.pointsEdgeLeft[i].y =
            (track.pointsEdgeRight[i].y + track.pointsEdgeLeft[i].y) / 2 -
            15; // 原来为除以2  减10
      }
    }
  }
};
