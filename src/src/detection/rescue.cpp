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
 * @file rescue.cpp
 * @author Leo
 * @brief 救援区检测
 * @version 0.1
 * @date 2024-01-11
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

#include "../../include/common.hpp"
#include "../../include/detection.hpp"
#include "../mapping.cpp"
#include "../recognition/tracking.cpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Rescue {
public:
int car_changepid=0;
  bool carStoping = false;  // 停车标志
  bool carExitting = false; // 出库标志
  int flagin;//
  int reflag;//这两个flag用来保证在识别到锥桶相对位置后使得distiance_in为距离
  int  flagdis;//为 0时distiance_in=distance_now
  int flagtrace;
  int flagchur;
   int flagchul;
  int a;
  void setdistancere(float distance) { distance_now = distance; };
  enum Step {
    None = 0, // AI检测
    Enable,   // 使能（标志识别成功）
    Enter,    // 进站
    Cruise,   // 巡航
    Stop,     // 停车
    Exit      // 出站
  };

  Step step = Step::None;
  bool entryLeft = true; // 左入库使能标志
  /**
   * @brief 检测初始化
   *
   */
  void reset(void) {
    carStoping = false;
    carExitting = false;
    step = Step::None;//None
    counterSession = 0;         // 图像场次计数器
    counterRec = 0;             // 标志检测计数器
    lastPointsEdgeLeft.clear(); // 记录上一场边缘点集（丢失边）
    lastPointsEdgeRight.clear();
    counterExit = 0;
    counterImmunity = 0;
  }

  /**
   * @brief 检测与路径规划
   *
   * @param track 赛道识别结果
   * @param detection AI检测结果
   */
  bool process(Tracking &track, vector<PredictResult> predict,Motion &motion) {
    _pointNearConeL = POINT(0, 0);
    _pointNearConeR = POINT(0, 0);
    _pointNearCone = POINT(0,0);
    _distance = 0;
    pointConeLeft.clear();
    pointConeRight.clear();
    levelCones = 0;
    indexDebug = 0;
     
     int retime=0;
     int counterme=0 ;      // 真正用到的标志检测计数器
     int stoptime=0;
      int chu=0;
      
    // cout<<"延时点数"<<retime<<endl;
    if(flagdis==0)distance_in=distance_now;
    if(reflag==1)
    {
        retime++;

    }
    //step = Step::Exit;
    switch (step) {
    case Step::None: //[01] 标志检测
      // cout<<"识别标志牌的计数"<<counterImmunity<<endl;
      if (                                      //(counterImmunity > 200 && again) || 
        (counterImmunity > 10 && !again)) {   //此处上方注释为第二次进入救援区，，，第一次进入为计数器---识别到标志牌
        for (int i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_TUMBLE ||
              predict[i].type == LABEL_PATIENT) // 伤员平民标志检测
          {
            cout<<"识别到伤员"<<endl;
            counterRec++;
            break;
          }
        }
        for (int i = 0; i < predict.size(); i++) {
          if (predict[i].type == LABEL_EVIL ||
              predict[i].type == LABEL_THIEF) // 小偷强盗标志检测
          {
            cout<<"识别到恐怖分子了"<<endl<<endl<<endl;
            counterExit++;
            break;
          }
        }

        if (counterRec || counterExit) {
          counterSession++;
          if (counterRec > 0 && counterSession <= 8) {  // 原来为3
            cout<<"判断为左进了666"<<endl;
            step = Step::Enable; // 使能
            entryLeft = true;
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
          } else if (counterExit > 0 && counterSession <= 8) {  //对右入库分析
            step = Step::Enable; // 使能
            cout<<"判断为右进了666"<<endl<<endl<<endl;
            entryLeft = false;//非左即右
            counterRec = 0;
            counterExit = 0;
            counterSession = 0;
          } else if (counterSession > 8) {
            counterRec = 0;
            counterSession = 0;
          }
         // step = Step::Enable;
        }
      } else
        counterImmunity++;
      break;

    case Step::Enable: //[02] 使能
    {
      cout<<"到达enable了"<<endl<<endl;
      counterExit++;
      if (counterExit > 150) // 超时退出
      {
        reset();
        return false;
      }

      searchCones(predict,motion); // 搜索赛道左右两边锥桶
      if (entryLeft)        // 左入库
      {
        _pointNearConeL = getConeLeftDown(track.pointsEdgeLeft,
                                         pointConeLeft); // 搜索左下锥桶
        _pointNearConeR = getConeRightDown(track.pointsEdgeRight,
                                         pointConeRight); // 搜索左下锥桶                                 
        if (((_pointNearConeL.x >
            ROWSIMAGE * 0.6)||(_pointNearConeR.x >ROWSIMAGE * 0.6)) &&!flagin)
            {// 当车辆开始靠近右边锥桶：准备入库
            reflag=1;
            
            cout<<"锥桶到达0.6的位置了"<<endl<<endl<<endl;
            }
            if(reflag)
         {
              distance_in=distance_now;
              cout<<"此时的距离"<<distance_in<<endl<<endl<<endl;
              flagin=1;
              reflag=0;
              flagdis=1;
         }    
        //  cout<<"距离差值"<<distance_now-distance_in<<endl;
            if(!reflag&&(distance_now-distance_in)>280)//原来为200
        {
          
           
          //counterRec++;
          // counterRec++;
          // counterme++;
          cout<<"看到右侧锥桶了"<<counterme<<endl<<endl<<endl;
          //}
          //if (counterme >= 2&&retime>15) {  //原先为2
          
            step = Step::Enter; // 进站使能
            reflag=0;
            cout<<"左进站辣辣辣"<<endl<<endl<<endl;
            counterRec = 0;
            counterme=0;
            counterSession = 0;
            counterExit = 0;
            pathsEdgeLeft.clear();
            pathsEdgeRight.clear();
          
        }
            
      } else // 右入库
      {
       // //cout<<"打算右入库22222220"<<endl<<endl<<endl;
        _pointNearConeR = getConeRightDown(track.pointsEdgeRight,
                                          pointConeRight); // 搜索右下锥桶
        _pointNearConeL = getConeLeftDown(track.pointsEdgeLeft,
                                          pointConeLeft); // 搜索左下锥桶          
        //cout<<"此时的距离为"<<distance_now<<endl;                              
        if (((_pointNearConeL.x >
            ROWSIMAGE * 0.6)||(_pointNearConeL.x >ROWSIMAGE * 0.6)) &&!flagin)
            {// 当车辆开始靠近右边锥桶：准备入库
            reflag=1;
            flagtrace=1;
            cout<<"锥桶到达0.6的位置了"<<endl<<endl<<endl;
            }
        if(flagtrace)
        {
          //  pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
          // pathsEdgeRight.push_back(track.pointsEdgeRight);
        }
            if(reflag)
         {
              distance_in=distance_now;
              cout<<"此时的距离"<<distance_in<<endl<<endl<<endl;
              flagin=1;
              reflag=0;
              flagdis=1;
         }    
        //  cout<<"距离差值"<<distance_now-distance_in<<endl;
            if(!reflag&&(distance_now-distance_in)>280)//原来为470
        {
          
           
          //counterRec++;
          // counterRec++;
          // counterme++;
          cout<<"看到右侧锥桶了"<<counterme<<endl<<endl<<endl;
          //}
          //if (counterme >= 2&&retime>15) {  //原先为2
          
            step = Step::Enter; // 进站使能
            reflag=0;
            cout<<"右进站辣辣辣"<<endl<<endl<<endl;
            counterRec = 0;
            counterme=0;
            counterSession = 0;
            counterExit = 0;
            pathsEdgeLeft.clear();
            pathsEdgeRight.clear();
          
        }
            
      }
      break;
    }
    case Step::Enter: //[03] 入库使能
    {
            searchCones(predict,motion);
          // if (counterExit > 11) {  //此处设置为了总的延时   用距离判断吗？距离写个  右侧为9-10
          //  //stoptime++;
          // // cout<<"stoptime"<<stoptime<<endl<<endl;
          //  //if(stoptime>8)      
          // // {                      ///原先为30
          //   counterExit = 0;
          //   cout<<"停车了老司机5555"<<endl<<endl;
          //   step = Step::Stop; // 停车使能
          //   counterRec = 0;
          //   counterSession = 0;
          // // }
          // }
        if (track.pointsEdgeLeft.size() < ROWSIMAGE / 2 &&
            track.pointsEdgeRight.size() < ROWSIMAGE / 2) // 赛道还未丢失
        {
          counterRec++;
          if (counterRec > 15) {
            counterRec = 0;
            cout<<"维持原先状态进行巡航666"<<endl<<endl<<endl;
            step = Step::Cruise; // 巡航使能
            counterSession = 0;
          }
        }

      int smallestcone = 0;
        if (entryLeft) // 左入库
        {
          for (int i = 0; i < pointConeLeft.size(); i++) {
          // int area = pointConeLeft[i].width * pointConeLeft[i].height;
           // if (area<areaMax){
            if (pointConeLeft[i].y > 110 &&
              pointConeLeft[i].y < 210) // 小于车身大小  原来减少了20
            if (pointConeLeft[i].x > smallestcone)
              smallestcone = pointConeLeft[i].x;
           //}
          if (smallestcone > motion.params.stop_num) // 距离车辆多少开始停车
          {
          cout << "停车了老司机5555" << endl << endl;
          step = Step::Stop; // 停车使能
          }}
          cout<<"开始左入库了"<<endl;
          POINT start = POINT(ROWSIMAGE - 40, COLSIMAGE - 1);
          POINT end = POINT(50, 40);//原先为0
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.7);//原先乘0.6
          vector<POINT> input = {start, middle, end};
          track.pointsEdgeRight = Bezier(0.05, input); // 补线
          track.pointsEdgeLeft =
              predictEdgeLeft(track.pointsEdgeRight); // 由右边缘补偿左边缘
          lastPointsEdgeLeft = track.pointsEdgeLeft;
          lastPointsEdgeRight = track.pointsEdgeRight;

          pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
          pathsEdgeRight.push_back(track.pointsEdgeRight);
        } else // 右入库
        { 
          for (int i = 0; i < pointConeRight.size(); i++) {

          if (pointConeRight[i].y > 110 &&
              pointConeRight[i].y < 210) // 小于车身大小  原先为210
            if (pointConeRight[i].x > smallestcone)
              smallestcone = pointConeRight[i].x;
          }
          if (smallestcone > motion.params.stop_num) // 距离车辆多少开始停车
          {
            cout << "停车了老司机5555" << endl << endl;
          step = Step::Stop; // 停车使能
          }
          cout<<"开始右入库了"<<endl;
          POINT start = POINT(ROWSIMAGE - 40, 0);
          POINT end = POINT(50, COLSIMAGE - 40);//原来为减1  后来减40
          POINT middle =
              POINT((start.x + end.x) * 0.4, (start.y + end.y) * 0.4);//原来为乘0.6
          vector<POINT> input = {start, middle, end};
          track.pointsEdgeLeft = Bezier(0.05, input); // 补线
          track.pointsEdgeRight =
              predictEdgeRight(track.pointsEdgeLeft); // 由右边缘补偿左边缘
          lastPointsEdgeLeft = track.pointsEdgeLeft;
          lastPointsEdgeRight = track.pointsEdgeRight;

          pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
          pathsEdgeRight.push_back(track.pointsEdgeRight);
        }
      
      break;
    }

    case Step::Cruise: //[04] 巡航使能
    {
      cout<<"到巡航了"<<endl;
      counterSession++;
      if (counterSession > 10) // 超时保护
      {
        step = Step::Stop; // 停车使能
        counterRec = 0;
        counterSession = 0;
        counterExit = 0;
      }

      if (track.pointsEdgeLeft.size() > ROWSIMAGE / 2 &&
          track.pointsEdgeRight.size() > ROWSIMAGE / 2) {
        counterExit++;
        if (counterExit > 15) {
          counterExit = 0;
          step = Step::Stop; // 停车使能
          counterRec = 0;
        }
      }

      searchCones(predict,motion);             // 计算锥桶的坐标
      POINT heighestCone = POINT(0, 0); // 搜索顶点锥桶
      for (int i = 0; i < pointConeLeft.size(); i++) {
        heighestCone = pointConeLeft[0];
        if (pointConeLeft[i].x < heighestCone.x)
          heighestCone = pointConeLeft[i];
      }
      for (int i = 0; i < pointConeRight.size(); i++) {
        if (pointConeRight[i].x < heighestCone.x)
          heighestCone = pointConeRight[i];
      }

      if (heighestCone.x > 0 && heighestCone.y > 0) {
        if (heighestCone.y >= COLSIMAGE / 3) // 顶角锥桶靠近右边→继续右转
        {
          vector<POINT> points;
          for (int i = 0; i < pointConeLeft.size();
               i++) // 搜索以最高点为分界线左边的锥桶
          {
            if (pointConeLeft[i].y <= heighestCone.y)
              points.push_back(pointConeLeft[i]);
          }

          if (points.size() >= 3) // 曲线补偿
          {
            pointsSortForY(points); // 排序
            vector<POINT> input = {points[0], points[points.size() / 2],
                                   points[points.size() - 1]};
            track.pointsEdgeLeft = Bezier(0.05, input); // 补线
            track.pointsEdgeRight =
                predictEdgeRight(points); // 由左边缘补偿右边缘

            indexDebug = 1;
          } else if (points.size() >= 2) {
            pointsSortForY(points); // 排序
            POINT middle =
                POINT((points[0].x + points[points.size() - 1].x) / 2,
                      (points[0].y + points[points.size() - 1].y) / 2);
            vector<POINT> input = {points[0], middle,
                                   points[points.size() - 1]};
            track.pointsEdgeLeft = Bezier(0.05, input); // 补线
            track.pointsEdgeRight =
                predictEdgeRight(points); // 由左边缘补偿右边缘
            indexDebug = 2;
          } else if (points.size() >= 1) {
            if (points[0].x > ROWSIMAGE / 2)
              step = Step::Exit; // 出站使能
          }
        } else // 顶角锥桶靠近左边→继续右转
        {
          indexDebug = 3;
          track.pointsEdgeLeft = lastPointsEdgeLeft;
          track.pointsEdgeRight = lastPointsEdgeRight;
        }

        // 计算所有锥桶的平均高度
        int num = 0;
        for (int i = 0; i < pointConeLeft.size(); i++)
          num += pointConeLeft[i].x;
        for (int i = 0; i < pointConeRight.size(); i++)
          num += pointConeRight[i].x;

        if (pointConeLeft.size() + pointConeRight.size() > 0)
          levelCones = num / (pointConeLeft.size() + pointConeRight.size());
        else
          levelCones = 0;
        if (levelCones > ROWSIMAGE * 0.5 || levelCones == 0) {
          counterRec++;
          if (counterRec > 2) {
            step = Step::Stop; // 停车使能
            counterRec = 0;
          }
        }
      } else {
        indexDebug = 4;
        track.pointsEdgeLeft = lastPointsEdgeLeft;
        track.pointsEdgeRight = lastPointsEdgeRight;
      }

      lastPointsEdgeLeft = track.pointsEdgeLeft;
      lastPointsEdgeRight = track.pointsEdgeRight;

      pathsEdgeLeft.push_back(track.pointsEdgeLeft); // 记录进厂轨迹
      pathsEdgeRight.push_back(track.pointsEdgeRight);
      break;
    }

    case Step::Stop: //[05] 停车使能
    {
      carStoping = true;
      counterRec++;
      if (counterRec > 10) // 停车：20场 = 2s  停车时间  为多少？
      {
        carStoping = false;
        carExitting = true;
        step = Step::Exit; // 出站使能
        counterRec = 0;
      }
      break;
    }

    case Step::Exit: //[06] 出站使能
    {
      // chu++;

      car_changepid=1;
      carExitting = true;//让电机赋值为负数
      if(entryLeft)
      flagchul=1;//左进库标志位
      else
      flagchur=1;
      cout<<"第一次准备出站辣"<<chu<<endl<<endl;
      // cout<<"左边线尺寸"<<pathsEdgeLeft.size()<<endl;
      // cout<<"右边线尺寸"<<pathsEdgeRight.size()<<endl;  track.stdevLeft>30&&track.stdevLeft<60&&track.stdevRight>5

      if ((track.stdevLeft>20&&track.stdevLeft<60&&track.stdevRight>4 )||(track.stdevRight>20&&track.stdevRight<60&&track.stdevLeft>4 ) ){  //原来为小于1 pathsEdgeLeft.size() < 1 || pathsEdgeRight.size() < 1    也是用延时  chu>80
        cout<<"出站完成辣辣"<<endl<<endl;
        step = Step::None; // 出站完成
        flagchur=0;
        flagchul=0;
        carExitting = false;
        again = true; // 第二次进入救援区标志
        reset();
      } else {
        a=0;
        // track.pointsEdgeLeft = pathsEdgeLeft[pathsEdgeLeft.size() - 1];
        // track.pointsEdgeRight = pathsEdgeRight[pathsEdgeRight.size() - 1];
        // pathsEdgeLeft.pop_back();//回退路线
        // pathsEdgeRight.pop_back();
      }
      break;
    }
    }

    if (step == Step::None) // 返回控制模式标志
      return false;
    else
      return true;
  }

  /**
   * @brief 识别结果图像绘制
   *
   */
  void drawImage(Tracking track, Mat &image) {
    // 赛道边缘
    for (int i = 0; i < track.pointsEdgeLeft.size(); i++) {
      circle(image, Point(track.pointsEdgeLeft[i].y, track.pointsEdgeLeft[i].x),
             1, Scalar(0, 255, 0), -1); // 绿色点
    }
    for (int i = 0; i < track.pointsEdgeRight.size(); i++) {
      circle(image,
             Point(track.pointsEdgeRight[i].y, track.pointsEdgeRight[i].x), 1,
             Scalar(0, 255, 255), -1); // 黄色点
    }

    // 入库状态
    string state = "None";
    switch (step) {
    case Step::Enable:
      state = "Enable";
      break;
    case Step::Enter:
      state = "Enter";
      break;
    case Step::Cruise:
      state = "Cruise";
      break;
    case Step::Stop:
      state = "Stop";
      break;
    case Step::Exit:
      state = "Exit";
      break;
    }
    if (entryLeft) {
      // 绘制锥桶坐标
      for (int i = 0; i < pointConeLeft.size(); i++) {
        circle(image, Point(pointConeLeft[i].y, pointConeLeft[i].x), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - LEFT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
    } else {
      // 绘制锥桶坐标
      for (int i = 0; i < pointConeRight.size(); i++) {
        circle(image, Point(pointConeRight[i].y, pointConeRight[i].x), 2,
               Scalar(92, 92, 205), -1); // 锥桶坐标：红色
      }
      putText(image, "[3] RESCUE - RIGHT", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
    }

    putText(image, state, Point(COLSIMAGE / 2 - 10, 30),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);

    putText(image, to_string(_distance), Point(COLSIMAGE / 2 - 15, 40),
            cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1,
            16); // 显示锥桶距离
    if (_pointNearCone.x > 0)
      circle(image, Point(_pointNearCone.y, _pointNearCone.x), 5,
             Scalar(200, 200, 200), -1);

    if (levelCones > 0)
      line(image, Point(0, levelCones), Point(image.cols, levelCones),
           Scalar(255, 255, 255), 1);

    putText(image, to_string(indexDebug),
            Point(COLSIMAGE / 2 - 10, ROWSIMAGE - 20), cv::FONT_HERSHEY_TRIPLEX,
            0.3, cv::Scalar(0, 0, 255), 1, 16);
  }

private:
  float distance_now;
  float distance_in;  
  bool again = false; // 第二次进入救援区标志
  double _distance = 0;
  int levelCones = 0; // 锥桶的平均高度
  POINT _pointNearConeL;
  POINT _pointNearConeR;
  POINT _pointNearCone;
  POINT pointHCone;
  vector<POINT> pointConeLeft;      // AI元素检测边缘点集
  vector<POINT> pointConeRight;     // AI元素检测边缘点集
  vector<POINT> lastPointsEdgeLeft; // 记录上一场边缘点集（丢失边）
  vector<POINT> lastPointsEdgeRight;

  vector<vector<POINT>> pathsEdgeLeft; // 记录入库路径
  vector<vector<POINT>> pathsEdgeRight;
  int indexDebug = 0;

  uint16_t counterSession = 0;  // 图像场次计数器
  uint16_t counterRec = 0;       // 标志检测计数器  uint16_t counterRec = 0;   
  uint16_t counterExit = 0;     // 标志结束计数器
  uint16_t counterImmunity = 0; // 屏蔽计数器
  Mapping ipm =
      Mapping(Size(COLSIMAGE, ROWSIMAGE), Size(COLSIMAGEIPM, ROWSIMAGEIPM));
  /**
   * @brief 从AI检测结果中检索锥桶坐标集合
   *
   * @param predict AI检测结果
   * @return vector<POINT>
   */
  void searchCones(vector<PredictResult> predict,Motion &motion) {
    pointConeLeft.clear();
    pointConeRight.clear();
    for (int i = 0; i < predict.size(); i++) {
      if (predict[i].type == LABEL_CONE) // 锥桶检测
      {
        if(predict[i].height*predict[i].width>motion .params.areaMax)continue;////////如果锥桶面积大于一定值，就排除他
        if ((predict[i].x + predict[i].width / 2) < COLSIMAGE / 2)
          pointConeLeft.push_back(POINT(predict[i].y + predict[i].height,
                                        predict[i].x + predict[i].width));
        else
          pointConeRight.push_back(
              POINT(predict[i].y + predict[i].height, predict[i].x));
      }
    }
  }

  /**
   * @brief 搜索距离赛道左边缘最近的锥桶坐标
   *
   * @param pointsEdgeLeft 赛道边缘点集
   * @param predict AI检测结果
   * @return POINT
   */
  POINT getConeLeftDown(vector<POINT> pointsEdgeLeft,
                        vector<POINT> pointsCone) {
    POINT point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeLeft.size() < 10)
      return point;

    for (int i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeLeft[pointsEdgeLeft.size() - 1].x < pointsCone[i].x) {
        int row = pointsEdgeLeft[0].x - pointsCone[i].x;
        if (row > 0 && row < pointsEdgeLeft.size()) {
          int dis = pointsEdgeLeft[row].y - pointsCone[i].y;
          if (dis < disMin && pointsCone[i].x > ROWSIMAGE / 4 &&
              pointsCone[i].x > point.x) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }

    return point;
  }

  /**
   * @brief 搜索距离赛道右边缘最近的锥桶坐标
   *
   * @param pointsEdgeRight 赛道边缘点集
   * @param predict AI检测结果
   * @return POINT
   */
  POINT getConeRightDown(vector<POINT> pointsEdgeRight,
                         vector<POINT> pointsCone) {
    POINT point(0, 0);
    double disMin = 60; // 右边缘锥桶离赛道左边缘最小距离

    if (pointsCone.size() <= 0 || pointsEdgeRight.size() < 10)
      return point;

    for (int i = 0; i < pointsCone.size(); i++) {
      if (pointsEdgeRight[pointsEdgeRight.size() - 1].x < pointsCone[i].x) {
        int row = pointsEdgeRight[0].x - pointsCone[i].x;
        if (row > 0 && row < pointsEdgeRight.size()) {
          int dis = pointsCone[i].y - pointsEdgeRight[row].y;
          if (dis < disMin && pointsCone[i].x > ROWSIMAGE / 4 &&
              pointsCone[i].x > point.x) {
            point = pointsCone[i];
            _distance = dis;
          }
        }
      }
    }

    return point;
  }

  /**
   * @brief 在俯视域由左边缘预测右边缘
   *
   * @param pointsEdgeLeft
   * @return vector<POINT>
   */
  vector<POINT> predictEdgeRight(vector<POINT> &pointsEdgeLeft) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeRight;
    if (pointsEdgeLeft.size() < 3)
      return pointsEdgeRight;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeLeft[0].y, pointsEdgeLeft[0].x)); // 透视变换
    Point2d prefictRight = Point2d(startIpm.x + offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT startPoint = POINT(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() / 2].y,
                pointsEdgeLeft[pointsEdgeLeft.size() / 2].x)); // 透视变换
    prefictRight = Point2d(middleIpm.x + offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);   // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeLeft[pointsEdgeLeft.size() - 1].y,
                pointsEdgeLeft[pointsEdgeLeft.size() - 1].x)); // 透视变换
    prefictRight = Point2d(endIpm.x + offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictRight); // 反透视变换
    POINT endPoint = POINT(endtIipm.y, endtIipm.x);

    // 补线
    vector<POINT> input = {startPoint, midPoint, endPoint};
    vector<POINT> repair = Bezier(0.05, input);

    for (int i = 0; i < repair.size(); i++) {
      if (repair[i].x >= ROWSIMAGE)
        repair[i].x = ROWSIMAGE - 1;

      else if (repair[i].x < 0)
        repair[i].x = 0;

      else if (repair[i].y >= COLSIMAGE)
        repair[i].y = COLSIMAGE - 1;
      else if (repair[i].y < 0)
        repair[i].y = 0;

      pointsEdgeRight.push_back(repair[i]);
    }

    return pointsEdgeRight;
  }

  /**
   * @brief 在俯视域由右边缘预测左边缘
   *
   * @param pointsEdgeRight
   * @return vector<POINT>
   */
  vector<POINT> predictEdgeLeft(vector<POINT> &pointsEdgeRight) {
    int offset = 120; // 右边缘平移尺度
    vector<POINT> pointsEdgeLeft;
    if (pointsEdgeRight.size() < 3)
      return pointsEdgeLeft;

    // Start
    Point2d startIpm = ipm.homography(
        Point2d(pointsEdgeRight[0].y, pointsEdgeRight[0].x)); // 透视变换
    Point2d prefictLeft = Point2d(startIpm.x - offset, startIpm.y);
    Point2d startIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT startPoint = POINT(startIipm.y, startIipm.x);

    // Middle
    Point2d middleIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() / 2].y,
                pointsEdgeRight[pointsEdgeRight.size() / 2].x)); // 透视变换
    prefictLeft = Point2d(middleIpm.x - offset, middleIpm.y);
    Point2d middleIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT midPoint = POINT(middleIipm.y, middleIipm.x);  // 补线中点

    // End
    Point2d endIpm = ipm.homography(
        Point2d(pointsEdgeRight[pointsEdgeRight.size() - 1].y,
                pointsEdgeRight[pointsEdgeRight.size() - 1].x)); // 透视变换
    prefictLeft = Point2d(endIpm.x - offset, endIpm.y);
    Point2d endtIipm = ipm.homographyInv(prefictLeft); // 反透视变换
    POINT endPoint = POINT(endtIipm.y, endtIipm.x);

    // 补线

    vector<POINT> input = {startPoint, midPoint, endPoint};
    vector<POINT> repair = Bezier(0.05, input);

    for (int i = 0; i < repair.size(); i++) {
      if (repair[i].x >= ROWSIMAGE)
        repair[i].x = ROWSIMAGE - 1;

      else if (repair[i].x < 0)
        repair[i].x = 0;

      else if (repair[i].y >= COLSIMAGE)
        repair[i].y = COLSIMAGE - 1;
      else if (repair[i].y < 0)
        repair[i].y = 0;

      pointsEdgeLeft.push_back(repair[i]);
    }

    return pointsEdgeLeft;
  }

  /**
   * @brief 按照坐标点的y排序
   *
   * @param points
   * @return vector<int>
   */
  void pointsSortForY(vector<POINT> &points) {
    int n = points.size();
    bool flag = true;

    for (int i = 0; i < n - 1 && flag; i++) {
      flag = false;
      for (int j = 0; j < n - i - 1; j++) {
        if (points[j].y > points[j + 1].y) {
          POINT temp = points[j];
          points[j] = points[j + 1];
          points[j + 1] = temp;
          flag =
              true; // 每次循环i有修改，这里为true
                    // 如果跑了一次I没有发生交换的情况，说明已经排序完成，不需要再跑后面的i
        }
      }
    }
  }
};


  // /**
  //  * @brief 去除不合理的锥桶 // 框面积
  //  *
  //  * @param pointsEdgeRight
  //  * @void
  //  */void  cone_square( vector<PredictResult> resultsObs,int areaMax)
  //  {
  // // 选取距离最近的锥桶，面积最大的就最近
  //   int index = 0;   // 目标序号
  //   for (int i = 0; i < resultsObs.size(); i++) {
  //     int area = resultsObs[i].width * resultsObs[i].height;
  //     if (resultsObs[i].type == LABEL_CONE) {
  //       if (area >= areaMax) {
  //         index = i;
  //         areaMax = area;
  //       }
  //     }
  //   }
  //  }