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
 * @file parking.cpp
 * @author
 * @brief 停车区AI识别与路径规划
 * @version 0.1
 * @date 2024-01-02
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "../../include/common.hpp"
#include "../../include/detection.hpp" // Ai模型预测
#include <cmath>
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
// #include "../CAM/global.hpp"


using namespace std;
using namespace cv;
// ///////////////////////////////////
// #define base_line 5   //基础行
// #define _EDGE_STORE_SIZE 22 //扫描时可容纳的赛道跳变沿数量最大值,必须为偶数
// #define _MIN_EDGE_WIDTH 20  //绝对最小赛道宽度
// #define forward_near 0      //最近端
// #define forward_far 115     //最远端
// struct lineinfo_s
// {
//         int16_t y;          //0~119  行数
//         int16_t left;       //0~160  本行车道左线所在列数
//         int16_t right;      //0~160  本行车道右线所在列数
//         int16_t edge_count; //0~160  本行中跳变沿总数
//         uint8_t edge_store[_EDGE_STORE_SIZE];
//         int16_t left_adjust; //补线后的左线
//         int16_t right_adjust;//补线后的右线
//         int left_lost;            //左线丢失
//         int right_lost;           //右线丢失
//         uint8_t zebra_flag; //斑马线标志

//         int16_t persp_lx; //0~255  //本行左线逆透视后x轴坐标
//         int16_t persp_ly; //0~255  //本行左线逆透视后y轴坐标
//         int16_t persp_rx; //0~255  //本行右线逆透视后x轴坐标
//         int16_t persp_ry; //0~255  //本行右线逆透视后y轴坐标

//         int16_t angel_left;
//         int16_t angel_right;

//         //整行线全部丢失标记变量 1：本行线全部丢失  0：本行线没有丢失
//         int16_t whole_lost;
// };
// extern struct lineinfo_s lineinfo[120];
// extern unsigned char our_img[ROWSIMAGE][COLSIMAGE];
///////////////////////////////////////////
/**
 * @brief 停车区AI识别与路径规划类
 *
 */
class Parking {
private:
  /**
   * @brief 场景状态
   *
   */
  enum Step {
    init = 0, // 初始化屏蔽
    det,      // AI标识检测
    enable,   // 场景使能
    stop      // 准备停车
  };
  Step step = Step::init; // 场景状态
  uint16_t countRec = 0;  // AI场景识别计数器
  uint16_t countSes = 0;  // 场次计数器

public:
  uint16_t countExit = 0; // 程序退出计数器
  bool park = false;      // 停车标志
  /**
   * @brief 停车区AI识别与路径规划处理
   *
   * @param predict AI检测结果
   * @return true
   * @return false
   */
  bool process(vector<PredictResult> predict) {
    switch (step) {
    case Step::init: // 初始化：起点斑马线屏蔽
      countSes++;
      for (int i = 0; i < predict.size(); i++) {
        if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
        {
          if ((predict[i].y + predict[i].height) >
              ROWSIMAGE * 0.2) // 标志距离计算
          {
            countSes = 0;
            break;
          }
        }
      }
      if (countSes > 50) {
        countSes = 0;
        step = Step::det;
      }
      break;

    case Step::det: // AI未识别
      for (int i = 0; i < predict.size(); i++) {
        if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
        {
          if ((predict[i].y + predict[i].height) >
              ROWSIMAGE * 0.4) // 标志距离计算
          {
            countRec++;
            break;
          }
        }
      }
      if (countRec) // 识别AI标志后开始场次计数
        countSes++;

      if (countSes >= 8) {
        if (countRec >= 4) {
          step = Step::enable;
        }

        countRec = 0;
        countSes = 0;
      }
      break;

    case Step::enable: // 场景使能: 检测斑马线标识丢失
      countSes++;
      for (int i = 0; i < predict.size(); i++) {
        if (predict[i].type == LABEL_CROSSWALK) // AI识别标志
        {
          if (predict[i].y > ROWSIMAGE * 0.3) // 标志距离计算
          {
            countSes = 0;
            break;
          }
        }
      }
      if (countSes > 5) {
        countExit = 0;
        step = Step::stop;
      }
      break;

    case Step::stop: // 准备停车
      park = true;
      countExit++; // 停车倒计时
      break;
    }

    // 输出场景状态结果
    if (step == Step::init || step == Step::det)
      return false;
    else
      return true;
  }

  /**
   * @brief 图像绘制禁行区识别结果
   *
   * @param img 需要叠加显示的图像
   */
  void drawImage(Mat &img) {
    if (step == Step::enable)
      putText(img, "[5] PARK - ENABLE", Point(COLSIMAGE / 2 - 30, 10),
              cv::FONT_HERSHEY_TRIPLEX, 0.3, cv::Scalar(0, 255, 0), 1, 16);
  }
   
// //////////////////////////////////////////////

//  /**
//    * @brief 检测是否停车
//    */

// void zebra_parking(void)
// {
//     for (int y = base_line-1; y >= forward_near; y--) //搜索斑马线
//     {
//         line_findnext(&lineinfo[y], our_img[239-y], &lineinfo[y + 1]);
//     }
// }

// /*
//  *存储所有黑白跳变沿位置到数组edge_store中
//  */
// uint8_t get_orign_edges(uint8_t *inputimg, uint8_t *edge_store)
// {
// #if _EDGE_STORE_SIZE % 2 != 0
// #error "_EDGE_STORE_SIZE must be even!"
// #endif
//     //查找并存储所有跳变沿对
  
//    int threshold=100;//给的固定阈值，实际在这里并无意义
//     uint8_t edge_store_idx = 0;
//     if (inputimg[0] >threshold)//数组第0位大于阈值（图像最左端为白）则认为此处为跳变沿
//     {
//         edge_store[edge_store_idx] = 0;
//         edge_store_idx++;
//     }
//     for (uint8_t px = 1; px < COLSIMAGE; px++)
//     {
//         //if ((inputimg[px - 1]-threshold)*(inputimg[px]-threshold) <= 0) //分布在threshold两侧认为是跳边沿。。不稳定
//         if(!((inputimg[px-1]>threshold&&inputimg[px]<threshold)||(inputimg[px-1]<threshold&&inputimg[px]>threshold)))    //可变灰度区分值
//             continue;
//         if (edge_store_idx >= _EDGE_STORE_SIZE)
//             break;
//         edge_store[edge_store_idx] = px;   //将跳变沿存入数组
//         edge_store_idx++;
//     }
//     if (inputimg[COLSIMAGE - 1] >threshold) //数组最后一位大于阈值（图像最右端为白）则认为此处为跳变沿
//     {
//         if (edge_store_idx < _EDGE_STORE_SIZE)
//         {
//             edge_store[edge_store_idx] = COLSIMAGE - 1;
//             edge_store_idx++;
//         }
//     }
//     return edge_store_idx;
// }
// /*
//  * 从基准行向下搜线
//  */
// int line_findnext(struct lineinfo_s *lineinfo, uint8_t *inputimg, struct lineinfo_s *lineinfo_ref)
// {
//     //得到所有凸边沿
//     uint8_t *edge_store = lineinfo->edge_store;
//     lineinfo->edge_count = get_orign_edges(inputimg, edge_store);
//      if (lineinfo->y > 20 && lineinfo->y < 100){
//          zebra_detect(lineinfo, edge_store, inputimg); //斑马线检测    
//      }
//     return 0;
// }
// //斑马线识别函数
// int zebra_detect(struct lineinfo_s *lineinfo, uint8_t *edge_store, uint8_t *inputimg)
// {
//     uint8_t white_width, zobra_white_count, edge_now;
//     zobra_white_count = 0;
//         if (lineinfo->edge_count > 12)
//         {
//             for (uint8_t k = 0; k < lineinfo->edge_count; k += 2)
//             {
//                 edge_now = edge_store[k];
//                 if (edge_now < COLSIMAGE)
//                 {
//                     if ((inputimg[edge_now] & 0x80) == 0x80)
//                     {
//                         white_width = edge_store[k + 1] - edge_store[k];
//                         if (white_width > 1 && white_width < 14)    ///根据图象大小不同要改
//                         {
//                             zobra_white_count++;
//                          //   watch.ZebraInLine=lineinfo->y;//取满足条件时y最大值作为ZebraLine
//                         }                 //watch.ZebraInLine为看到的斑马线最远端
//                     }
//                 }
//             }
//             if (zobra_white_count > 6)
//             {
//                 lineinfo->zebra_flag = 1;
//                 //beep(20);
//                 // if(!mycar.status)
//                 //     ips200_draw_horizon(0, 119 - lineinfo->y, 188, 119 - lineinfo->y, BLUE);
//             }
//         }
//         else
//         {
//             lineinfo->zebra_flag = 0;
//         }
//     return 0;
// }
};
