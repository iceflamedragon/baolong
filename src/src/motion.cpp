/**
 ********************************************************************************************************
 *                                               示例代码
 *                                             EXAMPLE  CODE
 *
 *                      (c) Copyright 2024; SaiShu.Lcc.; Leo;
 *https://bjsstech.com 版权所属[SASU-北京赛曙科技有
 限公司]
 *
 *            The code is for internal use only, not for commercial
 *transactions(开源学习,请勿商用). The code ADAPTS the corresponding hardware
 *circuit board(代码适配百度Edgeboard-智能汽车赛事版), The specific details
 *consult the professional(欢迎联系我们,代码持续更正，敬请关注相关开源渠道).
 *********************************************************************************************************
 * @file motion.cpp
 * @author Leo
 * @brief 运动控制器：PD姿态控制||速度控制
 * @version 0.1
 * @date 2023-12-26
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "../include/common.hpp"
#include "../include/json.hpp"
#include "./controlcenter.cpp"

#include <cmath>
#include <fstream>
#include <iostream>

using namespace std;
using namespace cv;

/**
 * @brief 运动控制器
 *
 */
class Motion {
private:
  int countShift = 0; // 变速计数器
  float p1_former;
  float p2_former;
  float d_former;
  float i_former;
  // Ring ring;                // 环岛识别类

public:
  /**
   * @brief 初始化：加载配置文件
   *
   */
  double angle_p;
  double angle;
  int flagbigringl;
  int flagbigringr;
  int flag = 0;
  Motion() {
    string jsonPath = "../src/config/config.json";
    std::ifstream config_is(jsonPath);
    if (!config_is.good()) {
      std::cout << "Error: Params file path:[" << jsonPath << "] not find .\n";
      exit(-1);
    }

    nlohmann::json js_value;
    config_is >> js_value;

    try {
      params = js_value.get<Params>();
    } catch (const nlohmann::detail::exception &e) {
      std::cerr << "Json Params Parse failed :" << e.what() << '\n';
      exit(-1);
    }

    speed = params.speedLow;
    cout << "--- runP1:" << params.runP1 << " | runP2:" << params.runP2
         << " | runP3:" << params.runP3 << endl;
    cout << "--- turnP:" << params.turnP << " | turnD:" << params.turnD << endl;
    cout << "--- speedLow:" << params.speedLow
         << "m/s  |  speedHigh:" << params.speedHigh << "m/s" << endl;
  };

  /**
   * @brief 控制器核心参数
   *
   */
  struct Params {

    int submit;
    float isrising_breakl;
    float isrising_breakr;
    float ringinbreakl;
    float ringinbreakr;
    float rescueout_distance;
    float dangerout_distance;
    float ringout_l;
    float ringout_r;
    float rescue_error_l;
    float rescue_error_r;
    int record_video;
    float runP1_fast;
    float runP2_fast;
    float turnD_fast;
    float ring_p1b; // 圆环的pid
    float ring_p2b;
    float ring_db;
    float ring_p1s; // 圆环的pid
    float ring_p2s;
    float ring_ds;
    bool motion_start;
    float danger_p1;
    float danger_p2;
    float danger_d;
    int areaMax;
    double angle_p;
    float speedLowpro;
    float speedHighpro;
    int Danger_distance;
    int Rescue_distance;
    int Bridge_distance;
    float turn_PIDkp;
    float turn_PIDkd;
    float gyroturn_PIDkp;
    float gyroturn_PIDki;
    float gyroturn_PIDkd;
    float loop_turn_PIDkp;
    float loop_turn_PIDkd;
    float big_loop_PIDkp;
    float big_loop_PIDkd;
    float camwf;
    float camwl;
    float camwr;
    int show_params_mode;
    int None_distance;
    float speedLow = 1.5;        // 智能车最低速
    float speedHigh = 4;         // 智能车最高速
    float speedBridge = 0.6;     // 坡道速度
    float speedDown = 0.5;       // 特殊区域降速速度
    float runP1 = 0;             // 一阶比例系数：直线控制量
    float runP2 = 0;             // 二阶比例系数：弯道控制量
    float runP3 = 0.0;           // 三阶比例系数：弯道控制量
    float turnP = 3.5;           // 一阶比例系数：转弯控制量
    float turnD = 0;             // 一阶微分系数：转弯控制量
    float turnI;                 // 积分项--减少漂移情况
    bool debug = false;          // 调试模式使能
    bool saveImg = false;        // 存图使能
    uint16_t rowCutUp = 200;     // 图像顶部切行
    uint16_t rowCutBottom = 200; // 图像顶部切行
    bool bridge = true;          // 坡道区使能
    bool danger = true;          // 危险区使能
    bool rescue = true;          // 救援区使能
    bool racing = true;          // 追逐区使能
    bool parking = true;         // 停车区使能
    bool ring = true;            // 环岛使能
    bool cross = true;           // 十字道路使能
    float score = 0.5;           // AI检测置信度
    int stop_num;
    float speed_max; /////////////速度决策
    float speed_add; ///////////
    float speed_min;
    float loop_target_speed;
    float loop_out_distance;
    float STEER_MID;
    int STEER_MIN;
    int STEER_MAX;
    bool Is_showimg;//显示原图
    string model = "../res/model/yolov3_mobilenet_v1"; // 模型路径
    string video = "../res/samples/demo.mp4";          // 视频路径
    NLOHMANN_DEFINE_TYPE_INTRUSIVE(
        Params, speedLow, speedHigh, turn_PIDkp, STEER_MID,STEER_MIN,STEER_MAX,Is_showimg, turn_PIDkd,
        speed_max, speed_add, speed_min, gyroturn_PIDkp, gyroturn_PIDki,
        gyroturn_PIDkd, loop_turn_PIDkp, loop_turn_PIDkd, big_loop_PIDkp,
        big_loop_PIDkd, camwf, camwl, camwr, speedBridge, speedDown, runP1,
        runP2, runP3, turnP, turnD, turnI, debug, saveImg, rowCutUp,
        rowCutBottom, bridge, danger, rescue, racing, parking, ring, cross,
        score, model, ring_p1b, ring_p2b, ring_db, record_video, video, areaMax,
        show_params_mode, submit, angle_p, loop_target_speed,
        loop_out_distance); // 添加构造函数
  };

  Params params; // 读取控制参数
  // 切换舵机pid并且保存原来数字，以用来还原
  void set_direction_pid(float p1, float p2, float d, float i) {
    p1_former = params.runP1;
    p2_former = params.runP2;
    d_former = params.turnD;
    i_former = params.turnI;
    params.runP1 = p1;
    params.runP2 = p2;
    params.turnD = d;
    params.turnI = i;
    cout << "p2的值" << p2 << endl;
    cout << "切换舵机pid" << endl;
  }

  // 还原舵机pid
  void reset_direction_pid(void) {
    params.runP1 = p1_former;
    params.runP2 = p2_former;
    params.turnD = d_former;
    params.turnI = i_former;
    cout << "恢复舵机pid" << endl;
  }
  uint16_t servoPwm = PWMSERVOMID; // 发送给舵机的PWM
  float speed = 0.3;               // 发送给电机的速度
  /**
   * @brief 姿态PD控制器  新加入I项
   *
   * @param controlCenter 智能车控制中心
   */

  float errorsum;
  void poseCtrl(int controlCenter, ControlCenter &control) {
    // if(ring.flagpid )flag=1;
    // cout<<"点集加权偏差值"<<controlCenter - COLSIMAGE / 2<<endl;
    if (angle < 0) {
      // cout<<"角度为负值"<<endl;
      controlCenter +=
          (abs(angle) - 90) *
          params.angle_p; // 车偏右，controlCenter小，angle为负的，直接加即可
      // cout<<"角度系数"<<params.angle_p<<endl;
      // cout<<"角度偏差"<<(abs(angle)-90)*params.angle_p<<endl;
    } else if (angle > 0) {
      controlCenter += (90 - angle) * params.angle_p;
      // cout<<"角度为正值"<<endl;
    } else {
      controlCenter += angle * params.angle_p; // angle_p乘的系数记得加params.
      // cout<<"角度为零"<<endl;
    }
    float error = controlCenter - COLSIMAGE / 2; // error为负向左转
    // cout<<"角度偏差值"<<error<<endl;

    // control.submiterror=params.submit;

    //  if(flagbigringl)
    //  {
    //   error=error;//加30
    //  }
    //  else if(flagbigringr)
    //  {
    //    error=error;//减30
    //  }
    //  else
    //  {
    //   error=error;
    //  }
    // cout<<"flag的值是    "<<flag<<endl<<endl;
    //  if(flag)
    // {
    //    cout<<"切换为圆环内的error求法了"<<endl;

    // if(error<0)
    // {
    // error=-pow(abs(error),0.5);
    // }
    // else {
    //   error=pow(abs(error),0.5);
    // }
    //  }
    //  else error = controlCenter - COLSIMAGE / 2;
    // cout<<error<<endl;
    // 图像控制中心转换偏差

    static int errorLast = 0; // 记录前一次的偏差
    if (abs(error - errorLast) > COLSIMAGE / 10) {
      error = error > errorLast ? errorLast + COLSIMAGE / 10
                                : errorLast - COLSIMAGE / 10;
    }
    errorsum += error; // 加个限幅加和

    // cout << "此时的P2值" << params.runP2 << endl << endl;
    params.turnP = abs(error) * params.runP2 + params.runP1;
    int pwmDiff = (error * params.turnP) + (error - errorLast) * params.turnD +
                  errorsum * params.turnI;
    errorLast = error;

    servoPwm = PWMSERVOMID - pwmDiff;
    if (servoPwm > 950)
      servoPwm = 950;
    if (servoPwm < 550)
      servoPwm = 550;
    // (uint16_t)(750 - pwmDiff); // PWM转换
    // ~~^~~~~~~~~~~~~~~~~~~~~~~PWMSERVOMID - pwmDiff
    // cout << "舵机pwm" << PWMSERVOMID - pwmDiff << endl;
  }

  /**
   * @brief 变加速控制
   *
   * @param enable 加速使能
   * @param control
   */
  void speedCtrl(bool enable, bool slowDown, ControlCenter control,
                 bool speedup) {
    // 控制率
    uint8_t controlLow = 0;   // 速度控制下限
    uint8_t controlMid = 5;   // 控制率
    uint8_t controlHigh = 10; // 速度控制上限

    if (slowDown) {
      countShift = controlLow;
      speed = params.speedDown;
    } else if (enable) // 加速使能
    {
      if (!speedup) {
        if (control.centerEdge.size() < 10) {
          speed = params.speedLow;
          countShift = controlLow;
          return;
        }
        if (control.centerEdge[control.centerEdge.size() - 1].x >
            ROWSIMAGE / 2) {
          speed = params.speedLow;
          countShift = controlLow;
          return;
        }
        if (abs(control.sigmaCenter) < 100.0) {
          countShift++;
          if (countShift > controlHigh)
            countShift = controlHigh;
        } else {
          countShift--;
          if (countShift < controlLow)
            countShift = controlLow;
        }

        if (countShift > controlMid)
          speed = params.speedHigh;
        else
          speed = params.speedLow;
      } else {
        if (control.centerEdge.size() < 10) {
          speed = params.speedLowpro;
          countShift = controlLow;
          return;
        }
        if (control.centerEdge[control.centerEdge.size() - 1].x >
            ROWSIMAGE / 2) {
          speed = params.speedLowpro;
          countShift = controlLow;
          return;
        }
        if (abs(control.sigmaCenter) < 100.0) {
          countShift++;
          if (countShift > controlHigh)
            countShift = controlHigh;
        } else {
          countShift--;
          if (countShift < controlLow)
            countShift = controlLow;
        }

        if (countShift > controlMid)
          speed = params.speedHighpro;
        else
          speed = params.speedLowpro;
      }
    } else {
      countShift = controlLow;
      speed = params.speedLow;
    }
  }
};
