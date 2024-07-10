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
 * @file icar.cpp
 * @author Leo
 * @brief 智能汽车-顶层框架（TOP）
 * @version 0.1
 * @date 2023-12-25
 * @copyright Copyright (c) 2024
 *
 */
#include "../include/common.hpp"     //公共类方法文件
#include "../include/detection.hpp"  //百度Paddle框架移动端部署
#include "../include/uart.hpp"       //串口通信驱动
#include "controlcenter.cpp"         //控制中心计算类
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/danger.cpp"      //AI检测：危险区
#include "detection/parking.cpp"     //AI检测：停车区
#include "detection/racing.cpp"      //AI检测：追逐区
#include "detection/rescue.cpp"      //AI检测：救援区
#include "motion.cpp"                //智能车运动控制类
#include "preprocess.cpp"            //图像预处理类
#include "recognition/crossroad.cpp" //十字道路识别与路径规划类
#include "recognition/ring.cpp"      //环岛道路识别与路径规划类
#include "recognition/tracking.cpp"  //赛道识别基础类
#include <iostream>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>

using namespace std;
using namespace cv;
bool app_stopped = false;
void sigint_handler(int sig);
int flag = 1;
int start = 0;                        // 发车计数器
int center_sum = 0, center_sum_n = 0; // 中心总值 ,计数
shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
int main(int argc, char const *argv[]) {
  Preprocess preprocess;    // 图像预处理类
  Motion motion;            // 运动控制类
  Tracking tracking;        // 赛道识别类
  Crossroad crossroad;      // 十字道路识别类
  Ring ring;                // 环岛识别类
  Bridge bridge;            // 坡道区检测类
  Parking parking;          // 停车区检测类
  Danger danger;            // 危险区检测类
  Rescue rescue;            // 救援区检测类
  Racing racing;            // 追逐区检测类
  ControlCenter ctrlCenter; // 控制中心计算类
  Display display(4);       // 初始化UI显示窗口
  VideoCapture capture(0);  // Opencv相机类

  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度

  // USB转串口初始化： /dev/ttyUSB0
  // if(ring.flagpid){
  // motion.flag=1;
  // cout<<"主函数中设置成进环pid了"<<endl;
  // }else  motion.flag=0;
  // cout<<"motion中的flag值"<<motion.flag<<endl;
  int ret = uart->open();
  if (ret != 0) {
    printf("[Error] Uart Open failed!\n");
    return -1;
  }
  uart->startReceive(); // 启动数据接收子线程

  // USB摄像头初始化
  // if (motion.params.debug)

  // capture = VideoCapture(motion.params.video); // 打开本地视频

  cv::Size S = cv::Size((int)capture.get(CAP_PROP_FRAME_WIDTH),
                        (int)capture.get(CAP_PROP_FRAME_HEIGHT));

  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }
  VideoWriter video("ouput.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                    30, Size(1280, 240), true);
  capture.set(CAP_PROP_FRAME_WIDTH, COLSIMAGE);  // 设置图像分辨率
  capture.set(CAP_PROP_FRAME_HEIGHT, ROWSIMAGE); // 设置图像分辨率

  // 等待按键发车
  // if (!motion.params.debug) {
  //   printf("--------------[等待按键发车!]-------------------\n");
  //   uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
  //   while (!uart->keypress)
  //     waitKey(300);
  //   while (ret < 10) // 延时3s
  //   {
  //     uart->carControl(0, PWMSERVOMID); // 通信控制车辆停止运动
  //     waitKey(300);
  //     ret++;
  //   }
  //   uart->keypress = false;
  //   uart->buzzerSound(uart->BUZZER_START); // 祖传提示音效
  // }

  // 初始化参数
  Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态
  long preTime;
  Mat img;
  uart->carpid(300, 750, 0, 0); // 调pid，参数分别为p，i，d，是否存入flash
  // clock_t startTime, endTime;     // 统计程序时间
  signal(SIGINT, sigint_handler); // 中断，结束的时候

  int s1 = 1;
  string s2 = ".jpg";
  float mpu6050_now;
  float mpu6050_later;
  float distance_now;
  int ai_check = 0;

  while (1) {
    //  ring.RoundaboutGetArc(tracking, 1, 20, 30, 160);

    if (ring.flagbigringl) {
      motion.flagbigringl = 1;

    } else if (ring.flagbigringr) {
      motion.flagbigringr = 1;
    } else {
      motion.flagbigringl = 0;
      motion.flagbigringr = 0;
    }
    //   cout<<"救援区出站时的flag值"<<rescue.flagchu<<endl;
    // //   if(ring.flagpid){
    // motion.flag=1;
    // cout<<"主函数中设置成进环pid了"<<endl;
    // }else  motion.flag=0;
    // cout<<"motion中的flag值"<<motion.flag<<endl;
    // if(std::cin.rdbuf()->in_avail()>0)
    // {
    //   char c =std::cin.get();
    //   if(c==' ')
    //   {cout<<"发车"<<endl<<endl<<endl;
    //   uart->carpid(300, 750, 0, 0); //
    //   调pid，参数分别为p，i，d，是否存入flash}
    // }
    // }
    mpu6050_now = uart->get_mpu6050();   // mpu6050_now就是mpu的数值
    distance_now = uart->get_distance(); // 编码器获取

    cout << mpu6050_now << endl; // 输出mpu

    ring.setmpu6050(mpu6050_now);
    ring.setdistance(distance_now);
    rescue.setdistancere(distance_now);
    crossroad.setmpu6050(mpu6050_now);
    preTime = chrono::duration_cast<chrono::milliseconds>(
                  chrono::system_clock::now().time_since_epoch())
                  .count();
    // startTime = clock();
    //[01] 视频源读取
    // 读取mpu6050

    if (motion.params.debug) // 综合显示调试UI窗口
      preTime = chrono::duration_cast<chrono::milliseconds>(
                    chrono::system_clock::now().time_since_epoch())
                    .count();
    if (!capture.read(img))
      continue;
    // if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
    //   savePicture(img);
    if (waitKey(1) == 27) { // 如果用户按下 ESC 键，退出循环

      // s1++;
      // imwrite("../res/calibration/temp/" + to_string(s1) + s2, img);
      // cout << "../res/calibration/temp/" + to_string(s1) + s2 << endl;
      uart->carpid(300, 750, 0, 0); // 调pid，参数分别为p，i，d，是否存入flash
      // cout << "fache" << endl << endl << endl << endl << endl;

      // 按键发车
    }
    //[02] 图像预处理

    Mat imgCorrect = img; // 图像矫正（已停止
    Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化
    Mat element = getStructuringElement(
        MORPH_RECT, Size(9, 9)); // 小于8*8方块的白色噪点都会被腐蚀
    erode(imgBinary, imgBinary, element);
    cout << "scene" << scene << endl;
    if (ai_check > 1 || detection->ai_flag && sceneLast != Scene::RingScene) {

      //[03] 启动AI推理
      detection->inference(imgCorrect);
      ai_check = 0;
    }
    //  detection->inference(imgCorrect);
    //   detection->set_ai_flag(0);//清零ai标志
    //   ai_check++;
    auto startTime = chrono::duration_cast<chrono::milliseconds>(
                         chrono::system_clock::now().time_since_epoch())
                         .count();
    printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
           1000.0 / (startTime - preTime));
    //[04] 赛道识别
    tracking.rowCutUp = motion.params.rowCutUp; // 图像顶部切行（前瞻距离）
    tracking.rowCutBottom =
        motion.params.rowCutBottom; // 图像底部切行（盲区距离）
    tracking.trackRecognition(imgBinary); // 扫线
    Mat imgTrack = imgCorrect.clone();
    if (motion.params.debug) // 综合显示调试UI窗口
    {

      tracking.drawImage(imgTrack); // 图像绘制赛道识别结果
      if (flag) {
        display.setNewWindow(2, "Track", imgTrack);
      }
    }
    // cout << "距离积分" << uart->get_distance();
    //[05] 停车区检测
    if (motion.params.parking) {
      if (parking.process(detection->results)) {
        scene = Scene::ParkingScene;
        if (parking.countExit > 10) {       /////刹车时间计数器
          uart->carpid(500, 1000, 0, 0);    // 刹车pid
          uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
          sleep(1);
          printf("-----> 检测到停车区!!! <-----\n");
          exit(0); // 程序退出
        }
      }
    }

    //[06] 救援区检测
    if ((scene == Scene::NormalScene || scene == Scene::RescueScene) &&
        motion.params.rescue) {
      if (rescue.process(tracking, detection->results)) {
        scene = Scene::RescueScene;
        if (rescue.entryLeft)
          printf("Rescue Left\n");
        else
          printf("Rescue Right\n");
      } else
        scene = Scene::NormalScene;
    }

    //[07] 追逐区检测
    if ((scene == Scene::NormalScene || scene == Scene::RacingScene) &&
        motion.params.racing) {
      if (racing.process(tracking, detection->results))
        scene = Scene::RacingScene;
      else
        scene = Scene::NormalScene;
    }

    //[08] 坡道区检测
    if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
        motion.params.bridge) {
      if (bridge.process(tracking, detection->results))
        scene = Scene::BridgeScene;
      else
        scene = Scene::NormalScene;
    }

    // [09] 危险区检测
    if ((scene == Scene::NormalScene || scene == Scene::DangerScene) &&
        motion.params.danger) {
      if (danger.process(tracking, detection->results, motion)) {
        uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
        scene = Scene::DangerScene;
      } else
        scene = Scene::NormalScene;
    }

    // [10] 十字道路识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::CrossScene) &&
        motion.params.cross) {
      if (crossroad.crossRecognition(tracking)) {
        scene = Scene::CrossScene;

      } else
        scene = Scene::NormalScene;
    }

    //[11] 环岛识别与路径规划
    if ((scene == Scene::NormalScene || scene == Scene::RingScene) &&
        motion.params.ring) {
      if (ring.process(tracking, imgBinary, motion)) {
        scene = Scene::RingScene;
        // ring.
      } else
        scene = Scene::NormalScene;
    }
    //  ring.RoundaboutGetArc(tracking, 1, 20, 30, 160);
    //[12] 车辆控制中心拟合
    ctrlCenter.fitting(tracking);
    // 冲出赛道
    //  if (scene != Scene::RescueScene) {
    //    if (ctrlCenter.derailmentCheck(tracking)) //
    //    // 车辆冲出赛道检测（保护车辆）
    //    {
    //      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
    //      sleep(1);
    //      printf("-----> System Exit!!! <-----\n");
    //      exit(0); // 程序退出
    //    }
    //  }

    //[13] 车辆运动控制(速度+方向)
    motion.params.motion_start = 1;
    // cout<<"motion.params.motion_start"<<motion.params.motion_start<<endl;
    if (motion.params.motion_start) // 是否运动
    {
      // cout << "motion.params.motion_start" << motion.params.motion_start
      //      << endl;

      if ((scene == Scene::RescueScene && rescue.carStoping) || parking.park ||
          racing.carStoping) // 特殊区域停车
        motion.speed = 0;
      else if (scene == Scene::RescueScene && rescue.carExitting) // 倒车出库
      {
        motion.speed = -motion.params.speedDown;
        cout << "速度值为" << motion.speed << endl;

      } else if (scene == Scene::RescueScene) // 减速
        motion.speedCtrl(true, true, ctrlCenter);
      else if (scene == Scene::BridgeScene) // 坡道速度
        motion.speed = motion.params.speedBridge;
      else
        motion.speedCtrl(true, false, ctrlCenter); // 车速控制
      if (rescue.flagchur) {
        cout << "危险区右出库舵机打角定了" << endl;
        motion.poseCtrl(
            210); // 姿态控制（舵机）  此处为救援区出站固定打角 --使其偏差值为0
      } else if (rescue.flagchul) {
        cout << "危险区左出库舵机打角定了" << endl;
        motion.poseCtrl(
            110); // 姿态控制（舵机）  此处为救援区出站固定打角 --使其偏差值为0
      } else if (ring.center_sum_flag == Center_Sum_Start) {
        center_sum += ctrlCenter.controlCenter;
        center_sum_n++;
        motion.poseCtrl(
            ctrlCenter.controlCenter); // 姿态控制（舵机） 别忘记打角
      } else if (ring.center_sum_flag == Center_Sum_End) {
        cout << "固定舵机打角" << ctrlCenter.controlCenter << endl;
        ctrlCenter.controlCenter = center_sum / center_sum_n;
        motion.poseCtrl(
            ctrlCenter.controlCenter); // 出环平均的中心姿态控制（舵机）
      } else if (danger.flag_cone_first) {
        motion.poseCtrl(ctrlCenter.controlCenter + 15); // 姿态控制（舵机）

      } else
        motion.poseCtrl(ctrlCenter.controlCenter); // 姿态控制（舵机）
      if (ring.center_sum_flag == Center_Sum_Reset) {
        center_sum = 0;
        center_sum_n = 0;
      }
      uart->carControl(motion.speed, motion.servoPwm); // 串口通信控制车辆
    }
    Mat imgRes =
        Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
    //[14] 综合显示调试UI窗口
    if (motion.params.record_video) {
      // 帧率计算
      auto startTime = chrono::duration_cast<chrono::milliseconds>(
                           chrono::system_clock::now().time_since_epoch())
                           .count();
      printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
             1000.0 / (startTime - preTime));

      if (flag) {
        display.setNewWindow(1, "Binary", imgBinary);
      }

      switch (scene) {
      case Scene::NormalScene:
        break;
      case Scene::CrossScene:                  //[ 十字区 ]
        crossroad.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        break;
      case Scene::RingScene:              //[ 环岛 ]
        ring.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        break;
      case Scene::BridgeScene:              //[ 坡道区 ]
        bridge.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "S", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::DangerScene:    //[ 危险区 ]
        danger.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "X", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::RescueScene:              //[ 救援区 ]
        rescue.drawImage(tracking, imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "O", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::RacingScene:    //[ 追逐区 ]
        racing.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "R", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      case Scene::ParkingScene:    //[ 停车区 ]
        parking.drawImage(imgRes); // 图像绘制特殊赛道识别结果
        circle(imgCorrect, Point(COLSIMAGE / 2, ROWSIMAGE / 2), 40,
               Scalar(40, 120, 250), -1);
        putText(imgCorrect, "P", Point(COLSIMAGE / 2 - 25, ROWSIMAGE / 2 + 27),
                FONT_HERSHEY_PLAIN, 5, Scalar(255, 255, 255), 3);
        break;
      default: // 常规道路场景：无特殊路径规划
        break;
      }
      // circle(imgCorrect,
      //        Point(tracking.pointsEdgeLeft[ring.left_breakpoint].y,
      //              tracking.pointsEdgeLeft[ring.left_breakpoint].x),
      //        5, Scalar(255, 152, 0), -1); // 我们自己的拐点
      detection->drawBox(imgCorrect); // 图像绘制AI结果
      ctrlCenter.drawImage(tracking,
                           imgCorrect); // 图像绘制路径计算结果（控制中心）
      if (flag) {
        display.setNewWindow(3, getScene(scene),
                             imgRes); // 图像绘制特殊场景识别结果
        display.setNewWindow(4, "Ctrl", imgCorrect);
        if (motion.params.debug)
          display.show(); // 显示综合绘图
      }
      waitKey(10); // 等待显示
    }
    for (int i = 0; i < tracking.spurroad.size(); i++) {
      circle(imgCorrect, Point(tracking.spurroad[i].y, tracking.spurroad[i].x),
             5, Scalar(0, 0, 255), -1); // 红色点画拐点
    }
    detection->drawBox(imgCorrect); // 图像绘制AI结果
    int w1 = imgTrack.cols;
    int h1 = imgTrack.rows;
    int w2 = imgRes.cols;
    int h2 = imgRes.rows;
    int w3 = imgCorrect.cols;
    int h3 = imgCorrect.rows;
    int width = w1 + w2 + w3;
    int height = h1;
    // cout<<"长"<<display.imgShow.rows<<"列"<<display.imgShow.cols<<endl;

    video.write(display.imgShow);

    // cout<<width<<"height"<<height<<endl<<endl<<endl<<endl;
    // video.write(imgBinary);//图像1录像
    //     if(motion.params.record_video==1)
    // {
    //    Mat  resultImg = Mat(width, height, CV_8UC3, Scalar::all(0));
    //     Mat ROI_1 = resultImg(Rect(0, 0, w1, h1));
    //     Mat ROI_2 = resultImg(Rect(w1, 0, w2, h2));
    //     Mat ROI_3 = resultImg(Rect(w2, 0, w3, h3));
    //    imgTrack.copyTo(ROI_1);
    //    imgRes.copyTo(ROI_2);
    //    imgCorrect.copyTo(ROI_3);
    // video.write(resultImg);
    //   }
    //[15] 状态复位
    if (sceneLast != scene) {
      if (scene == Scene::NormalScene)
        uart->buzzerSound(uart->BUZZER_DING); // 祖传提示音效
      else
        uart->buzzerSound(uart->BUZZER_OK); // 祖传提示音效
    }
    sceneLast = scene; // 记录当前状态
    if (scene == Scene::DangerScene)
      scene = Scene::NormalScene;
    else if (scene == Scene::CrossScene)
      scene = Scene::NormalScene;

    // capture >> img;
    // cv::imshow("output", img);
    // outputVideo << img;
    //[16] 按键退出程序
    if (char(waitKey(1)) == 'q')
      break;
    if (uart->keypress) {
      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
      sleep(1);
      printf("-----> System Exit!!! <-----\n");
      exit(0); // 程序退出
    }

    // endTime = clock();
    // cout << "the run time is " << (double)(endTime - startTime) /
    // CLOCKS_PER_SEC
    //      << "s" << endl;
  }

  uart->close(); // 串口通信关闭
  capture.release();
  return 0;
}
void sigint_handler(int sig) {

  if (sig == SIGINT) {
    // Ctrl+C 被按下时执行的代码
    std::cout << "Ctrl+C 被按下！" << std::endl;
    app_stopped = true;
    // uart->carpid(500, 1000, 0,
    //              0); // 调刹车pid，参数分别为p，i，d，是否存入flash
    uart->carControl(0, 750);

    exit(0);
  }
  //   if(start==0&&sig==SIGINT)
  //   {
  // uart->carpid(300, 750, 0, 0);
  // cout<<"fache"<<endl;
  // start++;

  //   }
}