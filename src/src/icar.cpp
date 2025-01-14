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
#include <math.h>
#include <opencv2/highgui.hpp> //OpenCV终端部署
#include <opencv2/opencv.hpp>  //OpenCV终端部署
#include <signal.h>
#include <unistd.h>
#include <chrono>//记录耗时
#include "CAM_cpp/global.hpp"
///多线程相关
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

#define toStr(name) (#name)

using namespace std;
using namespace cv;

uint8_t imo3[ROWSIMAGE][COLSIMAGE];
uint8_t imo4[ROWSIMAGE][COLSIMAGE];
cv::Mat imo3_img(ROWSIMAGE, COLSIMAGE, CV_8UC3);
cv::Mat imo4_img(ROWSIMAGE, COLSIMAGE, CV_8UC3);
struct vofa_struct vofa;
void sigint_handler(int sig);
extern uint8_t Grayscale[ROWSIMAGE][COLSIMAGE];
extern float Gyro_Z;
int flag = 1;
int start = 0;                        // 发车计数器
int center_sum = 0, center_sum_n = 0; // 中心总值 ,计数
bool Is_AI_detection = 1;             // 是否开启AI
int distance_start = 0;
double AI_distance_start = 0,
       AI_distance_end = 0; // AI标志与距离积分的开始和结束
int STEER_MIN ;        // 舵机限幅
int STEER_MAX ;
// 定义二维数组
uint8_t my_Grayscale[ROWSIMAGE][COLSIMAGE];
// struct lineinfo_s lineinfo[120];
// 将图像矩阵转换为二维数组的函数

//多线程相关
const int MAX_PRELOAD_QUEUE_SIZE = 10; // 预读取队列的最大大小
const int MAX_DISPLAY_QUEUE_SIZE = 5;  // 显示队列的最大大小

std::queue<cv::Mat> preload_queue; // 存储预读取的帧
std::queue<cv::Mat> display_queue; // 存储待显示的帧
std::mutex preload_mtx;            // 保护预读取队列的互斥锁
std::mutex display_mtx;            // 保护显示队列的互斥锁
std::condition_variable preload_cv; // 用于预读取线程和主线程通信的条件变量
std::condition_variable display_cv; // 用于主线程和显示线程通信的条件变量
// bool stop_threads = false;         // 标志位，用于停止线程
std::atomic<bool> stop_threads(false); // 控制线程退出的标志位
std::thread thread_cam,thread_show;   //线程对象
bool Is_showimg;//显示原图
//AI是否开启标志位
enum AI_Distance_Postion {
  AI_Distance_None = 0,
  AI_None_Start,
  AI_None_End,
  AI_Rescue_Start,
  AI_Rescue_End,
  AI_Danger_Start,
  AI_Danger_End,
  AI_Bridge_Start,
  AI_Bridge_End
} AI_distance_postion;


shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
void CAM_CPU_while(void);
void MatTo2DArray(const Mat &img, uchar array[ROWSIMAGE][COLSIMAGE]);
void draw_imo_color(uint8_t myimo[ROWSIMAGE][COLSIMAGE], Mat mat);
void show_params(Mat img, float *data);//显示参数
float *set_show_params_mode(int num);//选择显示模式

void preload_thread(cv::VideoCapture& capture);//预读取线程负责从摄像头或视频源中读取帧，并将帧放入预读取队列
void display_thread();
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
//  cv::VideoCapture capture("v4l2src ! videoconvert ! videoscale ! video/x-raw,width=640,height=480 ! appsink", cv::CAP_GSTREAMER);
  // 目标检测类(AI模型文件)
  shared_ptr<Detection> detection = make_shared<Detection>(motion.params.model);
  // detection->score = motion.params.score; // AI检测置信度

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

  // VideoWriter video("ouput.avi", cv::VideoWriter::fourcc('M', 'J', 'P',
  // 'G'),
  //                   30, Size(4 * COLSIMAGE, ROWSIMAGE), true);
    // 设置帧率
  //   double desired_fps = 350; // 设置帧率为 120 fps 


   capture.open(0, cv::CAP_V4L2);//打开摄像头
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  }

  capture.set(CAP_PROP_FRAME_WIDTH,320);// 设置图像分辨率//有些摄像头需要
  capture.set(CAP_PROP_FRAME_HEIGHT,240); // 设置图像分辨率 
  // capture.set(cv::CAP_PROP_FPS, 120);//设置帧率 
 // 启动预读取线程
    std::thread thread_cam(preload_thread, std::ref(capture));
// 启动显示线程
    std::thread thread_show(display_thread);
  AI_distance_postion = AI_Distance_None;
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
 
  uart->carpid(300, 750, 0, 0); // 调pid，参数分别为p，i，d，是否存入flash
  // clock_t startTime, endTime;     // 统计程序时间
  // signal(SIGINT, sigint_handler); // 中断，结束的时候

  int s1 = 1;
  string s2 = ".jpg";
  float mpu6050_now;
  float mpu6050_later;
  float distance_now;
  int ai_check = 0;
  int ai_middle_quanzhong;
  distance_start = uart->get_distance();
  int picture_num = 0;

  /////////////////祖传算法需要初始化的地方
  init_setpara();
  act_perst_init();
  PID_init();
  set_setpara(motion.params.turn_PIDkp, motion.params.turn_PIDkd,
              motion.params.gyroturn_PIDkp, motion.params.gyroturn_PIDki,
              motion.params.gyroturn_PIDkd, motion.params.loop_turn_PIDkp,
              motion.params.loop_turn_PIDkd, motion.params.big_loop_PIDkp,
              motion.params.big_loop_PIDkd, motion.params.camwf,
              motion.params.camwl, motion.params.camwr,
              motion.params.speed_max, /////////////速度决策,

              motion.params.speed_add, ///////////
              motion.params.speed_min, motion.params.loop_target_speed,
              motion.params.loop_out_distance,
              motion.params.STEER_MID,motion.params.STEER_MIN,motion.params.STEER_MAX,motion.params.Is_showimg); // 写在init_setpara（）后面
  // 改config文件，改set_setpara函数
  car_begin(); // 初始化车启动的标志位
  //////////////////视频输出
  cv::VideoWriter video("output.avi",
                        cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                        Size(1128, 360), true);

  // 检查视频写入器是否成功创建
  if (!video.isOpened()) {
    std::cerr << "Could not open the output video file for writing"
              << std::endl;
    return -1;
  }
  ///
  int temp = 456;
  string str = toStr(temp);
  cout << str.c_str() << endl; // temp



// capture.set(cv::CAP_PROP_BUFFERSIZE, 3);
  cv::Mat img;//原图像
  ///////////主循环
  while (!stop_threads) {
        signal(SIGINT, sigint_handler); // 中断，结束的时候
        // cout<<"camwf"<<motion.params.camwf<<endl;
      auto start = std::chrono::high_resolution_clock::now(); // 记录开始时间
// 从预读取队列中取出帧
        {
            std::unique_lock<std::mutex> lock(preload_mtx);
            preload_cv.wait(lock, [] { return !preload_queue.empty() || stop_threads; });
            if (stop_threads && preload_queue.empty()) {
                break;
            }
            img = preload_queue.front();
            preload_queue.pop();
        }
// 将处理后的帧放入显示队列
        {
            std::lock_guard<std::mutex> lock(display_mtx);
            display_queue.push(img.clone());
        }
        display_cv.notify_one(); // 通知显示线程

 auto end1 = std::chrono::high_resolution_clock::now(); // 记录结束时间
    std::chrono::duration<double> duration1 = end1 - start;
    std::cout << "reading " << duration1.count() << " seconds" << std::endl;
   


    for (int i = 0; i < ROWSIMAGE; ++i) {//耗时大概16us
      std::fill(imo3[i], imo3[i] + COLSIMAGE, 0);//全给灰色？
    }
    for (int i = 0; i < ROWSIMAGE; ++i) {
      std::fill(imo4[i], imo4[i] + COLSIMAGE, 0);
    }
    //  ring.RoundaboutGetArc(tracking, 1, 20, 30, 160);

    Size dsize_first = Size(188, 120);
    cv::resize(img, img, dsize_first, 0, 0, INTER_AREA);

    //最多16us

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
    Gyro_Z = uart->get_gyro_z();         // 角速度

    // cout << "现在的距离积分" << distance_now - distance_start << endl;
    // cout << mpu6050_now << endl; // 输出mpu
    // cout << "角速度" << Gyro_Z << endl;
    angal_integeral(mpu6050_now);    // 把现在角度积分不断传入
    distant_integeral(distance_now); //

    // 之前的代码可以去除掉了？----积分现在没用到？
    ring.setmpu6050(mpu6050_now);
    ring.setdistance(distance_now);
    rescue.setdistancere(distance_now);
    rescue.setmpu6050(mpu6050_now);
    crossroad.setmpu6050(mpu6050_now);
    danger.setdistance(distance_now);
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

//  auto end2 = std::chrono::high_resolution_clock::now(); // 记录结束时间
//     std::chrono::duration<double> duration2 = end2 - end1;
//     std::cout << "readimg " << duration2.count() << " seconds" << std::endl;

///从while到这28ms

    //  if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
    //    savePicture(img);
    //  if (waitKey(1) == 27) { // 如果用户按下 ESC 键，退出循环

    //   // s1++;
    //   // imwrite("../res/calibration/temp/" + to_string(s1) + s2, img);
    //   // cout << "../res/calibration/temp/" + to_string(s1) + s2 <<
    //   endl; uart->carpid(300, 750, 0, 0); //
    //   调pid，参数分别为p，i，d，是否存入flash
    //   // cout << "fache" << endl << endl << endl << endl << endl;

    //   // 按键发车
    // }
    //[02] 图像预处理

    Mat imgCorrect = img; // 图像矫正（已停止
    Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化

    // MatTo2DArray(imgBinary, Grayscale);// 调用函数将图像转换为二维数组

    // char buffer[50];
    // sprintf(buffer, "%d.jpg", picture_num);
    // picture_num++;//截图
    // std::cout << "图像" << buffer << std::endl; // 输出: The answer is:
    //  imwrite(buffer, imgBinary);
    // Mat element = getStructuringElement(
    //     MORPH_RECT, Size(4, 4)); // 小于8*8方块的白色噪点都会被腐蚀
    // erode(imgBinary, imgBinary, element)
    // 将矩阵转换为二维数组,耗时35us
    for (int i = 0; i < ROWSIMAGE; ++i) {
      for (int j = 0; j < COLSIMAGE; ++j) {
        Grayscale[i][j] = imgBinary.at<uchar>(i, j);//是白色，
// cout<< static_cast<int>(Grayscale[i][j]);
      }
// cout<<endl;
    }
///从while到这30ms

    /*///核心控制部分////*/
    CAM_CPU_while();
    cout << "圆环标志位" << watch.InLoop << endl;
    // cout << "目标速度" << mycar.target_speed << endl;
    cout << "watch.InLoopAngle2  " << watch.InLoopAngle2 << endl;


 ///从while到这30ms   
    if (motion.params.debug) // 开启视频
    {
    // imshow("original",img);//显示原图像

    // draw_imo_color(imo3, imo3_img); // 扫弦图绿色是右边，蓝色是左边

    // draw_imo_color(imo4, imo4_img); // 逆透视
    // cv::Mat colorImage = cv::Mat::zeros(120, 188, CV_8UC3);
    // cv::cvtColor(imgBinary, colorImage, cv::COLOR_GRAY2BGR);
    // for (int i = 0; i < colorImage.rows; ++i) {
    //   for (int j = 0; j < colorImage.cols; ++j) {
    //     cv::Vec3b overlayPixel = imo3_img.at<cv::Vec3b>(i, j);
    //     if (overlayPixel != cv::Vec3b(0, 0, 0)) {
    //       colorImage.at<cv::Vec3b>(i, j) = overlayPixel;
    //     }
    //   }
    // }

      draw_imo_color(imo3, imo3_img); // 扫弦图绿色是右边，蓝色是左边
      draw_imo_color(imo4, imo4_img); // 逆透视
      cv::Mat colorImage = cv::Mat::zeros(120, 188, CV_8UC3);
      cv::cvtColor(imgBinary, colorImage, cv::COLOR_GRAY2BGR);
      for (int i = 0; i < colorImage.rows; ++i) {
        for (int j = 0; j < colorImage.cols; ++j) {
          cv::Vec3b overlayPixel = imo3_img.at<cv::Vec3b>(i, j);
          if (overlayPixel != cv::Vec3b(0, 0, 0)) {
            colorImage.at<cv::Vec3b>(i, j) = overlayPixel;
          }
        }
      }
      double scale = 3;

      // 缩放图像
      Mat resizedImage1;
      Mat resizedImage2;
      Mat resizedImage3;
      Mat resizedImage4;
      //  resize(imgBinary, resizedImage1, Size(), scale, scale);

      resize(colorImage, resizedImage2, Size(), scale, scale); // 补线图
      resize(imo4_img, resizedImage3, Size(), scale, scale);   // 逆透视图

      cv::Mat combinedFrame;
      cv::hconcat(resizedImage2, resizedImage3, combinedFrame);
      cout << "mode" << motion.params.show_params_mode << endl;
      float *params_mode = set_show_params_mode(motion.params.show_params_mode);
      show_params(combinedFrame, params_mode);

      imshow("789", combinedFrame);
      if (motion.params.record_video) {
        video.write(combinedFrame);
      }
    
      waitKey(1); // 等待显示，不能删！！！
      // resize(imgBinary, resizedImage4, Size(), scale, scale);
      // 创建一个4x4的矩阵来显示四幅图像
      // Mat displayMatrix(2 * resizedImage1.rows, 2 * resizedImage1.cols,
      //                   CV_8UC3);

      // // 将四幅图像放入矩阵中
      // Mat roi;

      // // 左上角
      // roi = displayMatrix(Rect(0, 0, resizedImage1.cols,
      // resizedImage1.rows)); resizedImage1.copyTo(roi);

      // // 右上角
      // roi = displayMatrix(
      //     Rect(resizedImage1.cols, 0, resizedImage1.cols,
      //     resizedImage1.rows));
      // resizedImage2.copyTo(roi);

      // // 左下角
      // roi = displayMatrix(
      //     Rect(0, resizedImage1.rows, resizedImage1.cols,
      //     resizedImage1.rows));
      // resizedImage3.copyTo(roi);

      // // 右下角
      // roi = displayMatrix(Rect(resizedImage1.cols, resizedImage1.rows,
      //                          resizedImage1.cols, resizedImage1.rows));
      // resizedImage4.copyTo(roi);
      // imshow("四幅图像1", displayMatrix);

      // 显示结果

      // if (motion.params.record_video == 1) {
      //   // 检查VideoWriter是否成功创建
      //   if (!video.isOpened()) {
      //     cout << "无法创建视频文件" << endl;
      //     return -1;
      //   }

      //   // 录制视频

      //   video.write(roi);
    }

 ///从while到这35ms 
    // 打印二维数组
    // std::cout << "二维数组内容：" << std::endl;
    // for (int i = 0; i < ROWSIMAGE; ++i) {
    //   for (int j = 0; j < COLSIMAGE; ++j) {
    //     std::cout << static_cast<int>(imo4[i][j]) << " ";
    //   }
    //   std::cout << std::endl; // 换行
    // }

    auto startTime = chrono::duration_cast<chrono::milliseconds>(
                         chrono::system_clock::now().time_since_epoch())
                         .count(); // 原来这里有个q
    printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
           1000.0 / (startTime - preTime));

    mycar.RUNTIME += startTime - preTime; // 运行时间
    //  ring.RoundaboutGetArc(tracking, 1, 20, 30, 160);
    //[12] 车辆控制中心拟合

    // cout<<"角度"<<motion.angle<<endl;
    // 冲出赛道
    //  if (scene != Scene::RescueScene) {
    //    if (ctrlCenter.derailmentCheck(tracking)) //
    //    // 车辆冲出赛道检测（保护车辆）
    //    {
    //      uart->carControl(0, PWMSERVOMID); // 控制车辆停止运动
    //      sleep(1);
    //      printf("-----> System Exit!!! <-----\n");
    //      exit(0); // 程序退
    //    }
    //  }

    //[13] 车辆运动控制(速度+方向)
    motion.params.motion_start = 1;
    // cout<<"motion.params.motion_start"<<motion.params.motion_start<<endl;
    if (motion.params.motion_start) // 是否运动
    {

      if (mycar.uart_servo < STEER_MIN)
        mycar.uart_servo = STEER_MIN;
      if (mycar.uart_servo > STEER_MAX)
        mycar.uart_servo = STEER_MAX;
      // mycar.uart_speed = 1;
      // mycar.uart_servo=STEER_MID;
      // cout << "目标速度" << mycar.uart_speed << "舵机PWM" << mycar.uart_servo
      //      << endl;

      uart->carControl(
          mycar.uart_speed,
          mycar.uart_servo); // 串口通信控制车辆---传给下位机进行控制
    }

 ///从while到这35ms 
    // Mat imgRes =
    //     Mat::zeros(Size(COLSIMAGE, ROWSIMAGE), CV_8UC3); // 创建全黑图像
    //[14] 综合显示调试UI窗口
    // if (motion.params.record_video) {
    //   // 帧率计算
    //   // auto startTime = chrono::duration_cast<chrono::milliseconds>(
    //   // chrono::system_clock::now().time_since_epoch())
    //   //                      .count();
    //   // printf(">> FrameTime: %ldms | %.2ffps \n", startTime - preTime,
    //   //        1000.0 / (startTime - preTime));

    //   // // circle(imgCorrect,
    //   // //        Point(tracking.pointsEdgeLeft[ring.left_breakpoint].y,
    //   // //              tracking.pointsEdgeLeft[ring.left_breakpoint].x),
    //   // //        5, Scalar(255, 152, 0), -1); // 我们自己的拐点
    //   // detection->drawBox(imgCorrect); // 图像绘制AI结果
    //   // ctrlCenter.drawImage(tracking,
    //   //                      imgCorrect); //
    //   图像绘制路径计算结果（控制中心） waitKey(1); //
    //   等待显示，不能删！！！
    // } // 不能去

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
  }
  // 等待线程结束
    if (thread_cam.joinable()) {
        thread_cam.join();
    }
    if (thread_show.joinable()) {
        thread_show.join();
    }
    // preload_cv.notify_all();// 唤醒预读取线程
    // display_cv.notify_all(); // 唤醒显示线程
   
  uart->close(); // 串口通信关闭
  capture.release();//关闭摄像头
  cv::destroyAllWindows(); // 关闭所有窗口
  return 0;
}
//预读取线程负责从摄像头或视频源中读取帧，并将帧放入预读取队列
void preload_thread(cv::VideoCapture& capture) {
    cv::Mat img;
    while (!stop_threads) {
        if (!capture.read(img)) {
            continue; // 如果读取失败，跳过当前帧
        }

        // 将帧放入预读取队列
        {
            std::lock_guard<std::mutex> lock(preload_mtx);
            if (preload_queue.size() >= MAX_PRELOAD_QUEUE_SIZE) {
                preload_queue.pop(); // 丢弃旧的帧
            }
            preload_queue.push(img.clone());
        }
        preload_cv.notify_one(); // 通知主线程
    }
}

// 显示线程函数
void display_thread() {
    int display_counter = 0;
    int display_interval = 10; // 每 2 帧显示一次

    while (!stop_threads&&1) {
        cv::Mat img;
        {
            std::unique_lock<std::mutex> lock(display_mtx);
            // 等待队列中有帧可显示
            display_cv.wait(lock, [] { return !display_queue.empty() || stop_threads; });
            if (stop_threads && display_queue.empty()) {
                break; // 如果线程需要退出且队列为空，则退出
            }
            img = display_queue.front(); // 取出帧
            display_queue.pop();         // 从队列中移除
        }

        // 降低显示帧率
        if (display_counter % display_interval == 0) {
            // cv::imshow("Display", img);
            cv::waitKey(1); // 等待 1ms
        }
        display_counter++;
    }
}



//// Ctrl+C 被按下时执行的代码
void sigint_handler(int sig) {

  if (sig == SIGINT) {
    
    std::cout << "Ctrl+C 被按下！" << std::endl;
    // uart->carpid(500, 1000, 0,
    //              0); // 调刹车pid，参数分别为p，i，d，是否存入flash
    uart->carControl(0, 4800);

      // 设置停止标志并等待线程结束
    stop_threads = true; 
     // 等待线程结束
    if (thread_cam.joinable()) {
        thread_cam.join();
    }
    if (thread_show.joinable()) {
        thread_show.join();
    }
    exit(0);
  }
  //   if(start==0&&sig==SIGINT)
  //   {
  // uart->carpid(300, 750, 0, 0);
  // cout<<"fache"<<endl;
  // start++;

  //   }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void CAM_CPU_while(void) {
  scan_line();
  Element_recognition(); // 元素识别
  linefix();
  original_err_calculation();
  dir_control();   // 舵机控制
  motor_control(); // 电机控制
}

// 将图像矩阵转换为二维数组的函数
void MatTo2DArray(const Mat &img, uchar array[ROWSIMAGE][COLSIMAGE]) {
  // 检查输入图像是否为空
  if (img.empty()) {
    cerr << "Error: Input image is empty." << endl;
    return;
  }

  // 检查图像的类型是否为单通道灰度图像
  if (img.channels() != 1) {
    cerr << "Error: Input image is not a single-channel my_Grayscale image."
         << endl;
    return;
  }

  // 检查图像的尺寸是否匹配
  if (img.rows != ROWSIMAGE || img.cols != COLSIMAGE) {
    cerr << "Error: Input image dimensions do not match the provided row and "
            "column values."
         << endl;
    return;
  }

  // 将图像数据复制到二维数组中
  for (int i = 0; i < ROWSIMAGE; ++i) {
    for (int j = 0; j < COLSIMAGE; ++j) {
      array[i][j] = img.at<uchar>(i, j);
    }
  }
}
void draw_imo_color(uint8_t myimo[ROWSIMAGE][COLSIMAGE], Mat mat) {
  for (int i = 0; i < ROWSIMAGE; ++i) {
    for (int j = 0; j < COLSIMAGE; ++j) {
      uint8_t value = myimo[i][j];

      // 根据值设置颜色
      cv::Vec3b color;
      switch (value) {
      case 1:

        color = cv::Vec3b(0, 0, 255); // 红色
        break;
      case 2:
        color = cv::Vec3b(0, 255, 0); // 绿色
        break;
      case 3:
        color = cv::Vec3b(0, 255, 255); // 黄色
        break;
      case 4:
        color = cv::Vec3b(255, 0, 0); // 蓝色
        break;
      case 5:
        color = cv::Vec3b(255, 0, 255); // 洋红色
        break;
      case 6:
        color = cv::Vec3b(0, 255, 255); // 青色
        break;
      case 7:
        color = cv::Vec3b(128, 128, 128); // 灰色
        break;
      case 8:
        color = cv::Vec3b(128, 0, 0); // 深红色
        break;
      case 9:
        color = cv::Vec3b(0, 128, 0); // 深绿色
        break;
      default:
        color = cv::Vec3b(0, 0, 0); // 黑色
        break;
      }

      // 设置像素颜色
      mat.at<cv::Vec3b>(119 - i, j) = color;
    }
  }
}
void show_params(Mat img, float *data) {
  vofa.turn[0] = mycar.steer_pwm;
  vofa.turn[1] = mycar.original_err;
  vofa.turn[2] = Element;
  vofa.turn[3] = watch.OutLoopAngle2;
  vofa.turn[4] = CAM_Turn.PID_output;
  vofa.turn[5] = mycar.RUNTIME;
  // vofa.turn[6]=abs(mycar.original_err);
  vofa.turn[6] = watch.InLoop;
  vofa.turn[7] = mycar.present_speed;

  vofa.loop[0] = watch.InLoop;
  vofa.loop[1] = watch.InLoopAngleL;
  vofa.loop[2] = distance_integral.integeral_data;
  vofa.loop[3] = watch.InLoopAngle2;
  vofa.loop[4] = watch.OutLoop;
  vofa.loop[5] = Element;
  vofa.loop[6] = watch.OutLoopAngle2;
  vofa.loop[7] = mycar.RUNTIME;

  vofa.speed[0] = mycar.left_speed;
  vofa.speed[1] = mycar.right_speed;
  vofa.speed[2] = mycar.steer_pwm;
  vofa.speed[3] = mycar.left_pwm_set;
  vofa.speed[4] = mycar.right_pwm_set;
  vofa.speed[5] = (float)(mycar.right_speed - mycar.present_speed) /
                  (float)mycar.present_speed;
  vofa.speed[6] = mycar.RUNTIME;
  vofa.speed[7] = mycar.target_speed;
  vofa.speed[8] = Element;

  vofa.element_rem[0] = Element_rem.Element_count;
  vofa.element_rem[1] = Element_rem.Element_data[Element_rem.Element_count];
  vofa.element_rem[2] = watch.InLoop;
  vofa.element_rem[3] = watch.cross_flag;
  vofa.element_rem[4] = Element;
  vofa.element_rem[5] = watch.slope_flag;
  // vofa.element_rem[6] = imu.pitch;
  vofa.element_rem[7] = watch.out_garage_flag;

  vofa.obstacle[0] = watch.black_obstacle_flag;
  vofa.obstacle[1] = watch.black_obstacle_line;
  vofa.obstacle[2] = watch.left_obstacle_x;
  vofa.obstacle[3] = watch.ZebraInLine;
  vofa.obstacle[4] = Element;
  vofa.obstacle[5] = angle_integral.integeral_data;
  vofa.obstacle[6] = mycar.RUNTIME;

  vofa.broken[0] = watch.broken_circuit_flag;
  vofa.broken[1] = Element_rem.Element_count;
  // vofa.broken[2] = imu.pitch;//惯导
  // vofa.broken[3] = imu.gyroy;
  vofa.broken[4] = Element;
  vofa.broken[5] = mycar.RUNTIME;

  vofa.cross[0] = Element;
  vofa.cross[1] = watch.track_count_far;
  vofa.cross[2] = watch.cross_flag;
  vofa.cross[3] = watch.cross_LD_angle;
  vofa.cross[4] = watch.cross_RD_angle;
  vofa.cross[5] = watch.cross_AngleL;
  vofa.cross[6] = watch.cross_AngleR;
  vofa.cross[7] = watch.cross_AngleR_x;

  vofa.speed_differ[0] = mycar.steer_pwm;
  vofa.speed_differ[1] = mycar.RUNTIME;
  vofa.speed_differ[2] = mycar.right_pwm_set;
  vofa.speed_differ[3] = mycar.target_left_speed;
  vofa.speed_differ[4] = mycar.target_right_speed;
  vofa.speed_differ[5] = mycar.left_speed;
  vofa.speed_differ[6] = mycar.right_speed;
  //    vofa.speed_differ[7]=mycar.speed_differ;
  vofa.speed_differ[7] =
      mycar.target_speed *
      (1 - 2 * mycar.speed_left_differ / (1 + mycar.speed_left_differ));
  vofa.speed_differ[8] =
      mycar.target_speed *
      (1 + 2 * mycar.speed_right_differ / (1 - mycar.speed_right_differ));

  vofa.fuzzy_pid[0] = mycar.nonlinear_trackpos;
  vofa.fuzzy_pid[1] = mycar.track_diff;
  vofa.fuzzy_pid[2] = (float)CAM_FUZZY_PID.kp;
  vofa.fuzzy_pid[3] = (float)CAM_FUZZY_PID.kd;
  //    vofa.fuzzy_pid[3]=watch.track_count_far;
  //    vofa.fuzzy_pid[0]=mycar.target_left_speed;
  //    vofa.fuzzy_pid[1]=mycar.target_right_speed;
  //    vofa.fuzzy_pid[2]=mycar.left_speed;
  //    vofa.fuzzy_pid[3]=mycar.right_speed;
  //    vofa.fuzzy_pid[4]=mycar.left_pwm_set;
  //    vofa.fuzzy_pid[5]=mycar.right_pwm_set;
  vofa.fuzzy_pid[4] = watch.track_count;
  vofa.fuzzy_pid[5] = mycar.present_speed;
  //    vofa.fuzzy_pid[5]=mycar.fan_buchang;
  vofa.fuzzy_pid[6] = mycar.steer_pwm;
  //    vofa.fuzzy_pid[6]=mycar.corner_flag;
  //    vofa.fuzzy_pid[6]=mycar.distance_count;
  //    vofa.fuzzy_pid[7]=(imu.gyroz/(mycar.present_speed+0.1));
  //    vofa.fuzzy_pid[7]=Speed_left.PID_out_I;
  vofa.fuzzy_pid[7] = Element;
  vofa.fuzzy_pid[8] = mycar.RUNTIME;

  vofa.fuzzy_speed[0] = mycar.target_left_speed;
  vofa.fuzzy_speed[1] = mycar.target_right_speed;
  vofa.fuzzy_speed[2] = mycar.left_speed;
  vofa.fuzzy_speed[3] = mycar.right_speed;
  vofa.fuzzy_speed[4] = mycar.steer_pwm;
  vofa.fuzzy_speed[5] = watch.track_count;
  vofa.fuzzy_speed[6] = mycar.left_pwm_set;
  vofa.fuzzy_speed[7] = Element;
  vofa.fuzzy_speed[8] = mycar.RUNTIME;

  vofa.steer_cal[0] = mycar.nonlinear_trackpos;
  vofa.steer_cal[1] = mycar.Steer_PWM_Kp;
  vofa.steer_cal[2] = mycar.raw_track_diff;
  vofa.steer_cal[3] = mycar.track_diff;
  vofa.steer_cal[4] = mycar.original_err;
  vofa.steer_cal[5] = mycar.steer_pwm;
  vofa.steer_cal[6] = mycar.d_original_err;
  vofa.steer_cal[7] = Element;

  vofa.zebra[0] = mycar.target_speed;
  vofa.zebra[1] = watch.zebra_flag;
  vofa.zebra[2] = Element;
  vofa.zebra[3] = watch.stop_count;
  vofa.zebra[4] = mycar.RUNTIME;
  vofa.zebra[5] = mycar.present_speed;

  // vofa.current[0] = adc.current_l;
  vofa.current[1] = mycar.target_current_l;
  vofa.current[2] = mycar.left_speed;
  vofa.current[3] = mycar.right_speed;
  vofa.current[4] = mycar.left_pwm_set;
  vofa.current[5] = Element;
  vofa.current[6] = mycar.target_left_speed;
  vofa.current[7] = mycar.target_right_speed;
  vofa.current[8] = mycar.RUNTIME;
  std::string text1 = "Number 1: " + std::to_string(data[0]);
  std::string text2 = "Number 2: " + std::to_string(data[1]);
  std::string text3 = "Number 3: " + std::to_string(data[2]);
  std::string text4 = "Number 4: " + std::to_string(data[3]);
  std::string text5 = "Number 5: " + std::to_string(data[4]);
  std::string text6 = "Number 6: " + std::to_string(data[5]);
  std::string text7 = "Number 7: " + std::to_string(data[6]);
  std::string text8 = "Number 8: " + std::to_string(data[7]);
  std::string text9 = "Number 9: " + std::to_string(data[8]);
  cv::putText(img, text1, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text2, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text3, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text4, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text5, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text6, cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text7, cv::Point(10, 140), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text8, cv::Point(10, 160), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text9, cv::Point(10, 180), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(255, 0, 255), 1);
}
float *set_show_params_mode(int num) {
  switch (num) {
  case 1:
    return vofa.turn;
    break;
  case 2:
    return vofa.loop;
    break;
  case 3:
    return vofa.speed;
    break;
  case 4:
    return vofa.element_rem;
    break;
  case 5:
    return vofa.obstacle;
    break;
  case 6:
    return vofa.broken;
    break;
  case 7:
    return vofa.cross;
    break;
  case 8:
    return vofa.speed_differ;
    break;
  case 9:
    return vofa.fuzzy_pid;
    break;
  case 10:
    return vofa.steer_cal;
    break;
  case 11:
    return vofa.zebra;
    break;
  case 12:
    return vofa.current;
    break;
  }
}