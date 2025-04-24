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
#include "detection/bridge.cpp"      //AI检测：坡道区
#include "detection/obstacle.cpp"    //AI检测：障碍区
#include "detection/catering.cpp"    //AI检测：餐饮区
#include "detection/layby.cpp"       //AI检测：临时停车区
#include "detection/parking.cpp"     //AI检测：充电停车场
#include "detection/crosswalk.cpp"   //AI检测：停车区
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
#include <future>
////优化相关
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp> // 包含错误码定义
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
// 是否开启AI
bool Is_AI_detection = 1;             
int distance_start = 0;
int STEER_MIN ;        // 舵机限幅
int STEER_MAX ;
// 定义二维数组
uint8_t my_Grayscale[ROWSIMAGE][COLSIMAGE];
extern Element_range Element;     // 元素识别
// struct lineinfo_s lineinfo[120];
// 将图像矩阵转换为二维数组的函数
//多线程相关
const int MAX_PRELOAD_QUEUE_SIZE = 5; // 预读取队列的最大大小
const int MAX_DISPLAY_QUEUE_SIZE = 10;// 显示队列的最大大小

queue<cv::Mat> preload_queue; // 存储预读取的帧
queue<cv::Mat> display_queue; // 存储待显示的帧
queue<cv::Mat> inference_queue;// 存储推理的帧
mutex preload_mtx;            // 保护预读取队列的互斥锁
mutex display_mtx;            // 保护显示队列的互斥锁
mutex inference_mutex;
condition_variable preload_cv; // 用于预读取线程和主线程通信的条件变量
condition_variable display_cv; // 用于主线程和显示线程通信的条件变量
condition_variable inference_cv;
// bool stop_threads = false;         // 标志位，用于停止线程
atomic<bool> stop_threads(false); // 控制线程退出的标志位
thread thread_cam,thread_show;   //线程对象
thread inference_thread;
mutex inference_mtx;
cv::Mat inference_img;
bool inference_ready = false;
bool inference_done = false;
bool stop_inference_thread ;    //是否关闭推理线程
bool Is_showimg;//显示原图
float mpu6050_now;    //现在的mpu6050Z轴角度
float mpu6050_later;
float distance_now;   //距离积分
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
std::atomic<int> totalFrames(0);
std::atomic<int> highFPSCount(0);
std::atomic<int> lowFPSCount(0);
shared_ptr<Uart> uart = make_shared<Uart>("/dev/ttyUSB0"); // 初始化串口驱动
void CAM_CPU_while(void);

void draw_imo_color(uint8_t myimo[ROWSIMAGE][COLSIMAGE], Mat mat);
void show_params(Mat img, int *data);//显示参数
int *set_show_params_mode(int num);//选择显示模式

void preload_thread(cv::VideoCapture& capture);//预读取线程负责从摄像头或视频源中读取帧，并将帧放入预读取队列
void display_thread();
void inference_thread_func();
void init_AI_param();   //所有需要AI识别的参数初始化
// 将BGR转HSV范围定义（实际应用可能需要调整）
Scalar lower_red1(0, 100, 70);    // 红色下限1（0-10度）
Scalar upper_red1(10, 255, 255);
Scalar lower_red2(170, 100, 70);  // 红色下限2（170-180度）
Scalar upper_red2(180, 255, 255);
Scalar lower_yellow(25, 80, 70);  // 黄色范围
Scalar upper_yellow(35, 255, 255);
Scalar lower_purple(145, 80, 70); // 紫色范围
Scalar upper_purple(165, 255, 255);
Mat mask_red, mask_yellow, mask_purple;
void HSV_Process(Mat& frame);
void processContours(Mat& frame, const vector<vector<Point>>& contours, Scalar color, const string& label) ;
Motion motion;            // 运动控制类

Bridge bridge;  
Catering catering;
Eobstacle aobstacle;        // 障碍区检测类
Layby layby;              // 临时停车区检测类
Parking parking;          // 充电停车场检测类
StopArea stopArea;        // 停车区识别与路径规划类
int main(int argc, char const *argv[]) {
  Preprocess preprocess;    // 图像预处理类
  Tracking tracking;        // 赛道识别类
  Crossroad crossroad;      // 十字道路识别类
  Ring ring;                // 环岛识别类
  Display display(4);       // 初始化UI显示窗口
  shared_ptr<Detection> detection;
 

  //VideoCapture capture;     // Opencv相机类
 
  // VideoCapture capture(0);  // Opencv相机类
  cv::VideoCapture capture(0, cv::CAP_V4L2);  // 强制使用 V4L2 后端
  detection = make_shared<Detection>(motion.params.model);
  detection->score = motion.params.score; // AI检测置信度
  stop_inference_thread=!motion.params.Is_AI_detection;

  // USB转串口初始化： /dev/ttyUSB0
  int ret = uart->open();
  if (ret != 0) {
    printf("[Error] Uart Open failed!\n");
    return -1;
  }
  uart->startReceive(); // 启动数据接收子线程
  capture.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cv::VideoWriter video;
  video.open("Output.avi",
    cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30,
    Size(188, 120), true);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    capture.set(cv::CAP_PROP_FPS, 120);
    // capture.set(cv::CAP_PROP_BUFFERSIZE, 5);
  if (!capture.isOpened()) {
    printf("can not open video device!!!\n");
    return 0;
  } 
 // 启动预读取线程
    // thread thread_cam(preload_thread, ref(capture));
// 启动显示线程
    thread thread_show(display_thread);
// inference_thread = thread(inference_thread_func);
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

  // if(watch.InLoop ==1||watch.cross_flag ==1||watch.slope_flag ==1||motion.params.protect_status==true)         //设别到元素，蜂鸣器响
  // {
  //   uart->buzzerSound(uart->BUZZER_DING);        
  // } 
  // uart->buzzerSound(uart->BUZZER_DING);  
  // 初始化参数
  // Scene scene = Scene::NormalScene;     // 初始化场景：常规道路
  // Scene sceneLast = Scene::NormalScene; // 记录上一次场景状态

  uart->carpid(300, 750, 0, 0); // 调pid，参数分别为p，i，d，是否存入flash
  distance_start = uart->get_distance();
  int fps_count=0;
  float sum_time=0;
  float ave_time= 0;
  init_AI_param();
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
              motion.params.STEER_MID,motion.params.STEER_MIN,motion.params.STEER_MAX,&motion.params.Is_showimg,
              motion.params.zebra_distance,motion.params.zebra_begin_time,motion.params.zebra_line_count,motion.params.zebra_speed,
              motion.params.catering_speed,motion.params.elenable); 
              // 写在init_setpara（）后面
  // 改config文件，改set_setpara函数
  Is_showimg=motion.params.Is_showimg;
  car_begin(); // 初始化车启动的标志位
  //////////////////视频输出


  // 检查视频写入器是否成功创建
  if (!video.isOpened()) {
    cerr << "Could not open the output video file for writing"
              << endl;
    return -1;
  }
  signal(SIGINT, sigint_handler); // 中断，结束的时候
// capture.set(cv::CAP_PROP_BUFFERSIZE, 3);
  
  
  cv::Mat img;//原图像
  Mat img640,img320;
  std::future<void> future;
  vector<PredictResult> AIresults;
  bool AIFlag=false;
  while (1) {
    // 等待定时    
    auto start = chrono::high_resolution_clock::now(); // 记录开始时间
    // this_thread::sleep_until(next_time);
    // next_time += interval;
    // if(watch.InLoop ==1||watch.cross_flag ==1||watch.slope_flag ==1||motion.params.protect_status==true)         //设别到元素，蜂鸣器响
    // {
    //   uart->buzzerSound(uart->BUZZER_DING);        
    // } 
    // uart->buzzerSound(uart->BUZZER_DING);    
    //     // cout<<"camwf"<<motion.params.camwf<<endl;
// 从预读取队列中取出帧
          if (!capture.read(img640)) { 
        printf("%s","capture_read_fault");// 如果读取失败，跳过当前帧
        continue;   
          }
          Size dsize_first = Size(188, 120);//在这里做大小缩放
          //Size dsize_second = Size(320, 320);
          cv::resize(img640, img, dsize_first, 0, 0, INTER_AREA);//效果比较好
          // resize (img640,img320,dsize_second,0,0,INTER_AREA);
    //     {
    //         unique_lock<mutex> lock(preload_mtx);
    //         preload_cv.wait(lock, [] { return !preload_queue.empty() || stop_threads; });
    //         if (stop_threads && preload_queue.empty()) {
    //             break;
    //         }
     
    //         img = preload_queue.front();
    //         preload_queue.pop(); //
    //         // Size dsize_first = Size(188, 120);//在这里做大小缩放
    //         // cv::resize(img240, img, dsize_first, 0, 0, INTER_AREA);//效果比较好
    //      auto end_CV = chrono::high_resolution_clock::now(); // 记录结束时间
    // chrono::duration<double> duration_CV = end_CV - start;
    // cout << "reading_CV " << duration_CV.count() << " seconds" << endl;     
    //     }
        // if (1) {void *p;//？
        //   cv::Mat img_record=img.clone();
        //   img_record = cv::Mat(188, 120, CV_8UC3, (uchar*)p);
        //   video.write(img_record);
        //     // 检查VideoWriter是否成功创建
        //     if (!video.isOpened()) {
        //       cout << "无法创建视频文件" << endl;
        //       return -1;
        //     }
          
        // }
       
        Mat AIimg = img.clone();
        Mat Hsvmat = img.clone();
        auto start_AI = chrono::high_resolution_clock::now(); // 记录开始时间
        //  detection->inference(img);//AI推理
      if(!stop_inference_thread)
      {
        if(!future.valid())
        future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);
       if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready )
       {
         // 如果异步任务已经完成
         // 获取结果并处理
         AIresults = detection->results;
         AIFlag = true;
         future = std::async(std::launch::async, [&detection](const cv::Mat image) {detection->inference(image);}, AIimg);
       } 
       else 
       {
         AIresults.clear();
         AIFlag = false;
       }
      }
      //  HSV_Process(Hsvmat);

// 主线程提交帧

//       ai多线程检测
//  {
//   lock_guard<mutex> lock(inference_mutex);
//   if (inference_queue.size() < 3) { // 控制队列长度
//       inference_queue.push(img.clone());
//       inference_cv.notify_one();
//   }
//  }
        auto end_AI = chrono::high_resolution_clock::now(); // 记录开始时间
        chrono::duration<double> durationAI = end_AI - start_AI;
        printf("%s%.5f ","AI inference",durationAI.count());
// // 将处理后的帧放入显示队列
// auto end22 = chrono::high_resolution_clock::now(); // 记录结束时间
//     chrono::duration<double> duration22 = end22 - start;
    // cout << "reading2 " << duration22.count() << " seconds" << endl;
   
    for (int i = 0; i < ROWSIMAGE; ++i) {//最多16us
      fill(imo3[i], imo3[i] + COLSIMAGE, 0);//全给灰色？
    }
    for (int i = 0; i < ROWSIMAGE; ++i) {
      fill(imo4[i], imo4[i] + COLSIMAGE, 0);
    }
//     auto end2 = chrono::high_resolution_clock::now(); // 记录结束时间
  //mpu数据60us
    auto start_message = chrono::high_resolution_clock::now(); 
    mpu6050_now = uart->get_mpu6050();   // mpu6050_now就是mpu的数值
    distance_now = uart->get_distance(); // 编码器获取
    Gyro_Z = uart->get_gyro_z();         // 角速度
    cout << "现在的距离积分" << distance_now - distance_start ;
    cout <<"          mpu"<< mpu6050_now << endl; // 输出mpu
    // cout << "角速度" << Gyro_Z << endl;
    angal_integeral(mpu6050_now);    // 把现在角度积分不断传入
    distant_integeral(distance_now); //
    //[01] 视频源读取
    // 读取mpu6050  
  auto end_message = chrono::high_resolution_clock::now(); // 记录结束时间
    chrono::duration<double> duration_mess = end_message - start_message;
    // cout << "message " << duration_mess.count() << " seconds" << endl;
///从while到这28ms
    //  if (motion.params.saveImg && !motion.params.debug) // 存储原始图像
    //    savePicture(img);
    //  if (waitKey(1) == 27) { // 如果用户按下 ESC 键，退出循环
    //   // 按键发车
    // }
    //[02] 图像预处理

    Mat imgCorrect = img; // 图像矫正（已停止
    Mat imgBinary = preprocess.binaryzation(imgCorrect); // 图像二值化
    for (int i = 0; i < ROWSIMAGE; ++i) {
      for (int j = 0; j < COLSIMAGE; ++j) {
        Grayscale[i][j] = imgBinary.at<uchar>(i, j);
      }
    }//是白色，
    scan_line();//祖传算法扫线
    

    //[06] 快餐店检测
    if ((scene == Scene::NormalScene || scene == Scene::CateringScene) &&
        motion.params.catering) {
      if (catering.process(imgBinary,detection->results))  // 传入二值化图像进行再处理
        {scene = Scene::CateringScene;
         Element=ecatering;}
      else
        scene = Scene::NormalScene;
    }
           
  

    // //[07] 临时停车区检测
    // if ((scene == Scene::NormalScene || scene == Scene::LaybyScene) &&
    //     motion.params.layby) {
    //   if (layby.process1(imgBinary,detection->results))  // 传入二值化图像进行再处理
    //     {scene = Scene::LaybyScene;
    //      Element=elayby;}
    //   else
    //   {        
    //     scene = Scene::NormalScene;       
    //   }
    // }
    if ((scene == Scene::NormalScene||Element == black_obstacle || scene == Scene::LaybyScene) &&
        motion.params.layby) {
      if (layby.process1(imgBinary,detection->results))  // 传入二值化图像进行再处理
        {scene = Scene::LaybyScene;
         Element=elayby;}
      else
      {        
        scene = Scene::NormalScene;       
      }
    }
    

    //[08] 充电停车场检测
    if ((Element==Right_ring||scene == Scene::NormalScene || scene == Scene::ParkingScene) &&
        motion.params.parking) {
       parking.process2(imgBinary,detection->results);  // 传入二值化图像进行再处理   
    }
    
    //[09] 坡道区检测
    if ((scene == Scene::NormalScene || scene == Scene::BridgeScene) &&
        motion.params.bridge) {
      if (bridge.process(detection->results))
        {scene = Scene::BridgeScene;
         Element=ebridge;}
      else
        scene = Scene::NormalScene;
    }

    // [10] 障碍区检测
    if ((Element==None || Element==eobstacle)&& ( motion.params.obstacle))
       {
          // cout<<"开始障碍检测"<<endl<<endl;
       aobstacle.process2( detection->results);

    }


auto start_CAM = chrono::high_resolution_clock::now(); // 记录结束时间
///从while到这30ms
    /*///核心控制部分////*/
    CAM_CPU_while();/////400us
    // cout<<"ai"<<scene<<endl;
auto end_CAM = chrono::high_resolution_clock::now(); // 记录结束时间
    chrono::duration<double> duration_CAM = end_CAM - start_CAM;
    // cout << "cpu " << duration_CAM.count() << " seconds" << endl;
 ///从while到这30ms   
    if (motion.params.debug) // 开启视频
    {
      // 定义缩放比例
      // float orignal_scale = 1; // 你可以根据需要调整这个值
      // cv::Mat resizedImage;// 创建一个 Mat 对象来存储缩放后的图像
      // resize(img, resizedImage, Size(), orignal_scale, orignal_scale);// 使用 resize 函数对图像进行缩放
      // imshow("original", resizedImage);// 显示缩放后的原图像
      draw_imo_color(imo3, imo3_img); // 扫弦图绿色是右边，红色是左边
      draw_imo_color(imo4, imo4_img); // 逆透视
      //////////////如果嫌太慢可以用这个
      cv::Mat colorImage = cv::Mat::zeros(120, 188, CV_8UC3);
      if(motion.params.debug==2)
    {  
      cv::cvtColor(imgBinary, colorImage, cv::COLOR_GRAY2BGR);
      for (int i = 0; i < colorImage.rows; ++i) {
        for (int j = 0; j < colorImage.cols; ++j) {
          cv::Vec3b overlayPixel = imo3_img.at<cv::Vec3b>(i, j);
          if (overlayPixel != cv::Vec3b(0, 0, 0)) {
            colorImage.at<cv::Vec3b>(i, j) = overlayPixel;
          }
        }
      }
        for (int i = 0; i < colorImage.rows; ++i) {
          for (int j = 0; j < colorImage.cols; ++j) {
          cv::Vec3b overlayPixel = imo3_img.at<cv::Vec3b>(i, j);
          if (overlayPixel != cv::Vec3b(0, 0, 0)) { // 如果 imo3_img 的像素不是黑色
            colorImage.at<cv::Vec3b>(i, j) = overlayPixel; // 叠加到 colorImage 上
        }
        }
      }
    }

      // 将二值图像 imgBinary 转换为灰度图像
// 创建一个彩色图像 colorImage，用于叠加
      auto startshow = chrono::high_resolution_clock::now(); // 记录结束时间

// cv::Mat colorImage = cv::Mat::zeros(120, 188, CV_8UC3);
// // 将二值图像 imgBinary 转换为灰度图像，并将黑色部分改为灰色
// for (int i = 0; i < imgBinary.rows; ++i) {
//     for (int j = 0; j < imgBinary.cols; ++j) {
//         uchar binaryPixel = imgBinary.at<uchar>(i, j); // 获取二值图像的像素值
//         if (binaryPixel == 0) {
//             // 如果像素是黑色（值为 0），改为灰色（128）
//             colorImage.at<cv::Vec3b>(i, j) = cv::Vec3b(128, 128, 128); // BGR格式的灰色
//         } else {
//             // 如果像素是白色（值为 255），保持白色
//             colorImage.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255); // BGR格式的白色
//         }
//     }
// }
// 叠加 imo3_img 的非黑色像素到 colorImage 上
cv::Rect rect(50,119-80,68,50);
cv::rectangle(colorImage, rect,cv::Vec3b(255, 0, 0), 1);

colorImage.at<cv::Vec3b>(119-parking.blacky, parking.blackx) = cv::Vec3b(255, 0, 0); // BGR格式（红色点）
if(!stop_inference_thread)detection->drawBox(colorImage);
int *params_mode = set_show_params_mode(motion.params.show_params_mode);
show_params(colorImage,params_mode);
if (motion.params.record_video&& !colorImage.empty()) {
  video.write(colorImage);
  // static int write_count = 0;
  //     write_count++;
  //     cout << "成功写入第" << write_count << "帧" << endl;
  
}
   
      double scale = 1.5;
      // 缩放图像
      Mat resizedImage1;
      Mat resizedImage2;
      Mat resizedImage3;
      Mat resizedImage4;
      //  resize(imgBinary, resizedImage1, Size(), scale, scale);
    if(motion.params.debug==2)
      resize(colorImage, resizedImage2, Size(), scale, scale); // 补线图 colorImage  左图
    else resize(imo3_img, resizedImage2, Size(), scale, scale);
      resize(imo4_img, resizedImage3, Size(), scale, scale);   // 逆透视图
      // Hsvmat.copyTo(colorImage, Hsvmat); // 将HSV矩形叠加到补线图
     
   
      cv::Mat combinedFrame;  //定义变量
      cv::hconcat(resizedImage2, resizedImage3, combinedFrame);//两图拼接

       if(!motion.params.record_video){
// 在生产者线程（如主线程）中推送帧时添加判断：
{
  lock_guard<mutex> lock(display_mtx);
  if (display_queue.size() >= MAX_DISPLAY_QUEUE_SIZE) {
      display_queue.pop(); // 丢弃旧帧，保留最新帧
  }
  display_queue.push(combinedFrame.clone());
  display_cv.notify_one();
}}
      auto endshow = chrono::high_resolution_clock::now(); // 记录结束时间
    chrono::duration<double> duration_show = endshow - end_CAM;
    // cout << "draw " << duration_show.count() << " seconds" << endl;
      // imshow("789", combinedFrame);//显示图像    imshow("789", combinedFrame);
      // waitKey(1); // 等待显示，不能删！！！
      auto end_a = chrono::high_resolution_clock::now(); // 记录结束时间
      chrono::duration<double> duration_a = end_a - endshow;
      // cout << "show " << duration_a.count() << " seconds" << endl;
    }
 ///从while到这35ms 
    motion.params.motion_start = 1;
    if (motion.params.motion_start) // 是否运动
    {
      if (mycar.uart_servo < STEER_MIN)
        mycar.uart_servo = STEER_MIN;
      if (mycar.uart_servo > STEER_MAX)
        mycar.uart_servo = STEER_MAX;
      // mycar.uart_speed = 1;
      // mycar.uart_servo=STEER_MID;
      // cout << "目标速度" << mycar.uart_speed << "舵机PWM" << mycar.uart_servo << endl;
      uart->carControl(
          mycar.uart_speed,
          mycar.uart_servo); // 串口通信控制车辆---传给下位机进行控制
    }
    ///从while到这35ms 
     auto end_all = chrono::high_resolution_clock::now(); // 记录结束时间
    chrono::duration<double> duration_all = end_all - start;

    mycar.RUNTIME += duration_all.count()*1000; // 运行时间
    
    fps_count++;
    sum_time+=duration_all.count()*1000;
    ave_time=sum_time/fps_count;

    // cout << "FrameTime " <<1/duration_all.count()<<" FPS    " <<duration_all.count()*1000 << "  ms   " <<"平均时间： "<<ave_time<<" ms"<< endl;
    printf(">> FrameTime: %.2fms | %.2ffps \n", duration_all.count()*1000,
          1.0 / duration_all.count());
    cout <<"平均时间： "<<ave_time<<" ms"<< endl;

  }
  // 等待线程结束
    if (thread_cam.joinable()) {
        thread_cam.join();
    }
    if (thread_show.joinable()) {
        thread_show.join();
    }
    if (inference_thread.joinable()) {
      inference_thread.join();
  }
    // preload_cv.notify_all();// 唤醒预读取线程
    // display_cv.notify_all(); // 唤醒显示线程
  video.release();
  uart->close(); // 串口通信关闭
  capture.release();//关闭摄像头
  cv::destroyAllWindows(); // 关闭所有窗口
  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                  /*下面是函数*/
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void init_AI_param()
{
  setpara.cross_speed = motion.params.cross_speed;


  aobstacle.eobstacle_distance=motion.params.aobstacle_distance;
  layby.Layby_distance_left0=motion.params.layby_distance_left0;
  layby.Layby_distance_left1=motion.params.layby_distance_left1;
  layby.Layby_slope_left=motion.params.layby_slope_left;
  layby.Layby_distance_right0=motion.params.layby_distance_right0;
  layby.Layby_distance_right1=motion.params.layby_distance_right1;
  layby.Layby_slope_right=motion.params.layby_slope_right;
  
}
//预读取线程负责从摄像头或视频源中读取帧，并将帧放入预读取队列
void preload_thread(cv::VideoCapture& capture) {
    cv::Mat img;
    while (!stop_threads) {
      auto start_read = chrono::high_resolution_clock::now(); // 记录结束时间
        if (!capture.read(img)) { 
            printf("%s","capture_read_fault");// 如果读取失败，跳过当前帧
            continue;
        }
 auto end_read = chrono::high_resolution_clock::now(); // 记录结束时间
    chrono::duration<double> duration_read = end_read - start_read;
    cout << "read_thread " << duration_read.count() << " seconds" << endl;
        // 将帧放入预读取队列
        {
            lock_guard<mutex> lock(preload_mtx);
            if (preload_queue.size() >= MAX_PRELOAD_QUEUE_SIZE) {
                preload_queue.pop(); // 丢弃旧的帧
            }
            auto start_resize = chrono::high_resolution_clock::now();
            Size dsize_first = Size(188, 120);//在这里做大小缩放
            cv::resize(img, img, dsize_first, 0, 0, INTER_AREA);//效果比较好
            auto end_resize = chrono::high_resolution_clock::now(); // 记录结束时间
            chrono::duration<double> duration_size = end_resize - start_resize;
            cout << "reszie_thread " << duration_size.count() << " seconds" << endl;
            preload_queue.push(img.clone());
        }
        preload_cv.notify_one(); // 通知主线程
    }
}

// 推理线程函数

// void inference_thread_func() {
//   while (!stop_inference_thread) {
//       cv::Mat frame;
//       {
//           unique_lock<mutex> lock(inference_mutex);
//           inference_cv.wait(lock, [] { return !inference_queue.empty() || stop_inference_thread; });
//           if (stop_inference_thread) break;
//           frame = inference_queue.front();
//           inference_queue.pop();
//       }
//       detection->inference(frame);
//   }
// }
int display_interval = 4; // 每 display_interval 帧显示一次
int display_counter = 0;
// 显示线程函数，
void display_thread() {
    int display_counter = 0;
    // int display_interval = 5; // 每 2 帧显示一次

    while (!stop_threads) {
        cv::Mat img;
        // cout<<"Ishowommg"<<Is_showimg<<endl;
        auto start_imshow = chrono::high_resolution_clock::now(); // 记录开始时间
        {
            unique_lock<mutex> lock(display_mtx);
            // 等待队列中有帧可显示
            display_cv.wait(lock, [] { return !display_queue.empty() || stop_threads; });
            if (stop_threads && display_queue.empty()) {
                break; // 如果线程需要退出且队列为空，则退出
            }
            img = display_queue.front(); // 取出帧
            display_queue.pop();         // 从队列中移除
          }
          // auto start_imshow = chrono::high_resolution_clock::now(); // 记录结束时间
          if (++display_counter % display_interval == 0) {
            imshow("sacnline", img);
            waitKey(1); // 使用 1ms 延迟保证窗口刷新
        }  
          auto end_imshow = chrono::high_resolution_clock::now(); // 记录结束时间
          chrono::duration<double> duration_imshow = end_imshow - start_imshow;
          // cout << "imshow_thread " << duration_imshow.count() << " seconds" << endl;
        // 降低显示帧率
       
    }
}



//// Ctrl+C 被按下时执行的代码
void sigint_handler(int sig) {

  if (sig == SIGINT) {
    
    cout << "Ctrl+C 被按下！" << endl;
    // uart->carpid(500, 1000, 0,
    //              0); // 调刹车pid，参数分别为p，i，d，是否存入flash
    uart->carControl(0, 4800);

      // 设置停止标志并等待线程结束
    stop_threads = true; 
     // 等待线程结束
    if (thread_cam.joinable()) {
        thread_cam.join();
    }
    // if (thread_show.joinable()) {
    //     thread_show.join();
    // }
    exit(0);
  }
}


void CAM_CPU_while(void) {
  if(setpara.elenable)
  {Element_recognition(); }        // 元素识别
  linefix();                      //赛道补线
  original_err_calculation();  //原始误差计算
  dir_control();                 // 舵机控制
  motor_control();               // 电机控制
}

void draw_imo_color(uint8_t myimo[ROWSIMAGE][COLSIMAGE], Mat mat) {
  for (int i = 0; i < ROWSIMAGE; ++i) {
    for (int j = 0; j < COLSIMAGE; ++j) {
      uint8_t value = myimo[i][j];

      // 根据值设置颜色
      cv::Vec3b color;
      switch (value) {
      case 1:

        color = cv::Vec3b(0, 0, 255);     // 红色
        break;
      case 2:
        color = cv::Vec3b(0, 255, 0);     // 绿色
        break;
      case 3:
        color = cv::Vec3b(0, 255, 255);   // 黄色
        break;
      case 4:
        color = cv::Vec3b(255, 0, 0);     // 蓝色
        break;
      case 5:
        color = cv::Vec3b(255, 0, 255); // 洋红色
        break;
      case 6:
        color = cv::Vec3b(0, 255, 255);   // 青色
        break;
      case 7:
        color = cv::Vec3b(128, 128, 128); // 灰色
        break;
      case 8:
        color = cv::Vec3b(128, 0, 0);   // 深红色
        break;
      case 9:
        color = cv::Vec3b(0, 128, 0);   // 深绿色
        break;
      default:
        color = cv::Vec3b(0, 0, 0);       // 黑色
        break;
      }

      // 设置像素颜色
      mat.at<cv::Vec3b>(119 - i, j) = color;
    }
  }
}
void show_params(Mat img, int *data) {
  for (size_t i = 0; i < 11; i++)
  {
    data[i]=0;//清空数组
  }
  
  vofa.turn[0] = Element;
  vofa.turn[1] = mycar.original_err;
  vofa.turn[2] = mycar.uart_speed;
  vofa.turn[3] = mycar.uart_servo;
  vofa.turn[4] = CAM_Turn.PID_output;
  vofa.turn[5] = mycar.RUNTIME;
  // vofa.turn[6]=abs(mycar.original_err);
  vofa.turn[6] = watch.InLoop;
  vofa.turn[7] = mycar.present_speed;

  vofa.loop[0] = Element;
  vofa.loop[1] = watch.InLoop;        // 入环标志位
  vofa.loop[2] = watch.InLoopAngleL;  // 入左环前直行的第一个角所在行（直道与圆环交接的角点）
  vofa.loop[3] = watch.InLoopAngleR;  //入右环前直行的第一个角所在行（直道与圆环交接的角点）
  vofa.loop[4] = watch.InLoopblackpoint;         // 角点附近黑色像素数量
  vofa.loop[5] = distance_integral.integeral_data;//积分距离
  vofa.loop[6] = watch.InLoopAngle2;  //开始转向入环时前方的角点所在行（直道与圆环交接的角点）
  vofa.loop[7] = watch.OutLoop;//出环标记变量 1:出环时进入直道与圆环交界处
  vofa.loop[8] = watch.OutLoopAngle2;   //出环后直行时前方的角点所在行（直道与圆环交接的角点）


  vofa.speed[0] = mycar.left_speed;
  vofa.speed[1] = mycar.right_speed;
  vofa.speed[2] = mycar.steer_pwm;
  vofa.speed[3] = mycar.left_pwm_set;
  vofa.speed[4] = mycar.right_pwm_set;
  vofa.speed[5] = (mycar.right_speed - mycar.present_speed) /
                  mycar.present_speed;
  vofa.speed[6] = mycar.RUNTIME;
  vofa.speed[7] = mycar.target_speed;
  vofa.speed[8] = mycar.speed_ctrl;//mycar.speed_ctrl;

  vofa.element_rem[0] = Element_rem.Element_count;
  vofa.element_rem[1] = Element_rem.Element_data[Element_rem.Element_count];
  vofa.element_rem[2] = watch.InLoop;
  vofa.element_rem[3] = watch.cross_flag;
  vofa.element_rem[4] = Element;
  vofa.element_rem[5] = watch.slope_flag;
  // vofa.element_rem[6] = imu.pitch;
  vofa.element_rem[7] = watch.out_garage_flag;

  vofa.obstacle[0] = Element;
  vofa.obstacle[1] = watch.black_obstacle_flag;
  vofa.obstacle[2] = 0;
  vofa.obstacle[3] = scene;
  vofa.obstacle[4] = bridge.counterRec;
  vofa.obstacle[5] = bridge.bridgeEnable;
  vofa.obstacle[6] = mycar.RUNTIME;

  vofa.broken[0] = Element;
  vofa.broken[1] = watch.broken_circuit_flag;
  vofa.broken[2] = Element_rem.Element_count;
  // vofa.broken[2] = imu.pitch;//惯导
  // vofa.broken[3] = imu.gyroy;  
  vofa.broken[5] = mycar.RUNTIME;

  vofa.cross[0] = Element;
  vofa.cross[1] = watch.track_count_far;
  vofa.cross[2] = watch.cross_flag;
  vofa.cross[3] = watch.cross_LD_angle;
  vofa.cross[4] = watch.cross_RD_angle;
  vofa.cross[5] = watch.cross_AngleL;         //左角点所在行
  vofa.cross[6] = watch.cross_AngleL_x;
  vofa.cross[7] = watch.cross_AngleR;         //右角点所在行
  vofa.cross[8] = watch.cross_AngleR_x;

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

  vofa.fuzzy_speed[0] = Element;
  vofa.fuzzy_speed[1] = mycar.target_left_speed;
  vofa.fuzzy_speed[2] = mycar.target_right_speed;
  vofa.fuzzy_speed[3] = mycar.left_speed;
  vofa.fuzzy_speed[4] = mycar.right_speed;
  vofa.fuzzy_speed[5] = mycar.steer_pwm;
  vofa.fuzzy_speed[6] = watch.track_count;
  vofa.fuzzy_speed[7] = mycar.left_pwm_set;
  vofa.fuzzy_speed[8] = mycar.RUNTIME;

  vofa.steer_cal[0] = Element;
  vofa.steer_cal[1] = mycar.nonlinear_trackpos;
  vofa.steer_cal[2] = mycar.Steer_PWM_Kp;
  vofa.steer_cal[3] = mycar.raw_track_diff;
  vofa.steer_cal[4] = mycar.track_diff;
  vofa.steer_cal[5] = mycar.original_err;
  vofa.steer_cal[6] = mycar.steer_pwm;
  vofa.steer_cal[7] = mycar.d_original_err;

  vofa.zebra[0] = Element;
  vofa.zebra[1] = watch.zebra_flag;
  vofa.zebra[2] = mycar.target_speed;
  vofa.zebra[3] = mycar.present_speed;
  vofa.zebra[4] = watch.stop_count;
  vofa.zebra[5] = mycar.RUNTIME;
  vofa.zebra[6] = distance_integral.integeral_flag;
  vofa.zebra[7] = distance_integral.integeral_data;
  vofa.zebra[8] = distance_integral.integeral_Thres;


  vofa.mpu6050[0]=Element;
  vofa.mpu6050[1]=mpu6050_now;
  vofa.mpu6050[2]=distance_now;

  vofa.catering[0]=Element;
  vofa.catering[1]=catering.counterRec;
  vofa.catering[2]=catering.cateringEnable;
  vofa.catering[3]=catering.burgerLeft;
  vofa.catering[4]=catering.burgerY;
  vofa.catering[5]=catering.state;
  vofa.catering[6]=watch.cross_LD_angle;
  vofa.catering[7]=watch.cross_RD_angle;
  vofa.catering[8]=setpara.catering_speed;


  vofa.eobstacle[0]=Element;
  vofa.eobstacle[1]=watch.watch_lost;
  vofa.eobstacle[2]=aobstacle.enable;
  vofa.eobstacle[3]=aobstacle.cone_pedestrian_left;
  vofa.eobstacle[4]=aobstacle.cone_pedestrian_right;
  vofa.eobstacle[5]=aobstacle.resultObs.x;
  vofa.eobstacle[6]=aobstacle.resultObs.y;
  vofa.eobstacle[7]=get_integeral_data(&distance_integral);
  vofa.eobstacle[8]=get_integeral_state(&distance_integral);

  vofa.eparking[0]=Element;
  vofa.eparking[1]=parking.step;
  vofa.eparking[2]=get_integeral_data(&angle_integral);
  vofa.eparking[3]=get_integeral_state(&angle_integral);
  vofa.eparking[4]=parking.ave_steer_pwm;
  vofa.eparking[5]=parking.blackx ;
  vofa.eparking[6]=parking.blacky;
  vofa.eparking[7]=get_integeral_data(&distance_integral);
  vofa.eparking[8]=get_integeral_state(&distance_integral);


  
  
  
  string text1;
  switch (Element)
  {
  case 0:      text1 = " None: "           ;break;//无元素       0
  case 1:      text1 = " Left_ring: "      ;break;//左环         1
  case 2:      text1 = " Right_ring: "     ;break;//右环         2
  case 3:      text1 = " Slope: "          ;break;//斜坡         3
  case 4:      text1 = " broken_circuit: " ;break;//断路         4
  case 5:      text1 = " eobstacle: "       ;break;//障碍         5
  case 6:      text1 = " ingarage: "       ;break;//入库         6
  case 7:      text1 = " outgarage: "      ;break;//出库         7
  case 8:      text1 = " crossing: "       ;break;//十字         8
  case 9:      text1 = " black_obstacle: " ;break;//小型黑色路障  9
  case 10:     text1 = " zebra: "          ;break;//斑马线       10
  case 11:     text1 = " running_protect " ;break;//保护         11
  case 12:     text1 = " catering"          ;break;
  case 13:     text1 = " layby"            ;break;
  case 14:     text1 = " parking"          ;break;
  case 15:     text1 = " bridge"           ;break;
  default: break;
  }
  // string text1 = " 1: " + to_string(data[0]);
  string text2 = " 2: " + to_string(data[1]);
  string text3 = " 3: " + to_string(data[2]);
  string text4 = " 4: " + to_string(data[3]);
  string text5 = " 5: " + to_string(data[4]);
  string text6 = " 6: " + to_string(data[5]);
  string text7 = " 7: " + to_string(data[6]);
  string text8 = " 8: " + to_string(data[7]);
  string text9 = " 9: " + to_string(data[8]);
  string text10 = "10: " + to_string(data[9]);
  string text11 = "11: " + to_string(data[10]);
  cv::putText(img, text1, cv::Point(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4,
              cv::Scalar(0, 0, 255), 1);
  cv::putText(img, text2, cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text3, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text4, cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text5, cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text6, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text7, cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text8, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text9, cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text10, cv::Point(10, 100), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
  cv::putText(img, text11, cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.3,
              cv::Scalar(255, 0, 255), 1);
              
}
int *set_show_params_mode(int num) {
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
    return vofa.eobstacle;
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
    return vofa.mpu6050;
    break;
  case 13:
    return vofa.catering;
    break;
  case 14:
    return vofa.eparking;
    break;
  }
}


void HSV_Process(Mat& frame) { // frame是传入的原图引用
  Mat hsv_frame;
  cvtColor(frame, hsv_frame, COLOR_BGR2HSV);

  // 创建颜色掩码（原有代码）
  inRange(hsv_frame, lower_red1, upper_red1, mask_red);
  inRange(hsv_frame, lower_red2, upper_red2, mask_red);
  bitwise_or(mask_red, mask_red, mask_red);
  inRange(hsv_frame, lower_yellow, upper_yellow, mask_yellow);
  inRange(hsv_frame, lower_purple, upper_purple, mask_purple);

  // 形态学操作（原有代码）
  morphologyEx(mask_red, mask_red, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
  morphologyEx(mask_yellow, mask_yellow, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));
  morphologyEx(mask_purple, mask_purple, MORPH_OPEN, getStructuringElement(MORPH_ELLIPSE, Size(5,5)));

  // 新增：找到轮廓并绘制到原图frame上
  vector<vector<Point>> contours_red, contours_yellow, contours_purple;

  // 处理红色
  findContours(mask_red, contours_red, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  processContours(frame, contours_red, Scalar(0, 0, 255), "Red"); // 红色矩形

  // 处理黄色
  findContours(mask_yellow, contours_yellow, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  processContours(frame, contours_yellow, Scalar(0, 255, 255), "Yellow"); // 黄色矩形

  // 处理紫色
  findContours(mask_purple, contours_purple, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  processContours(frame, contours_purple, Scalar(255, 0, 255), "Purple"); // 紫色矩形
}
// 辅助函数：处理轮廓并标注
void processContours(Mat& frame, const vector<vector<Point>>& contours, Scalar color, const string& label) {
  for (const auto& contour : contours) {
      double area = contourArea(contour);
      if (area > 100) {
          Moments m = moments(contour);
          int cx = m.m10/m.m00;
          int cy = m.m01/m.m00;
          rectangle(frame, boundingRect(contour), color, 2);
          putText(frame, label, Point(cx, cy), FONT_HERSHEY_SIMPLEX, 1, color, 2);
      }
  }
}