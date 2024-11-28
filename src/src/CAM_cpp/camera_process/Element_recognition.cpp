/*
 * Element_recognition.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "Element_recognition.hpp"
#include "cross.hpp"
#include "mycar.hpp"
#include "obstacle.hpp"

#include "zebra.hpp"
using namespace std;
Element_range Element = None;
struct Element_struct Element_rem;
struct watch_o watch;
// 总的元素判断的代码
void Element_recognition() {
  // Element=broken_circuit;
  switch (Element) {
  case None:
    enter_task(); // 元素入口函数，在此识别并进入函数   从此处开始看
    break;
  case Left_ring:
    left_ring_first_angle(); // 寻找左环第一个角点
    left_ring_circular_arc(); // 检测左环入环第一个角点后的圆弧,用于补线
    left_ring_begin_turn();    // 左环开始转向
    left_ring_second_angle();  // 检测左环第二个角点,用于补线
    left_ring_in_loop();       // 检验小车是否完全入环
    left_ring_prepare_out();   // 小车角度积分完成，准备出环
    left_ring_out_loop_turn(); // 出左环左转
    left_ring_out_angle();     // 检测出环时右角点位置
    left_ring_out_loop();      // 右侧为直线时直行
    left_ring_straight_out_angle(); // 检测出左环进入直线后左侧角点，用于补线
    left_ring_complete_out(); // 检测完全出环
    break;
  case Right_ring:
    cout << "识别到圆环了" << endl;
    right_ring_first_angle();  // 检测右环第一个角点
    right_ring_circular_arc(); // 检测右环入环第一个角点后的圆弧
                               // ----即为实际的第一个角点
    right_ring_begin_turn(); // 右环开始转向函数     ---开始角度积分
    right_ring_second_angle(); // 检测右环第二个角点     结合补线函数看
    right_ring_in_loop();            // 检验小车是否完全入右环
    right_ring_prepare_out();        // 小车角度积分完成，准备出环
    right_ring_out_loop_turn();      // 出右环右转   有距离积分+loop_out   
    right_ring_out_angle();          // 检测出环时左角点位置
    right_ring_out_loop();           // 左侧为直线时直行    有loop_out 
    right_ring_straight_out_angle(); // 检测出右环进入直线后左侧角点
    right_ring_complete_out();       // 检测完全出环
    break;
  case ingarage:
    //             left_garage_first_angle();//检测左侧车库第一个角点
    /*             left_garage_second_turn();
                 right_garage_first_angle();
                 right_garage_second_turn();
                 garage_stop();*/
    if (setpara.start_mode == 1 || setpara.start_mode == 3) {
      left_garage_first_angle();
      left_garage_second_angle();
      left_garage_begin_turn();
      left_garage_stop_turn();
      garage_stop2();
    } else if (setpara.start_mode == 2 || setpara.start_mode == 4) {
      right_garage_first_angle();
      right_garage_second_angle();
      right_garage_begin_turn();
      right_garage_stop_turn();
      garage_stop2();
    }
    break;
  case broken_circuit:
    broken_circuit_enter();
    broken_circuit_slow();
    broken_circuit_complete_enter();
    broken_circuit_complete_out();
    break;
  case obstacle:
    obstacle_stop();
    break;
  case crossing:
    cross_running();
    cross_running2();
    cross_out();
    break;
  case Slope:
    slope_down();
    slope_out();
    break;
  case outgarage:
    out_garage_turn();
    complete_out_garage();
    break;
  case black_obstacle:
    // black_obstacle_enter();
    black_obstacle_out();
  case zebra:
    zebra_stop();
    zebra_out();

  default:
    break;
  }
}
// 元素识别入口函数
void enter_task() {
  switch (Element_rem.Element_data[Element_rem.Element_count]) {
  case 1:
    left_ring_first_angle(); // 检测左环第一个角点
    break;
  case 2:
    right_ring_first_angle(); // 检测右环第一个角点
    break;
  case 3:
    // slope_enter();
    break;
  case 4:
    broken_circuit_prepare();
    break;
  case 5:

    break;
  case 6:
    garage_prepare();
    break;
  case 7:
    out_garage();
    break;
  case 8:
    cross_enter();
    break;
  case 0: // 元素标记为0时自动扫描元素    当时只用了0这一种情况
    if (mycar.RUNTIME >= setpara.begin_time || mycar.car_running == 0) {
      left_ring_first_angle();  // 检测左环第一个角点
      right_ring_first_angle(); // 检测右环第一个角点

      // slope_enter();
      // obstacle_identification();

      if (setpara.cross_open_flag >= 1) {
        cross_enter();
      }

      if (setpara.bla_obs_open_flag >= 1) {
        black_obstacle_enter();
      }

      // zebra_enter();
    }
    break;
  default:
    out_element();
    // zebra_out();
    break;
  }
  // zebra_indentification(); //斑马线识别
}
// 元素初始化
void element_init() {
  uint8_t count = 0;
  Element_rem.Element_count = 0;
  Element_rem.Element_data[0] = 7; // 出库
  uint8_t Element[21];
  if (setpara.start_mode == 1 || setpara.start_mode == 3) {
    for (int i = 1; i < 21; i++) {
      Element_rem.Element_data[i] = setpara.set_element[i];
    }
  } else if (setpara.start_mode == 2 || setpara.start_mode == 4) {
    for (int i = 1; i < 21; i++) {
      if (setpara.set_element[i] != 6) {
        Element[i] = setpara.set_element[i];
        count++;
      } else if (setpara.set_element[i] == 6) {
        count++;
        break;
      }
    }
    for (int i = 1; i < count; i++) {
      if (Element[i] == 1)
        Element_rem.Element_data[count - i] = 2;
      else if (Element[i] == 2)
        Element_rem.Element_data[count - i] = 1;
      else
        Element_rem.Element_data[count - i] = Element[i];
    }
    Element_rem.Element_data[count] = 6;
  }
  Element_rem.loop_count = 0;
  Element_rem.loop_data[0] = setpara.loop_data[0];
  Element_rem.loop_data[1] = setpara.loop_data[1];
  Element_rem.loop_data[2] = setpara.loop_data[2];
  Element_rem.loop_data[3] = setpara.loop_data[3];
}
// 进入元素
void enter_element(Element_range element) { Element = element; }
// 离开元素
void out_element() {
  //    if(Element==Left_ring||Element==Right_ring)
  //    Element_rem.loop_count++;
  clear_all_flags();
  //    Element_rem.Element_count++;
}

void clear_all_flags() {
  // 清除元素标记
  Element = None;
  // 清除积分标志位
  clear_angle_integeral();
  clear_distant_integeral();
  // 清楚圆环标志
  watch.InLoop = 0;
  watch.InLoopAngleL = 120;
  watch.InLoopAngleR = 120;
  watch.InLoopCirc = 120;
  watch.InLoopAngle2 = 120;
  watch.OutLoop = 0;

  /*    watch.OutLoopRight = 0;
      watch.OutLoopRightY =120;
      watch.OutLoopLeft = 187;
      watch.OutLoopLeftY = 120;*/
  watch.OutLoopAngle2 = 120;
  watch.OutLoopAngle1 = 120;

  // 清除十字路口的变量
  watch.cross_flag = 0;
  watch.cross_line = 120;
  watch.Garge_line = 120;
  watch.cross_RD_angle = 120;
  watch.cross_LD_angle = 120;

  watch.cross_AngleL = 120;
  watch.cross_AngleR = 120;
  watch.cross_AngleL_x = 0;
  watch.cross_AngleR_x = 187;
  // 清除断路标志位
  watch.broken_circuit_flag = 0;
  // 清除斑马线标志位

  // flag.stop = 0;
  // indata.YawAngle = 0;
  // gpio_set(C10,0);
  // if(caminfo.apriltag_count<3)
  //     watch.AprilLine = 120 ;
  // 清除斜坡标志位
  watch.slope_flag = 0;

  // 恢复寻中线模式
  watch.Line_patrol_mode = 0;
  mycar.speed_ctrl = 1;                       // 恢复正常速度
  mycar.pid_ctrl = 1;                         // 恢复正常PID
  change_pid_para(&CAM_Turn, &CAM_FUZZY_PID); // 恢复正常转向PID
  change_pid_para(&Speed_middle, &setpara.com_speed_PID);
  change_pid_para(&Speed_left, &setpara.com_speed_PID);
  change_pid_para(&Speed_right, &setpara.com_speed_PID);
  // 清除车库标记位
  watch.zebra_flag = 0;
  watch.Zebra_Angle = 120;
  watch.Zebra_Angle2 = 120;
  watch.stop_count = 0;
  // 清除出库标志位
  watch.out_garage_flag = 0;
  // 清除停车标记位
  watch.garage_stop = 0;
  // 清除停车标记位
  mycar.car_stop = 0;
  watch.cross_flag = 0;

  watch.obstacle_flag = 0;
  watch.black_obstacle_flag = 0;
  watch.black_obstacle_line = 120;
  watch.left_obstacle_x = 0;
  watch.right_obstacle_x = 187;
  mycar.tracking_mode = 0; // 恢复摄像头循迹

  watch.angle_near_line = 30;
  watch.angle_far_line = setpara.far_line;
  watch.garage_flag = 0;
  mycar.target_speed = setpara.speed_min;
  cout << "setpara.speed_min" << setpara.speed_min;
  cout << "在清除元素标志位时赋值" << mycar.target_speed << endl;
}
