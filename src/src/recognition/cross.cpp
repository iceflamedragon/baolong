/*
 * cross.c
 *
 *  Created on: 2023年7月9日
 *      Author: Admin
 */
#include "cross.h"
// 十字进入函数
void cross_enter() {
  uint8 cross_count = 0;
  for (int y = 20; y < 70; y++) {
    if (lineinfo[y].left_lost && lineinfo[y].right_lost) {
      cross_count++;
    }
  }
  if (cross_count > 40 && watch.cross_flag == 0) {
    clear_all_flags();
    enter_element(crossing); // 进入十字元素
    watch.angle_far_line = 90;
    watch.cross_flag = 1;
    begin_distant_integeral(6000);
  }
}
// 十字离开函数
void cross_out() {
  if (get_integeral_state(&distance_integral) == 2 && watch.cross_flag == 1) {
    clear_distant_integeral();
    out_element();
  }
}
