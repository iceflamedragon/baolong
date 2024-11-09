/*
 * black_obstacle.h
 *
 *  Created on: 2024��3��23��
 *      Author: hwj
 */

#ifndef CODE_CAMERA_PROCESS_BLACK_OBSTACLE_h_
#define CODE_CAMERA_PROCESS_BLACK_OBSTACLE_h_
#include "../global.h"
#ifdef __cplusplus
extern "C" {
 #endif 
void black_obstacle_enter();
void black_obstacle_out();
void black_obstacle_confirm();
void find_angle_left(int*angle_x,int*angle_y);

#ifdef __cplusplus
}
#endif 


#endif /* CODE_CAMERA_PROCESS_BLACK_OBSTACLE_H_ */
