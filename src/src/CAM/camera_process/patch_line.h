/*
 * patch_line.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_PATCH_LINE_h_
#define CODE_CAMERA_PROCESS_PATCH_LINE_h_
#ifdef __cplusplus
extern "C" {
#endif
#include "../global.h"
void linefix();
void left_ring_linefix();
void right_ring_linefix();
void outgarage_linefix();
void ingarage_linefix_1();
void ingarage_linefix_2();
void crossing_linefix();
void common_linefix();
void obstacle_linefix();
void left_garage_linefix();
void black_obstacle_linefix();

#ifdef __cplusplus
}
#endif 
#endif /* CODE_CAMERA_PROCESS_PATCH_LINE_H_ */
