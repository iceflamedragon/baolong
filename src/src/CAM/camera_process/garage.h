/*
 * garage.h
 *
 *  Created on: 2023��6��22��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_GARAGE_H_
#define CODE_CAMERA_PROCESS_GARAGE_H_
#include "../global.h"

#ifdef __cplusplus
extern "C" {
 #endif 
void garage_stop();
void out_garage();
void garage_prepare();
void left_garage_first_angle();
void left_garage_confirm();
void left_garage_second_angle();
void left_garage_begin_turn();
void left_garage_stop_turn();
void garage_stop2();
void right_garage_first_angle();
void right_garage_second_angle();
void right_garage_confirm();
void right_garage_begin_turn();
void right_garage_stop_turn();
/*void left_zebra_identification();
void right_zebra_identification();
void left_zebra_confirm();
void right_zebra_confirm();
void left_garage_first_angle();
void left_garage_second_angle();
void left_garage_second_turn();
void right_garage_first_angle();
void right_garage_second_turn();
void garage_stop();
void out_garage();*/
#ifdef __cplusplus
}
#endif 
#endif /* CODE_CAMERA_PROCESS_GARAGE_H_ */
