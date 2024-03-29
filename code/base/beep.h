/*
 * beep.h
 *
 *  Created on: 2023Äê3ÔÂ14ÈÕ
 *      Author: Admin
 */

#ifndef CODE_BASE_BEEP_H_
#define CODE_BASE_BEEP_H_
#include "base.h"

#define beep_port   ATOM3_CH0_P33_10

extern uint16 beep_time;
void beep_init();
void beeping();
void beep(uint16 time);
void beep2(uint8 yinjie,uint16 time);
void beep_warning();
void warning();
void car_start_warning();
#endif /* CODE_BASE_BEEP_H_ */
