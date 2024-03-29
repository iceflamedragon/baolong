/*
 * vofa.h
 *
 *  Created on: 2023Äê3ÔÂ6ÈÕ
 *      Author: Admin
 */

#ifndef CODE_ADJUST_VOFA_H_
#define CODE_ADJUST_VOFA_H_


#include "global.h"


int USARTPrintf(const char* format, ...);
void send_adjust_to_vofa(float*anjust);
void SendDataToVofa(float _a,float _b,float _c,float _d,float _e,float _f,float _g,float _h,float _i);
void vofa_uart_send_data(uint8*DataToSend, uint16 _cnt);
void Send_one_DataToVofa(float _a);
void Send_two_DataToVofa(float _a,float _b);
void Send_four_DataToVofa(float _a,float _b,float _c,float _d);
void Send_five_DataToVofa(float _a,float _b,float _c,float _d,float _e);

#endif /* CODE_ADJUST_VOFA_H_ */
