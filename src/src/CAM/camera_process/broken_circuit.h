/*
 * broken_circuit.h
 *
 *  Created on: 2023��7��11��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_BROKEN_CIRCUIT_h_
#define CODE_CAMERA_PROCESS_BROKEN_CIRCUIT_h_
#include "../global.h"
#ifdef __cplusplus
extern "C" {
 #endif 
void broken_circuit_prepare();
void broken_circuit_slow();
void broken_circuit_enter();
void broken_circuit_complete_enter();
void broken_circuit_complete_out();

#ifdef __cplusplus
}
#endif 

#endif /* CODE_CAMERA_PROCESS_BROKEN_CIRCUIT_H_ */
