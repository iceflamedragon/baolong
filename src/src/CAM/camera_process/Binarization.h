/*
 * Binarization.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_BINARIZATION_H_
#define CODE_CAMERA_PROCESS_BINARIZATION_H_
#include "../global.h"

#ifdef __cplusplus
extern "C" {
 #endif 
extern uint8_t Grayscale[120][188];
int img_otsu(uint8_t *img, uint8_t img_v, uint8_t img_h, uint8_t step);
void Binarization();
void img_otsu_exposure_adjust();

#ifdef __cplusplus
}
#endif 


#endif /* CODE_CAMERA_PROCESS_BINARIZATION_H_ */
