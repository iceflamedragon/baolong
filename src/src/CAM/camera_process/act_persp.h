/*
 * actpersp.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_ACT_PERSP_h_
#define CODE_CAMERA_PROCESS_ACT_PERSP_h_
#include "../global.h"
#ifdef __cplusplus
extern "C" {
 #endif 
struct persp_struct
{
      uint8_t persp_x[188];
      int16_t y;
};
void act_perst_init();
void persp_task(int16_t xl,int16_t xr,int16_t y);
int16_t get_persp_data_ox(int16_t x,int16_t y);
int16_t get_persp_data_oy(int16_t x,int16_t y);


#ifdef __cplusplus
}
#endif 

#endif /* CODE_CAMERA_PROCESS_ACT_PERSP_H_ */
