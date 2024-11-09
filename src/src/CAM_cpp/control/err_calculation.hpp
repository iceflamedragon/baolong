/*
 * err_calculation.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CONTROL_ERR_CALCULATION_HPP_
#define CODE_CONTROL_ERR_CALCULATION_HPP_
#include "../global.hpp"
float cam_err_calculation();
void original_err_calculation();
extern float cam_original_err;
int Steer_PWM_Cal(int track_pos) ;

#endif /* CODE_CONTROL_ERR_CALCULATION_H_ */
