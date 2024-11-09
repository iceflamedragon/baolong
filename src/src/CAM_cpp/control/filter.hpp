/*
 * filter.h
 *
 *  Created on: 2024��4��18��
 *      Author: 86138
 */

#ifndef CODE_CONTROL_FILTER_HPP_
#define CODE_CONTROL_FILTER_HPP_
#include "../global.hpp"

typedef struct filter_handle_s {
  float b[3];
  float a[3];
  float x[3];
  float y[3];
  unsigned char order;
  unsigned char is_inited;
} filter_handle_s;

float filter_calc(filter_handle_s *fh, float val);

void filter_calc_butterworth_2nd_lowpass_coeff(filter_handle_s *fh, float cutoff_freq, float sample_rate);



#endif /* CODE_CONTROL_FILTER_H_ */
