/*
 * inductance_adc.h
 *
 *  Created on: 2023年3月19日
 *      Author: Admin
 */

#ifndef CODE_BASE_INDUCTANCE_ADC_H_
#define CODE_BASE_INDUCTANCE_ADC_H_
#include "base.h"
void inductance_adc_init();
void get_ad_value();
void get_battery_value();

struct adc_struct
{
    float original_data[8];
    float data[8];
    float Battery;          //电池电压
};
extern struct adc_struct adc;
#endif /* CODE_BASE_INDUCTANCE_ADC_H_ */
