/*
 * adc.c
 *
 *  Created on: 2023年3月19日
 *      Author: Admin
 */
#include "inductance_adc.h"
struct adc_struct adc;

void inductance_adc_init()
{
    adc_init(ADC0_CH4_A4,ADC_12BIT);

}
uint8 Battery_warning=0;
void get_battery_value()
{

//   adc.Battery= adc_mean_filter_convert(ADC0_CH4_A4,10)*442/(2442)*11.8/430;
       adc.Battery= adc_mean_filter_convert(ADC0_CH4_A4,10)/4095*3.3*4;
       if(adc.Battery>7&&adc.Battery<11.5&&!Battery_warning)
       {
           beep(5000);//电压监控，如果小于11.5V,报警5s
           Battery_warning=1;
       }beeping();
}


//冒泡排序，对检测到的数值进行排序
void bubbleSort(unsigned short int array[], int n)
{
  int i, j, temp;
  for (i = 0; i < n-1; i++)
  {
    for (j = 0; j < n-i-1; j++)
    {
      if (array[j] > array[j+1])
      {
        temp = array[j];
        array[j] = array[j+1];
        array[j+1] = temp;
      }
    }
  }
}
#define AVE_TIMES 15
int mid_i = (AVE_TIMES-3); // Third max
unsigned short int raw_ad[3][AVE_TIMES];
void get_ad_value()
{
  int i;
  for (i = 0; i < AVE_TIMES; i++)
  {
    raw_ad[0][i] = adc_convert(ADC1_CH5_A21);   //LH
    raw_ad[1][i] = adc_convert(ADC1_CH4_A20); //MH
    raw_ad[2][i] = adc_convert(ADC1_CH0_A16);   //RH
  }
  for (i = 0; i < 3; i++)
  {
    bubbleSort(raw_ad[i], AVE_TIMES);
  }

}

