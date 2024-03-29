/*
 * integeral.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CONTROL_INTEGRAL_H_
#define CODE_CONTROL_INTEGRAL_H_
#include "global.h"
typedef struct
{
   uint8 integeral_flag; //���ֱ��λ 0:δ��ʼ���� 1�����ڻ��� 2���������
   float integeral_data; //��������
   float integeral_Thres; //������ֵ
   //float last_data;      //�ϴ�����

}integeral_STRUCT;
extern integeral_STRUCT angle_integral,distance_integral;
extern uint8 get_integeral_state(integeral_STRUCT* integeral);
extern float get_integeral_data(integeral_STRUCT* integeral);
void begin_angle_integeral(int16 target_angle);
void angal_integeral(float gyro_z);
void begin_distant_integeral(int target_distant);
void distant_integeral(float present_speed);
void clear_angle_integeral(void);
void clear_distant_integeral(void);




#endif /* CODE_CONTROL_INTEGRAL_H_ */
