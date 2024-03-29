/*
 * mycar.h
 *
 *  Created on: 2023��6��21��
 *      Author: Admin
 */

#ifndef CODE_CONTROL_MYCAR_H_
#define CODE_CONTROL_MYCAR_H_
#include "global.h"


//С��״̬��־λ
typedef struct mycar_STRUCT{
        uint8 status;   //С������״̬
        uint32 RUNTIME; //С���˶�ʱ��(��λΪ1ms)
        uint32 TIME;    //С������ʱ��(��λΪ1ms)
        uint32 senddata;
/*
        u32 save_times;
        u32 savenum;
        u32 save_img_count;

        u32 WAITTIME;//�����ȴ�ʱ��
        uint8 out_garage;//�������ǣ�����ʱˢ��
        uint8 in_garage;//�����
*/
        uint8 car_running; //С������״̬ 1������  0��ֹͣ
        uint8 car_stop;    //ɲ�����λ����1ʱɲ��
        float present_speed;  ////��ǰ��ȡ���������ٶ�
        float present_speed2;
        float present_physical_speed;//��ǰ�����ٶ�

        float   left_speed,         //�������ȡ���ٶ�(��������������)
                right_speed,        //�Ҳ�����ȡ���ٶ�(��������������)
//


                original_err,      //ԭʼƫ��ֵ
                target_gyro,   //Ŀ����ٶ�
                gyro_err,     //���ٶ�ƫ��
                Speed_err,  //���������ת����ٶȲ�
                current_left,//���ٶȻ�PID����ĵ���ֵ
                current_right;//���ٶȻ�PID����ĵ���ֵ
        float target_left_speed;
        float target_right_speed;

        int16   speed,          //�����ȡ���ٶ�(��������������)

                left_pwm_set,      //�����Ҫ�趨��PWM
                right_pwm_set,      //�Ҳ���Ҫ�趨��PWM

                pwm_set,        //ֻ��һ������趨��pwm

                speed_PWM;          //���ٶȻ�PID�����ֵ

        int16   speed_Middle_PWM, //�ٶȻ������PWM��ֵ
                Turn_PWM;         //���������PWM��ֵ
        int16 steer_pwm;
        float target_speed;  //С����ǰĿ���ٶȣ�����
        uint8 CircCount;     //С����ǰȦ��
        uint8 speed_ctrl;     //���õ�ǰС���ٶ�(ƫ��Խ���ٶ�Խ��)
        uint8 tracking_mode;  //ѭ����ʽ����1ʱ�õ��ѭ��

        float speed_differ;  //�����ֲ��٣��ٷֱȣ�

}mycar_STRUCT;

extern mycar_STRUCT mycar;
void motor_control(void);
void dir_control(void);
void dir_pid_adjust();
void speed_pid_adjust();
void common_running();
void running_protect();
void car_begin();
void car_stop();
void dir_control();
void speed_ctrl();
float speed_differ_set(int16 steer_pwm);
void set_speed(uint16 speed);
#endif /* CODE_CONTROL_MYCAR_H_ */
