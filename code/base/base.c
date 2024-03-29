/*
 * base.c
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */
#include "base.h"
void hardwareinit()//Ӳ����ʼ��
{

    ips200_init(IPS200_TYPE_PARALLEL8);     //�ȳ�ʼ����Ļ�������������
//��ѹ���
    inductance_adc_init();
    beep_init();
//UART_Init
    //UART0��忨ͨ��
    uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);// ��ʼ������0 ������115200 ��������ʹ��P14_0 ��������ʹ��P14_1
//    uart_rx_interrupt(UART_0,1);          //ֻ��Ҫ���;����˲���Ҫ���ж�
//
//   UART3��vofaͨ��
    uart_init(UART_3,115200,UART3_TX_P20_0,UART3_RX_P20_3);// ��ʼ������0 ������115200 ��������ʹ��P14_0 ��������ʹ��P14_1
    uart_rx_interrupt(UART_3,1);
    system_delay_ms(1000);

    motor_init();

    encoder_init();

    GPIO_Init();

////��ʱ�ò��ˣ����ˣ�
//    mpu_init();

    GUI_Init();

    PID_init();
//PIT_Init
    pit_ms_init(CCU60_CH0,1);//��ʱ���ж�1(PWM)
    pit_ms_init(CCU61_CH0,1);//��ʱ���ж�3(key)


////    pit_ms_init(CCU60_CH0,10);
//    //pit_us_init(CCU60_CH1,800);//��ʱ���ж�2(current)
//    pit_ms_init(CCU61_CH1,5);//��ʱ���ж�4(Device)

    clear_all_flags();//���б��λ��ʼ��

}

void GPIO_Init(void)
{
    gpio_init(PRESS_PROT,GPI,0,GPI_PULL_UP);//press
    gpio_init(CW_PROT,GPI,0,GPI_PULL_UP);//cw
    gpio_init(CCW_PROT,GPI,0,GPI_PULL_UP);//ccw
    gpio_init(KEY_UP_PROT,GPI,0,GPI_PULL_UP);//up
    gpio_init(KEY_DOWN_PROT,GPI,0,GPI_PULL_UP);//down
    gpio_init(KEY_PUSH_PROT,GPI,0,GPI_PULL_UP);//push

}

void clear_all_flags(void)
{
    //������ֱ�־λ
    clear_angle_integeral();
    clear_distant_integeral();

    mycar.speed_ctrl=1;                         //�ָ������ٶ�
     change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//�ָ�����ת��PID
     change_pid_para(&Speed_middle,&setpara.com_speed_PID);
//     change_pid_para(&Speed_left,&setpara.com_speed_PID);
//     change_pid_para(&Speed_right,&setpara.com_speed_PID);
    //���ͣ�����λ
    mycar.car_stop=0;
    mycar.car_running=0;

}

