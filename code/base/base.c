/*
 * base.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "base.h"
void hardwareinit()//硬件初始化
{

    ips200_init(IPS200_TYPE_PARALLEL8);     //先初始化屏幕，方便输出错误
//电压监控
    inductance_adc_init();
    beep_init();
//UART_Init
    //UART0与板卡通信
    uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);// 初始化串口0 波特率115200 发送引脚使用P14_0 接收引脚使用P14_1
//    uart_rx_interrupt(UART_0,1);          //只需要发送就行了不需要开中断
//
//   UART3与vofa通信
    uart_init(UART_3,115200,UART3_TX_P20_0,UART3_RX_P20_3);// 初始化串口0 波特率115200 发送引脚使用P14_0 接收引脚使用P14_1
    uart_rx_interrupt(UART_3,1);
    system_delay_ms(1000);

    motor_init();

    encoder_init();

    GPIO_Init();

////暂时用不了，坏了？
//    mpu_init();

    GUI_Init();

    PID_init();
//PIT_Init
    pit_ms_init(CCU60_CH0,1);//定时器中断1(PWM)
    pit_ms_init(CCU61_CH0,1);//定时器中断3(key)


////    pit_ms_init(CCU60_CH0,10);
//    //pit_us_init(CCU60_CH1,800);//定时器中断2(current)
//    pit_ms_init(CCU61_CH1,5);//定时器中断4(Device)

    clear_all_flags();//所有标记位初始化

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
    //清除积分标志位
    clear_angle_integeral();
    clear_distant_integeral();

    mycar.speed_ctrl=1;                         //恢复正常速度
     change_pid_para(&CAM_Turn,&setpara.com_turn_PID);//恢复正常转向PID
     change_pid_para(&Speed_middle,&setpara.com_speed_PID);
//     change_pid_para(&Speed_left,&setpara.com_speed_PID);
//     change_pid_para(&Speed_right,&setpara.com_speed_PID);
    //清除停车标记位
    mycar.car_stop=0;
    mycar.car_running=0;

}

