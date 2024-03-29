/*
 * adjust.c
 *
 *  Created on: 2023年6月23日
 *      Author: Admin
 */
#include "adjust.h"
#pragma section all "cpu1_dsram"
struct adjust_struct adjust;
struct vofa_struct vofa;
float adjust_data[8];
float line_data[120][9];
IfxCpu_mutexLock core_original_err=0;


extern uint8 buff;
extern uint8 receive_data[30];
extern int16 enc_count;
//vofa上传数据
void vofa_display(float *data)
{
    SendDataToVofa(data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8]);
}

//CPU1调试代码
void CPU0_adjust()
{

    vofa_display(vofa.turn);
//    set_steer(setpara.SteerMid);
    if(mycar.car_running==0)
    {
//        第零页：基本数据
        sprintf(page[0].line5,"run_time=%d ",mycar.RUNTIME);
        sprintf(page[0].line4,"time=%d",mycar.TIME);
        sprintf(page[0].line6,"Turn.PID_out_P=%d",CAM_Turn.PID_out_P); //2表示存进去了
        sprintf(page[0].line7,"Turn.PID_out_D=%d",CAM_Turn.PID_out_D);
        sprintf(page[0].line8,"PIDoutput=%d",CAM_Turn.PID_output);
//        sprintf(page[0].line3,"buff=%d",buff);
//        sprintf(page[0].line5,"left_speed=%.1f",mycar.left_speed);
        sprintf(page[0].line1,"car_running=%d",mycar.car_running);
        sprintf(page[0].line2,"present_v=%.3f",mycar.present_speed);
        sprintf(page[0].line3,"physical_v=%.4f  ",mycar.present_speed2);

        sprintf(page[0].line9,"battery=%.3f  ",adc.Battery);

        sprintf(page[0].line10,"control_v=%.5f  ",setpara.control_speed);
       sprintf(page[0].line11,"steer=%d",mycar.steer_pwm);

////查看串口接受的数据，未解码
//        sprintf(page[0].line6,"receive_data=%d ",receive_data[0]);
//        sprintf(page[0].line7,"receive_data=%d ",receive_data[1]);
//        sprintf(page[0].line8,"receive_data=%d  ",receive_data[2]);
//        sprintf(page[0].line9,"receive_data=%d",receive_data[3]);
//        sprintf(page[0].line10,"receive_data=%d ", receive_data[4]);
//        sprintf(page[0].line11,"receive_data=%d ", receive_data[5]);
//        sprintf(page[0].line12,"receive_data=%d ",receive_data[6]);
//        sprintf(page[0].line13,"receive_data=%d  ",receive_data[7]);
//        sprintf(page[0].line14,"receive_data=%d",receive_data[8]);
//        sprintf(page[0].line14,"enc_count=%d",enc_count);
     //第二页：入库标记


        GUI_Display();
    }
//    else
//    {
//        system_delay_ms(5);//延迟函数，防止vofa卡死
//    }
//    }
    vofa.turn[0]=mycar.steer_pwm;
    vofa.turn[1]=setpara.control_speed;
//    vofa.turn[2]=CAM_Turn.PID_out_P;
//    vofa.turn[3]=CAM_Turn.PID_out_D;
//    vofa.turn[4]=CAM_Turn.PID_output;
    vofa.turn[2]=Speed_middle.PID_out_P;
    vofa.turn[3]=Speed_middle.PID_out_D;
    vofa.turn[4]=Speed_middle.PID_output;
    vofa.turn[5]=mycar.TIME;
    vofa.turn[6]=mycar.present_speed;
    vofa.turn[7]=mycar.present_speed2;
    vofa.turn[8]=(233);


//
//    //vofa.loop[5]=Element;
//    vofa.adc_err[0]=adc.data[0];
//    vofa.adc_err[1]=adc.data[1];
//    vofa.adc_err[2]=adc.data[2];
//    vofa.adc_err[3]=s.h_ave;
//    vofa.adc_err[4]=s.near_raw;
//    vofa.adc_err[5]=s.near_result;
//    vofa.adc_err[6]=s.far_distance;
//    vofa.adc_err[7]=s.track_position;

    vofa.speed[0]=mycar.left_speed;
    vofa.speed[1]=mycar.right_speed;
    vofa.speed[2]=mycar.present_speed;
    vofa.speed[3]=mycar.left_pwm_set;
    vofa.speed[4]=mycar.right_pwm_set;
    vofa.speed[5]=mycar.present_speed2;
    vofa.speed[6]=mycar.RUNTIME;
    vofa.speed[7]=mycar.target_speed+mycar.Speed_err;
    vofa.speed[8]=mycar.target_speed-mycar.Speed_err;

}

//CPU0调试代码
void CPU1_adjust()
{

}
//定时器1调试代码
void timer1_adjust()
{


/*    vofa.turn[0]=mycar.left_speed;
    vofa.turn[1]=mycar.right_speed;
    vofa.turn[2]=mycar.original_err;
    vofa.turn[3]=mycar.gyro_err;
    vofa.turn[4]=mycar.Speed_err;
    vofa.turn[5]=mycar.present_speed;
    vofa.turn[6]=imu.gyroz;
    vofa.turn[7]=mycar.target_speed+mycar.Speed_err;
    vofa.turn[8]=mycar.target_speed-mycar.Speed_err;*/

}
//定时器4调试代码
void timer4_adjust()
{


    //sprintf(page[0].line4,"imu.pitch=%.2f  ",imu.pitch);
}
//以数组的形式记录左右边线，目的是上传上位机
void edge_line(uint8 y,uint8 xl,uint8 xr)
{
    adjust.left_line[119-y]=xl;
    adjust.right_line[119-y]=xr;
}

#pragma section all restore
