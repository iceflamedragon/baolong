/*
 * Binarization.c
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */
#include "Binarization.hpp"
#include "../control/mycar.hpp"
using namespace std;
// uint8_t Grayscale[120][188];////////////板卡上注释掉的
uint8_t img_threshold[12];
//struct caminfo_s caminfo;
//struct dir_control_struct cardir;
//struct watch_o watch;
//struct FLAG_STRUCT flag;

//大津法（祖传算法）
#define _GRAY_SCALE 64
#define _SHIFT_FOR_GRAYSCALE 2
#define _Thresh_Mult 2 //1/2*(256/_GRAY_SCALE)
//大津法，计算二值化阈值
int img_otsu(uint8_t *img, uint8_t img_v, uint8_t img_h, uint8_t step)
{
    uint8_t grayhist[_GRAY_SCALE] = {0}; //灰度直方图
    uint16_t px_sum_all = 0;             //像素点总数
    uint32_t gray_sum_all = 0;           //总灰度积分
    uint16_t px_sum = 0;                 //像素点数量
    uint32_t gray_sum = 0;               //灰度积分

    float fTemp_maxvar = 0;
    uint8_t temp_best_th = 0;
    uint8_t temp_best_th2 = 0;
    uint8_t temp_this_pixel = 0;
    float fCal_var;
    float u0, u1, w0, w1;

    //生成：1. 灰度直方图 2. 像素点总数 3. 总灰度积分
    for (int i = 0; i < img_v; i += step)
    {
        for (int j = 0; j < img_h; j += step)
        {
            temp_this_pixel = img[i * img_h + j] >> _SHIFT_FOR_GRAYSCALE;
            if (temp_this_pixel < _GRAY_SCALE)
                grayhist[temp_this_pixel]++;
            gray_sum_all += temp_this_pixel;
            px_sum_all++;
        }
    }
    //  //总平均灰度
    //  float u = 1.0*gray_sum_all/px_sum_all;
    //迭代求得最大类间方差的阈值
    for (uint8_t k = 0; k < _GRAY_SCALE; k++)
    {
        px_sum += grayhist[k];       //该灰度及以下的像素点数量
        gray_sum += k * grayhist[k]; //该灰度及以下的像素点的灰度和
        w0 = 1.0 * px_sum / px_sum_all;
        w1 = 1.0 - w0;
        if (px_sum > 0)
            u0 = 1.0 * gray_sum / px_sum;
        else
            u0 = 0.0;
        if (px_sum_all - px_sum > 0)
            u1 = 1.0 * (gray_sum_all - gray_sum) / (px_sum_all - px_sum);
        else
            u1 = 0.0;
        //fCal_var = w0*(u0-u)*(u0-u)+w1*(u1-u)*(u1-u);
        fCal_var = w0 * w1 * (u0 - u1) * (u0 - u1);
        if (fCal_var > fTemp_maxvar)
        {
            fTemp_maxvar = fCal_var;
            temp_best_th = k;
            temp_best_th2 = k;
        }
        else if (fCal_var == fTemp_maxvar)
        {
            temp_best_th2 = k;
        }
    }
    return (temp_best_th + temp_best_th2) * _Thresh_Mult;
}

////////////////////////////////////暂时注释掉可以修改为我们自己的参数后启用图形二值化
void Binarization()
{
    // int row=0,colum,area;
    // for(row=0;row<120;row++)
    // {
    //     for(colum=0;colum<MT9V03X_W;colum++)
    //     {
    //         if(mt9v03x_image[row][colum]<watch.threshold)
    //             Grayscale[row][colum]=0;
    //         else
    //             Grayscale[row][colum]=255;
    //     }
    // }
}

#define max_int_time_us  200  //曝光时间上限
uint16_t int_time_us;   //曝光时间
//int cam_times = 0;//大津法计数
uint8_t last_autogain=0;
#define OTSU_Hz 20  //每20帧进行一次大津法
#define expect_brightness 90 //期望亮度值
//大津法曝光调整
/*
void img_otsu_exposure_adjust()
{

    if (setpara.camcfg.manbri && !(watch.cam_times % OTSU_Hz)) //手动调节图像亮度
    {
        //mt9v03x_set_exposure_time(setpara.camcfg.exptime);
    }
    else if(indata.CPU1_Timecount<1000) //自动调整亮度—O21
    {
        watch.brightness = img_otsu((uint8_t *)mt9v03x_image, IMG_H, IMG_V, 10); //大津法调整曝光—O21
        int_time_us -= (abs(watch.brightness - expect_brightness) > 3 ? setpara.exp_ki * (watch.brightness - expect_brightness) : 0);
        int_time_us = _LIMIT(int_time_us, 1, max_int_time_us);
       // if(watch.brightness>90)
            mt9v03x_set_exposure_time(int_time_us);
    }
    else if(!(watch.cam_times % OTSU_Hz)) //自动调整亮度—O21
    {
        watch.brightness = img_otsu((uint8_t *)mt9v03x_image, IMG_H, IMG_V, 10); //大津法调整曝光—O21
        int_time_us -= (abs(watch.brightness - expect_brightness) > 3 ? setpara.exp_ki * (watch.brightness - expect_brightness) : 0);
        int_time_us = _LIMIT(int_time_us, 1, max_int_time_us);
       // if(watch.brightness>90)
            mt9v03x_set_exposure_time(int_time_us);
        watch.cam_times = 0;
    }
    sprintf(page[0].line9,"int_time_us=%d",int_time_us);
    if (last_autogain != setpara.camcfg.autogain)
    {
        last_autogain = setpara.camcfg.autogain;
        mt9v03x_set_reg( 0x35, setpara.camcfg.autogain);//设置摄像头增益
    }
}*/






