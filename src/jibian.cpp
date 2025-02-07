//
// Created by RUPC on 2022/10/14.
//
#include "main.h"
extern vector<vector<uint8_t>> array_img;
#define RESULT_ROW 160 // 结果图行列
#define RESULT_COL 120
#define USED_ROW 160 // 用于透视图的行列
#define USED_COL 120
#define PER_IMG array_img      // array:用于透视变换的图像 也可以使用二值化图
#define ImageUsed *PerImg_ip   //*PerImg_ip定义使用的图像，ImageUsed为用于巡线和识别的图像
typedef unsigned char uint8_t; // 无符号  8 bits
uint8_t *PerImg_ip[RESULT_ROW][RESULT_COL];

void ImagePerspective_Init(void)
{

    static uint8_t BlackColor = 0;
    double cameraMatrix[3][3] = {{296.482019, 0.000000, 152.664982}, {0.000000, 286.375269, 104.540031}, {0.000000, 0.000000, 1.000000}};
    double distCoeffs[5] = {-0.459946, 0.283675, 0.002304, 0.002566, -0.109265};
    int move_xy[2] = {0, 0};

    /* 0:USED_COL
     * 1:USED_ROW
     * 2:fx =K.at<double>(0, 0)
     * 3:fy = K.at<double>(1, 1)
     * 4: ux = K.at<double>(0, 2)
     * 5:  uy = K.at<double>(1, 2);
     * 6: k1 = .at<double>(0, 0),
     * 7:  k2 = D.at<double>(0, 1),
     * 8: k3=  D.at<double>(0, 4),
     * 9: p1 = D.at<double>(0, 2),
     * 10:   p2 = D.at<double>(0, 3);
     */

    /*********************Mat******************************/

    double fx = cameraMatrix[0][0], fy = cameraMatrix[1][1], ux = cameraMatrix[0][2], uy = cameraMatrix[1][2], k1 = distCoeffs[0], k2 = distCoeffs[1], k3 = distCoeffs[4], p1 = distCoeffs[2], p2 = distCoeffs[3];

    int move_x = move_xy[0], move_y = move_xy[1];

    for (int i = -move_y; i < RESULT_ROW - move_y; i++)
    {
        for (int j = -move_x; j < RESULT_COL - move_x; j++)
        {
            double xCorrected = (j - ux) / fx;
            double yCorrected = (i - uy) / fy;
            double xDistortion, yDistortion;
            double r2 = xCorrected * xCorrected + yCorrected * yCorrected;
            double deltaRa = 1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            double deltaRb = 1 / (1.);
            double deltaTx = 2. * p1 * xCorrected * yCorrected + p2 * (r2 + 2. * xCorrected * xCorrected);
            double deltaTy = p1 * (r2 + 2. * yCorrected * yCorrected) + 2. * p2 * xCorrected * yCorrected;
            xDistortion = xCorrected * deltaRa * deltaRb + deltaTx;
            yDistortion = yCorrected * deltaRa * deltaRb + deltaTy;
            xDistortion = xDistortion * fx + ux;
            yDistortion = yDistortion * fy + uy;
            if (i + move_y >= 0 && i + move_y < RESULT_ROW && j + move_x >= 0 && j + move_x < RESULT_COL)
            {
                if (yDistortion >= 0 && yDistortion < USED_ROW && xDistortion >= 0 && xDistortion < USED_COL)
                {
                    PerImg_ip[i + move_y][j + move_x] = &array_img[(int)yDistortion][(int)xDistortion];
                }
                else
                    PerImg_ip[i + move_y][j + move_x] = &BlackColor;
            }
        }
    }
}

/*ImageUsed[0][0]代表图像左上角的值*/

/*完成摄像头初始化后，调用一次ImagePerspective_Init，此后，直接调用ImageUsed   即为去畸变结果*/
