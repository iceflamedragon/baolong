#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void threshold_otsu(Mat &mat, Mat &mat_thresh,int num)    
//mat为输入图像，mat_thresh为二值化后的输出图像
{
	//求出图像的最大和最小像素值，确定阈值区间
	double min_value_mat, max_value_mat;
	Point min_position_mat, max_position_mat;
	minMaxLoc(mat, &min_value_mat, &max_value_mat, &min_position_mat, &max_position_mat);

	vector <double> var(max_value_mat - min_value_mat + 1);    /* 类间方差容器，不能用数组，因为数组要求用常量定义大小，而mat.rows和mat.cols是变量,除非给数组一个足够大的空间。mat.rows*mat.cols也不能少，否则会报错vector下标越界 */
	double thresh_value;   //二值化阈值
	int m = 0;    //m必须定义在第一层for函数外面，否则每次都会被初始化为0。
	for (thresh_value = min_value_mat; thresh_value < max_value_mat; thresh_value++)
	{
		double sum = mat.rows*mat.cols;     //图像像素点总数
		double sum_aim = 0, sum_bg = 0;     //目标和背景像素点总数
		double sum_vaule_aim = 0, sum_vaule_bg = 0;     //目标和背景像素点的总灰度
		for (int i = 0; i < mat.rows; i++)
			for (int j = 0; j < mat.cols; j++)
			{
				int vaule = mat.at<uchar>(i, j);
				if (vaule < thresh_value)     //背景
				{
					sum_bg += 1;
					sum_vaule_bg += vaule;
				}
				else       //目标
				{
					sum_aim += 1;
					sum_vaule_aim += vaule;
				}
			}
		double ratio_aim = sum_aim / sum;   //目标像素点所占比例
		double ratio_bg = sum_bg / sum;     //背景像素点所占比例
		double aver_vaule_aim = sum_vaule_aim / sum_aim;     //目标像素点的平均灰度
		double aver_vaule_bg = sum_vaule_bg / sum_bg;        //背景像素点的平均灰度
		double aver_vaule_mat = ratio_aim * aver_vaule_aim + ratio_bg * aver_vaule_bg;    //图像总平均灰度

		//计算每个阈值下的类间方差并保存到var中
		var[m] = ratio_aim * (aver_vaule_aim - aver_vaule_mat)*(aver_vaule_aim - aver_vaule_mat) +
			ratio_bg * (aver_vaule_bg - aver_vaule_mat)*(aver_vaule_bg - aver_vaule_mat);
		m += 1;
	}

	//找到最大类间方差以及其对应的阈值
	double var_max = 0, var_maxnum = 0;
	for (int k = 0; k < max_value_mat - min_value_mat; k++)
		if (var[k] > var_max)
		{
			var_max = var[k];
			var_maxnum = k + min_value_mat;
		}
	thresh_value = var_maxnum;

	//二值化，阈值以下的置为0，阈值以上的灰度值不变
	threshold(mat, mat_thresh, thresh_value+num, 255, THRESH_BINARY);   
}
