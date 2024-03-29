#include "main.h"
#include <vector>
#include "picture.h"
#include "opencv2/opencv.hpp"
#include "xbox.h"
#include "serial.h"
#include "onMouse.h"
using namespace cv;
using namespace std;
using namespace LibSerial;
int left_findflag;
int right_findflag;
Mat after_img;
// 可能有错

extern struct LEFT_EDGE L_edge[L_search_amount];  // 左边界结构体
extern struct RIGHT_EDGE R_edge[R_search_amount]; // 右边界结构体
SerialPort serial_port;
union serial_direction
{
	float diao;
	unsigned char send_byte[4]; // 从250到1250
};
union serial_speed
{
	signed short int diao;
	unsigned char send_byte[2]; // 从0到10000
	unsigned char flag;
};
int main()
{
	// 十字的四个点
	std::vector<cv::Point2f> srcPoints;
	srcPoints.push_back(cv::Point2f(31, 40));
	srcPoints.push_back(cv::Point2f(13, 96));
	srcPoints.push_back(cv::Point2f(103, 97));
	srcPoints.push_back(cv::Point2f(89, 39));

	// 定义目标图像上的四个点
	std::vector<cv::Point2f> dstPoints;
	dstPoints.push_back(cv::Point2f(40, 60));
	dstPoints.push_back(cv::Point2f(40, 100));
	dstPoints.push_back(cv::Point2f(80, 100));
	dstPoints.push_back(cv::Point2f(80, 60));
	SerialPort serial_port;

	// 去畸变矩阵
	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<double>(0, 0) = 1.007342167202611e+02;
	cameraMatrix.at<double>(0, 1) = 0;
	cameraMatrix.at<double>(0, 2) = 59.308994060397220;
	cameraMatrix.at<double>(1, 1) = 9.968651209953272e+02;
	cameraMatrix.at<double>(1, 2) = 83.785984650404220;
	cameraMatrix.at<double>(2, 2) = 1;
	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	distCoeffs.at<double>(0, 0) = -0.148004626720803;	  // k1
	distCoeffs.at<double>(1, 0) = 0.103454832683536;	  // k2
	distCoeffs.at<double>(2, 0) = 2.339424306625256e-04;  // p1
	distCoeffs.at<double>(3, 0) = -9.569429486376358e-04; // p2
	distCoeffs.at<double>(4, 0) = 0;					  // k3
	union serial_speed niuniu_speed;
	union serial_direction niuniu_direction;
	// xbox的参数
	int xbox_fd;
	xbox_map_t map;
	int len, type;
	int axis_value, button_value;
	int number_of_axis, number_of_buttons;
	// xbox初始化
	memset(&map, 0, sizeof(xbox_map_t));
	xbox_fd = xbox_open("/dev/input/js0");
	if (xbox_fd < 0)
	{
		cout << "xbox手柄未找到" << endl;
	}

	// 串口初始化
	serial_init();

	// std::string write_string_1 = "BAOLONG";
	// char write_byte_1 = 'd';
	// for (int i = 0; i < 10;i++)
	// 	serial_port.Write(write_string_1);

	// 计算逆透视变换矩阵
	cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

	Mat view, rview, map1, map2;
	Mat img_2;
	Mat img;
	Size dsize = Size(picture_long, picture_width);
	int yuzhi = 0;
	// std::cout << "serial_port read:   " << read_byte_1 << std::endl ;
	string path = "../pic/qipan";
	int s1 = 1;
	string s2 = ".jpg";

	// img = imread("../pic/qipan2.jpg");

	// 打开摄像头
	VideoCapture cap(0);
	cap.read(img);
	niuniu_speed.diao = 666.666;
	niuniu_direction.diao = 300;

	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
							getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, dsize, 1, dsize, 0),
							dsize, CV_16SC2, map1, map2);

	// pid的调节及发送
	// 	  CAM_Turn.kp_ratio=0.0001;
	// CAM_Turn.ki_ratio=0.001;
	// CAM_Turn.kd_ratio=1;
	// CAM_Turn.integral_Limit=150;
	// CAM_Turn.total_limit=350;
	// CAM_Turn.PID_para=&setpara.com_turn_PID;
	// CAM_Turn.d_limit=0;
	//     	setpara.com_turn_PID;
	int kp = 63; // 转弯PID
	int kd = 0;
	// 		serial_port.WriteByte((unsigned char)(0xAA));

	// serial_port.WriteByte((unsigned char)(kp));

	// 		serial_port.WriteByte((unsigned char)(kd));
	// 				serial_port.WriteByte((unsigned char)(1));
	// 		serial_port.WriteByte((unsigned char)(0xDD));
	int i = 0;
	int g = 1;

	// 摄像头参数设置
	cap.set(CAP_PROP_FRAME_WIDTH, 160);	 // 宽度
	cap.set(CAP_PROP_FRAME_HEIGHT, 120); // 高度
	// cap.set(CAP_PROP_CONVERT_RGB, 1);
	// cap.set(CAP_PROP_FORMAT, CV_8UC1);
	cap.set(6, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
	// cap.set(CAP_PROP_BUFFERSIZE, 1);
	while (true)
	{
		clock_t start_time = clock();

		cap.read(img);

		// double fps = cap.get(cv::CAP_PROP_FPS); // 帧率
		//  cout << "帧率: " << fps << endl;
		//       int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);	 // 视频帧宽度
		//       int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT); // 视频帧高度

		//     Mat img = imread("../yuanhuan/yuanhuan10.png");
		if (img.data == nullptr) // nullptr是c++11新出现的空指针常量
		{
			cout << "图片文件不存在" << endl;
		}
		// // 裁剪图像
		// resize(img, img, dsize, 0, 0, INTER_AREA);
		// // img_2没有经过去畸变
		// // 矩阵变换（此处是去畸变）

		// remap(img, img_2, map1, map2, INTER_LINEAR);
		// cvtColor(img, img, COLOR_BGR2GRAY);
		// // 测量运行时间

		// threshold_otsu(img, img, 25); // 大津法

		// //   黑色是0，白色是255
		// Mat perspectiveMatrix = cv::getPerspectiveTransform(srcPoints, dstPoints);

		// // 逆透视
		// cv::warpPerspective(img, img, perspectiveMatrix, dsize);

		// Mat element = getStructuringElement(MORPH_RECT, Size(7, 7)); // 小于8*8方块的白色噪点都会被腐蚀
		// erode(img, img, element);
		// draw_boundary_line(img);
		// search_neighborhood(img);
		// show_edge(img);
		// show_middle_line(img);
		// deviation_direction();
		// ten_word_detect(img);
		// //   detect_round( img);

		imshow("Image", img);
		//  // 获取鼠标坐标
		//  // setMouseCallback("Image", onMouse, reinterpret_cast<void *>(&img));
		//  imshow("img_12", img_2);
		if (waitKey(1) == 27)
		{ // 如果用户按下 ESC 键，退出循环

			s1++;
			imwrite(path + to_string(s1) + s2, img);
			cout << path + to_string(s1) + s2 << endl;
			niuniu_speed.flag = 0;
		}
		// 等待按键

		len = xbox_map_read(xbox_fd, &map);
		if (len < 0)
		{
			usleep(10 * 1000);
			continue;
		}

		// printf("\rTime:%8d A:%d B:%d X:%d Y:%d LB:%d RB:%d start:%d back:%d home:%d LO:%d RO:%d XX:%-6d YY:%-6d LX:%-6d LY:%-6d RX:%-6d RY:%-6d LT:%-6d RT:%-6d",map.time, map.a, map.b, map.x, map.y, map.lb, map.rb, map.start, map.back, map.home, map.lo, map.ro,map.xx, map.yy, map.lx, map.ly, map.rx, map.ry, map.lt, map.rt);
		fflush(stdout);

		// while (1)
		// {
		// 速度是int16，转向是float

		cout << "手柄" << (float)map.lx / 32767 * 1000 + 250 << " " << -((double)map.ly / 32767 * 1000) + 1000 << " " << map.a << endl;
		niuniu_direction.diao = (float)map.lx / 32767 * 1000 + 250;
		niuniu_speed.diao = -(signed short int)((float)map.ly / 32767 * 1000) + 1000;
		if (map.a == 0)
			niuniu_speed.flag = 1;
		else if (map.a == 1)
			niuniu_speed.flag = 0;

		// serial_port.WriteByte((unsigned char)(0x0A));
		// serial_port.WriteByte((unsigned char)(niuniu_direction.send_byte[0]));
		// serial_port.WriteByte((unsigned char)(niuniu_direction.send_byte[1]));
		// serial_port.WriteByte((unsigned char)(niuniu_direction.send_byte[2]));
		// serial_port.WriteByte((unsigned char)(niuniu_direction.send_byte[3]));
		// // 方向
		// serial_port.WriteByte((unsigned char)(niuniu_speed.send_byte[0]));
		// serial_port.WriteByte((unsigned char)(niuniu_speed.send_byte[1]));

		// serial_port.WriteByte((unsigned char)(niuniu_speed.flag));
		// serial_port.WriteByte((unsigned char)(0x0D));

		// }
		clock_t end_time = clock();
		cout << "The run time is: " << (double)(end_time - start_time) / CLOCKS_PER_SEC << "s" << endl;
	}
	serial_port.Close();
	xbox_close(xbox_fd);
	// cap.release();			 // 释放摄像头
	cv::destroyAllWindows(); // 取消窗口
	return 0;
	// diao
}
