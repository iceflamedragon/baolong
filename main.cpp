#include "main.h"
#include "picture.h"
#include "line.h"
using namespace cv;
using namespace std;
int left_findflag;
int right_findflag;

extern struct LEFT_EDGE L_edge[L_search_amount];  // 左边界结构体
extern struct RIGHT_EDGE R_edge[R_search_amount]; // 右边界结构体
int main()
{
	// int picture[picture_long][picture_width];
	//VideoCapture cap(0);
	Mat img;
	Size dsize = Size(picture_long, picture_width);
	int yuzhi = 0;
	while (true)
	{

		// cap.read(img);
		// double fps = cap.get(cv::CAP_PROP_FPS); // 帧率
		//  int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);	 // 视频帧宽度
		//  int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT); // 视频帧高度
		// std::cout << "帧率: " << fps << std::endl;

		Mat img = imread("../images/tumble129.jpg");

		if (img.data == nullptr) // nullptr是c++11新出现的空指针常量
		{
			cout << "图片文件不存在" << endl;
		}
		resize(img, img, dsize, 0, 0, INTER_AREA);
		cvtColor(img, img, COLOR_BGR2GRAY);

		Mat after_img;
		threshold_otsu(img, after_img, 0); // 大津法
		// 黑色是0，白色是255

		Mat element = getStructuringElement(MORPH_RECT, Size(8, 8)); // 小于8*8方块的白色噪点都会被腐蚀
		erode(after_img, img, element);

		search_neighborhood(img);
		//show_edge(img);
		show_middle_line(img);
		// To_Array(picture[picture_long][picture_width],img);图像矩阵转成数组

		// 		printf("%u\n", img.at<uchar>(i, j));//打印像素点
		// 左上角图像原点，向下i(row)增大，向右j(col)增大
		//  for(int i=20;i<40;i++)
		//  {
		//  	for(int j=0;j<20;j++)
		//  	img.at<uchar>(i,j)=255;
		//  }

		ten_word_detect(img);
		imshow("Image", img);
waitKey(0);
		if (waitKey(1) == 27)
		{ // 如果用户按下 ESC 键，退出循环
			break;
		}
	}
	// cap.release();//释放摄像头
	cv::destroyAllWindows(); // 取消窗口
	return 0;
}
