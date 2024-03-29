#include "onMouse.h"
void onMouse(int event, int x, int y, int flags, void *param) // evnet:鼠标事件类型 x,y:鼠标坐标 flags：鼠标哪个键
{
    Mat *im = reinterpret_cast<Mat *>(param);
    switch (event)
    {

    case EVENT_LBUTTONDOWN:
        // 显示图像像素值

        if (static_cast<int>(im->channels()) == 1)
        {
            // 若图像为灰度图像，则显示鼠标点击的坐标以及灰度值
            cout << "at (" << x << ", " << y << " ) value is: " << static_cast<int>(im->at<uchar>(Point(x, y))) << endl;
        }
        else
        {
            // 若图像为彩色图像，则显示鼠标点击坐标以及对应的B, G, R值
            cout << "at (" << x << ", " << y << ")"
                 << "  B value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[0])
                 << "  G value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[1])
                 << "  R value is: " << static_cast<int>(im->at<Vec3b>(Point(x, y))[2])
                 << endl;
        }

        break;
    }
}
