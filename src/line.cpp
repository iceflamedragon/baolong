#include "line.h"
#include "main.h"

// 获取 mat 的行和列
int row = picture_long;
int col = picture_width;
int left_guai_hou_point_x, right_guai_hou_point_x, left_guai_qian_point_x, right_guai_qian_point_x;
int left_guai_hou_point_y, right_guai_hou_point_y, left_guai_qian_point_y, right_guai_qian_point_y; // 靠车近的是后
int L_start_width, L_start_long, R_start_width, R_start_long;
int L_start_front_long, L_start_front_width, R_start_front_long, R_start_front_width;
int L_front_edge_count, R_front_edge_count;
int left_guai_use_x, left_guai_use_y, right_guai_use_x, right_guai_use_y;

int right_guai_hou_flag = 0;
int left_guai_hou_flag = 0;
int lost_left_flag = 0;
int lost_right_flag = 0;
int left_guai_qian_flag = 0, right_guai_qian_flag = 0;
int left_front_findflag = 0, right_front_findflag = 0;
extern int left_findflag;
extern int right_findflag;
int four_guai_flag = 0;
// 八邻域画边界
struct LEFT_EDGE
{
    int16_t row;  // 行坐标
    int16_t col;  // 列坐标
    uint8_t flag; // 存在边界的标志
};
struct RIGHT_EDGE
{
    int16_t row;  // 行坐标
    int16_t col;  // 列坐标
    uint8_t flag; // 存在边界的标志
};
struct LEFT_front_EDGE
{
    int16_t row;  // 行坐标
    int16_t col;  // 列坐标
    uint8_t flag; // 存在边界的标志
};
struct RIGHT_front_EDGE
{
    int16_t row;  // 行坐标
    int16_t col;  // 列坐标
    uint8_t flag; // 存在边界的标志
};
struct LEFT_EDGE R_edge[L_search_amount];              // 左边界结构体
struct RIGHT_EDGE L_edge[R_search_amount];             // 右边界结构体
struct LEFT_front_EDGE R_front_edge[L_search_amount];  // 左边界结构体
struct RIGHT_front_EDGE L_front_edge[R_search_amount]; // 右边界结构体
uint8_t R_edge_count = 0, L_edge_count = 0;            // 左右边点的个数
uint8_t dire_left, dire_right;                         // 记录上一个点的相对位置

uint16_t middle_line[140][2] = {0};

// 将四周涂黑,不需要
void draw_boundary_line(Mat img)
{
    for (int i = 1; i < 159; i++)
    {
        img.at<uchar>(i, 0) = 0;
        img.at<uchar>(i, 119) = 0;
    }
    for (int j = 1; j < 119; j++)
    {
        img.at<uchar>(0, j) = 0;
        img.at<uchar>(159, j) = 0;
    }
}
void search_neighborhood(Mat img)
{
    R_edge_count = 0; // 左边点个数清0
    L_edge_count = 0; // 右边点个数清0

    for (int j = 130; j > 60; j--)
    {
        for (int i = 2; i < 80; i++) // 找左边起始点
        {

            if (img.at<uchar>(j, i + 1) == BLACK)
                if (img.at<uchar>(j, i + 2) == line_white)

                {
                    left_findflag = 1;
                    L_start_long = j;
                    L_start_width = i + 2;
                    // cout << j << " " << i + 1 << endl;
                    break;
                }
        }
        if (left_findflag == 1)
            break;
    }

    for (int j = 130; j > 60; j--)
    {
        for (int i = 120; i > 30; i--) // 找右边起始点
        {
            if (img.at<uchar>(j, i - 1) == BLACK)
                if (img.at<uchar>(j, i - 2) == line_white)

                {
                    right_findflag = 1;
                    R_start_long = j;
                    R_start_width = i - 2;
                    // cout << j << " " << i - 1 << endl;
                    break;
                }
        }
        if (right_findflag == 1)
            break;
    }
    if (left_findflag) // 如果左边界点存在并找到,则开始爬线
    {

        // 变量声明
        R_edge[0].row = L_start_long;
        R_edge[0].col = L_start_width;
        R_edge[0].flag = 1;
        int16_t curr_row = L_start_long;  // 初始化行坐标
        int16_t curr_col = L_start_width; // 初始化列坐标
        dire_left = 0;                    // 初始化上个边界点的来向
        // 开始搜线，最多取150个点，不会往下搜，共7个方位
        for (int i = 1; i < L_search_amount; i++) // 最多搜索150个点
        {
            // 越界退出 行越界和列越界（向上向下向左向右）,opencv的矩阵超过上限不会报错，必须要加入限定条件
            if (curr_row >= picture_width - 1 || curr_col >= picture_long - 1 || curr_col < 1)
                break;

            // 搜线过程
            if (dire_left != 2 && img.at<uchar>(curr_row - 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col) == line_white) // 左上黑，2，右边白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 7;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_left != 3 && img.at<uchar>(curr_row - 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row, curr_col + 1) == line_white) // 右上黑，3，下边白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 6;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (img.at<uchar>(curr_row - 1, curr_col) == BLACK && img.at<uchar>(curr_row - 1, curr_col + 1) == line_white) // 正上黑，1，右白
            {
                curr_row = curr_row - 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 0;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_left != 5 && img.at<uchar>(curr_row, curr_col - 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col - 1) == line_white) // 正左黑，5，上白
            {
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 4;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_left != 4 && img.at<uchar>(curr_row, curr_col + 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col + 1) == line_white) // 正右黑，4，下白
            {
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 5;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_left != 6 && img.at<uchar>(curr_row + 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row, curr_col - 1) == line_white) // 左下黑，6，上白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 3;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else if (dire_left != 7 && img.at<uchar>(curr_row + 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col) == line_white) // 右下黑，7，左白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                R_edge_count = R_edge_count + 1;
                dire_left = 2;
                R_edge[i].row = curr_row;
                R_edge[i].col = curr_col;
                R_edge[i].flag = 1;
            }
            else
                break;
        }
    }

    if (right_findflag) // 如果右边界存在并搜到
    {
        L_edge[0].row = R_start_long;
        L_edge[0].col = R_start_width;
        L_edge[0].flag = 1;
        int16_t curr_row = R_start_long;
        int16_t curr_col = R_start_width;
        dire_right = 0;
        for (int i = 1; i < R_search_amount; i++)
        {
            if (curr_row >= picture_width - 1 || curr_col >= picture_long - 1 || curr_col < 1)
                break;
            if (curr_col < picture_width && dire_right != 3 && img.at<uchar>(curr_row - 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col) == line_white) // 右上黑，3，左白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 6;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_right != 2 && img.at<uchar>(curr_row - 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row, curr_col - 1) == line_white) // 左上黑，2，下白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 7;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (img.at<uchar>(curr_row - 1, curr_col) == BLACK && img.at<uchar>(curr_row - 1, curr_col - 1) == line_white) // 正上黑，1，左白
            {
                curr_row = curr_row - 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 0;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_right != 4 && img.at<uchar>(curr_row, curr_col + 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col + 1) == line_white) // 正右黑，4，上白
            {
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 5;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_right != 5 && img.at<uchar>(curr_row, curr_col - 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col - 1) == line_white) // 正左黑，5，下白
            {
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 4;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }

            else if (dire_right != 6 && img.at<uchar>(curr_row + 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col) == line_white) // 左下黑，6，右白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 3;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else if (dire_right != 7 && img.at<uchar>(curr_row + 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row, curr_col + 1) == line_white) // 右下黑，7，上白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                L_edge_count = L_edge_count + 1;
                dire_right = 2;
                L_edge[i].row = curr_row;
                L_edge[i].col = curr_col;
                L_edge[i].flag = 1;
            }
            else
                break;
        }
    }

    // 丢线处理
    // if(left_findflag==0&&right_findflag==1)
    // {
    // 	for(int i=0;i<140;i++)
    // 	{
    // 	R_edge[i].row=L_edge[i].row;
    // 	R_edge[i].col=L_edge[i].col-40;
    // 	R_edge[i].flag=1;

    // 	}
    // }
    // if(right_findflag==0&&left_findflag==1)
    // {
    // 	for(int i=0;i<140;i++)
    // 	{
    // 	L_edge[i].row=R_edge[i].row;
    // 	L_edge[i].col=R_edge[i].col+40;
    // 	L_edge[i].flag=1;
    // 	}
    // }
}

// 左边线丢失检测
void Lost_line_left(void)
{
    int i;
    for (i = line_detect_front; i < line_detect_front + 10; i++)
        if (L_edge[i].flag == 0)
            lost_left_flag += 1;
    // cout << "lost_left_flag:" << lost_left_flag << endl;
}

// 右边线丢线检测
void Lost_line_right(void)
{
    int i;
    for (i = line_detect_front; i < line_detect_front + 10; i++)
        if (R_edge[i].flag == 0)
            lost_right_flag += 1;
    // cout << "lost_right_flag:" << lost_right_flag << endl;
}

// 画出赛道
void show_edge(Mat img)
{
    for (int i = 0; i < L_search_amount; i++)
    {
        img.at<uchar>(R_edge[i].row, R_edge[i].col) = 130;

        img.at<uchar>(L_edge[i].row, L_edge[i].col) = 130;
    }
    // for(int i=0;i<L_search_amount;i++)//打出赛道的点
    // {
    //     cout<<i<<" "<<R_edge[i].row<<endl;
    // }
}

// 画出中线
void show_middle_line(Mat img)
{
    for (int i = 0; i < L_search_amount; i++)
    {
        if (R_edge[i].flag >= 1 && L_edge[i].flag >= 1)
        {
            // img.at<uchar>((R_edge[i].row + L_edge[i].row) / 2, (R_edge[i].col + L_edge[i].col) / 2) = 130;
            middle_line[i][0] = (R_edge[i].row + L_edge[i].row) / 2;
            middle_line[i][1] = (R_edge[i].col + L_edge[i].col) / 2;
            img.at<uchar>(middle_line[i][0], middle_line[i][1]) = 130;
        }
    }
}

// 十字丢线后找出上面的边线
void search_front_neighborhood(Mat img)
{
    R_front_edge_count = 0; // 左边点个数清0
    L_front_edge_count = 0; // 右边点个数清0

    for (int j = 70; j > 3; j--)
    {

        if (img.at<uchar>(j, 2) == line_white)
            if (img.at<uchar>(j - 1, 2) == BLACK)

            {
                left_front_findflag = 1;
                L_start_front_long = j;
                L_start_front_width = 2;
                // cout << "zuodian" << L_start_front_long <<" "<< L_start_front_width<<endl;
                break;
            }
        if (left_front_findflag == 1)
            break;
    }

    for (int j = 70; j > 3; j--)
    {

        if (img.at<uchar>(j, 118) == line_white)

            if (img.at<uchar>(j - 1, 118) == BLACK)

            {
                right_front_findflag = 1;
                R_start_front_long = j;
                R_start_front_width = 118;
                // cout << "youdian" << R_start_front_long << " "<<R_start_front_width<<endl;
                break;
            }
        if (right_front_findflag == 1)
            break;
    }
    if (right_front_findflag) // 如果左边界点存在并找到,则开始爬线
    {

        // 变量声明
        R_front_edge[0].row = L_start_front_long;
        R_front_edge[0].col = L_start_front_width;
        R_front_edge[0].flag = 1;
        int16_t curr_row = L_start_front_long;  // 初始化行坐标
        int16_t curr_col = L_start_front_width; // 初始化列坐标
        dire_left = 0;                          // 初始化上个边界点的来向
        // 开始搜线，最多取150个点，不会往下搜，共7个方位
        for (int i = 1; i < L_search_amount; i++) // 最多搜索150个点
        {
            ////越界退出 行越界和列越界（向上向下向左向右）
            if (curr_row >= picture_width - 1 || curr_col >= picture_long + 1 || curr_col < 1)
                break;
            // 搜线过程
            if (dire_left != 2 && img.at<uchar>(curr_row - 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col) == line_white) // 左上黑，2，右边白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 7;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (dire_left != 3 && img.at<uchar>(curr_row - 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row, curr_col + 1) == line_white) // 右上黑，3，下边白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 6;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (img.at<uchar>(curr_row - 1, curr_col) == BLACK && img.at<uchar>(curr_row - 1, curr_col + 1) == line_white) // 正上黑，1，右白
            {
                curr_row = curr_row - 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 0;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (dire_left != 5 && img.at<uchar>(curr_row, curr_col - 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col - 1) == line_white) // 正左黑，5，上白
            {
                curr_col = curr_col - 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 4;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (dire_left != 4 && img.at<uchar>(curr_row, curr_col + 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col + 1) == line_white) // 正右黑，4，下白
            {
                curr_col = curr_col + 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 5;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (dire_left != 6 && img.at<uchar>(curr_row + 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row, curr_col - 1) == line_white) // 左下黑，6，上白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 3;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else if (dire_left != 7 && img.at<uchar>(curr_row + 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col) == line_white) // 右下黑，7，左白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                R_front_edge_count = R_front_edge_count + 1;
                dire_left = 2;
                R_front_edge[i].row = curr_row;
                R_front_edge[i].col = curr_col;
                R_front_edge[i].flag = 1;
            }
            else
                break;
        }
    }

    if (left_front_findflag) // 如果右边界存在并搜到
    {
        L_front_edge[0].row = R_start_front_long;
        L_front_edge[0].col = R_start_front_width;
        L_front_edge[0].flag = 1;
        int16_t curr_row = R_start_front_long;
        int16_t curr_col = R_start_front_width;
        dire_right = 0;
        for (int i = 1; i < R_search_amount; i++)
        {
            ////越界退出 行越界和列越界（向上向下向左向右）
            if (curr_row >= picture_width - 1 || curr_col >= picture_long - 1 || curr_col < -1)
                break;
            // 爬线过程
            if (curr_col < picture_width && dire_right != 3 && img.at<uchar>(curr_row - 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col) == line_white) // 右上黑，3，左白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col + 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 6;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else if (dire_right != 2 && img.at<uchar>(curr_row - 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row, curr_col - 1) == line_white) // 左上黑，2，下白
            {
                curr_row = curr_row - 1;
                curr_col = curr_col - 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 7;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else if (img.at<uchar>(curr_row - 1, curr_col) == BLACK && img.at<uchar>(curr_row - 1, curr_col - 1) == line_white) // 正上黑，1，左白
            {
                curr_row = curr_row - 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 0;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else if (dire_right != 4 && img.at<uchar>(curr_row, curr_col + 1) == BLACK && img.at<uchar>(curr_row - 1, curr_col + 1) == line_white) // 正右黑，4，上白
            {
                curr_col = curr_col + 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 5;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else if (dire_right != 5 && img.at<uchar>(curr_row, curr_col - 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col - 1) == line_white) // 正左黑，5，下白
            {
                curr_col = curr_col - 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 4;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }

            else if (dire_right != 6 && img.at<uchar>(curr_row + 1, curr_col - 1) == BLACK && img.at<uchar>(curr_row + 1, curr_col) == line_white) // 左下黑，6，右白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col - 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 3;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else if (dire_right != 7 && img.at<uchar>(curr_row + 1, curr_col + 1) == BLACK && img.at<uchar>(curr_row, curr_col + 1) == line_white) // 右下黑，7，上白
            {
                curr_row = curr_row + 1;
                curr_col = curr_col + 1;
                L_front_edge_count = L_front_edge_count + 1;
                dire_right = 2;
                L_front_edge[i].row = curr_row;
                L_front_edge[i].col = curr_col;
                L_front_edge[i].flag = 1;
            }
            else
                break;
        }
    }

    // 丢线处理
    // if(left_findflag==0&&right_findflag==1)
    // {
    // 	for(int i=0;i<140;i++)
    // 	{
    // 	R_front_edge[i].row=L_front_edge[i].row;
    // 	R_front_edge[i].col=L_front_edge[i].col-40;
    // 	R_front_edge[i].flag=1;

    // 	}
    // }
    // if(right_findflag==0&&left_findflag==1)
    // {
    // 	for(int i=0;i<140;i++)
    // 	{
    // 	L_front_edge[i].row=R_front_edge[i].row;
    // 	L_front_edge[i].col=R_front_edge[i].col+40;
    // 	L_front_edge[i].flag=1;
    // 	}
    // }
}

void show_front_edge(Mat img)
{
    for (int i = 0; i < L_search_amount; i++)
    {
        img.at<uchar>(R_front_edge[i].row, R_front_edge[i].col) = 130;
        img.at<uchar>(L_front_edge[i].row, L_front_edge[i].col) = 130;
        // cout<<"xian"<<L_front_edge[i].row<<" "<<L_front_edge[i].col<<endl;
    }

    // cout << "画完边线" << endl;
}

void draw_Left_Lines(Mat img)
{
    Point p1 = Point(left_guai_qian_point_y, left_guai_qian_point_x);
    Point p2 = Point(left_guai_hou_point_y, left_guai_hou_point_x);

    line(img, p2, p1, 0, 2, cv::LINE_8); // 在背景图绘画直线
}

void draw_right_Lines(Mat img)
{
    Point p1 = Point(right_guai_qian_point_y, right_guai_qian_point_x);
    Point p2 = Point(right_guai_hou_point_y, right_guai_hou_point_x);
    // cout<<"dian"<<right_guai_qian_point_y<<" "<< right_guai_qian_point_x<<endl;

    // img.at<uchar>(right_guai_qian_point_x, right_guai_qian_point_y) = 155;
    line(img, p1, p2, 0, 2, cv::LINE_8); // 在背景图绘画直线
}

// 十字检测
void ten_word_detect(Mat img)
{
    Lost_line_left();
    Lost_line_right();
    if (lost_left_flag + lost_right_flag >= 7) // 左右丢线
    {
        cout << "检测到左右丢线" << endl;

        // 检测离车远的两个拐点
        // cout << "画上面边线" << endl;
        search_front_neighborhood(img);
        // show_front_edge(img);
        for (int i = 60; i > 15; i -= 2)
        { // 斜率为0是直线上升，线正数为向右上方向，左线斜率由正转负为拐点，右线斜率由负变正为拐点
            if (((double)(R_front_edge[i - 4].row - R_front_edge[i].row) / (double)(R_front_edge[i].col - R_front_edge[i - 4].col) - (double)(R_front_edge[i - 3].row - R_front_edge[i - 7].row) / (double)(R_front_edge[i - 3].col - R_front_edge[i - 7].col)) < 2)

            {
                // cout<<i<<" "<<((double)(R_front_edge[i - 4].row - R_front_edge[i].row) / (double)(R_front_edge[i].col - R_front_edge[i - 4].col) -(double)(R_front_edge[i - 3].row - R_front_edge[i - 7].row) / (double)(R_front_edge[i - 3].col - R_front_edge[i - 7].col) )<<endl;
                right_guai_qian_point_x = R_front_edge[i - 3].row;
                right_guai_qian_point_y = R_front_edge[i - 3].col;
                img.at<uchar>(right_guai_qian_point_x, right_guai_qian_point_y) = 155;
                right_guai_qian_flag = 1;
                right_guai_use_x = R_front_edge[i - 5].row;
                right_guai_use_y = R_front_edge[i - 5].col;
                // cout << "右" << right_guai_qian_point_x << " " << right_guai_qian_point_y << endl;
                cout << "找到左前拐点" << endl;
                break;
            }
        }
        for (int i = 60; i > 15; i -= 2)
        {
            if ((double)(L_front_edge[i - 4].row - L_front_edge[i].row) / (double)(L_front_edge[i - 4].col - L_front_edge[i].col) - (double)(L_front_edge[i - 7].row - L_front_edge[i - 3].row) / (double)(L_front_edge[i - 7].col - L_front_edge[i - 3].col) > 3)
            //(double)(L_front_edge[i - 4].row - L_front_edge[i].row) / (double)(L_front_edge[i - 4].col - L_front_edge[i].col) - (double)(L_front_edge[i - 7].row - L_front_edge[i - 3].row) / (double)(L_front_edge[i - 7].col - L_front_edge[i - 3].col) > 3
            {
                cout << i << " " << (double)(L_front_edge[i - 4].row - L_front_edge[i].row) / (double)(L_front_edge[i - 4].col - L_front_edge[i].col) - (double)(L_front_edge[i - 7].row - L_front_edge[i - 3].row) / (double)(L_front_edge[i - 7].col - L_front_edge[i - 3].col) << endl;
                left_guai_qian_point_x = L_front_edge[i - 3].row;
                left_guai_qian_point_y = L_front_edge[i - 3].col;
                // img.at<uchar>(left_guai_qian_point_x, left_guai_qian_point_y) = 155;
                left_guai_qian_flag = 1;
                cout << "找到右前拐点" << endl;
                // break;
                cout << "左" << left_guai_qian_point_x << " " << left_guai_qian_point_y << endl;
            }
        }

        if (left_guai_qian_flag == 1 && right_guai_qian_flag == 1)
        {
            // 找到靠近车的两个拐点
            for (int i = 40; i > 10; i -= 2)
            { // 斜率为0是直线上升，线正数为向右上方向，左线斜率由正转负为拐点，右线斜率由负变正为拐点
                if ((double)(R_edge[i - 6].row - R_edge[i].row) / (double)(R_edge[i - 6].col - R_edge[i].col) - (double)(R_edge[i - 9].row - R_edge[i - 3].row) / (double)(R_edge[i - 9].col - R_edge[i - 3].col) < -5)
                {
                    // cout << i << " " << (double)(R_edge[i - 6].row - R_edge[i].row) / (double)(R_edge[i - 6].col - R_edge[i].col) - (double)(R_edge[i - 9].row - R_edge[i - 3].row) / (double)(R_edge[i - 9].col - R_edge[i - 3].col) << endl;
                    right_guai_hou_point_x = R_edge[i].row;
                    right_guai_hou_point_y = R_edge[i].col;
                    // img.at<uchar>(right_guai_hou_point_x, right_guai_hou_point_y) = 155;
                    right_guai_hou_flag = 1;
                    cout << "找到右后拐点" << endl;
                    // cout << "右点" << right_guai_hou_point_x << " " << right_guai_hou_point_y << endl;
                    break;
                }
            }
            for (int i = 40; i > 10; i -= 2)
            {
                if ((L_edge[i - 6].row - L_edge[i].row) / (double)(L_edge[i - 6].col - L_edge[i].col) - (double)(L_edge[i - 9].row - L_edge[i - 3].row) / (double)(L_edge[i - 9].col - L_edge[i - 3].col) < -5)
                {
                    // cout << i << " " << (L_edge[i - 6].row - L_edge[i].row) / (double)(L_edge[i - 6].col - L_edge[i].col) - (double)(L_edge[i - 9].row - L_edge[i - 3].row) / (double)(L_edge[i - 9].col - L_edge[i - 3].col) << endl;
                    left_guai_hou_point_x = L_edge[i - 4].row;
                    left_guai_hou_point_y = L_edge[i - 4].col;
                    // img.at<uchar>(left_guai_hou_point_x, left_guai_hou_point_y) = 155;
                    left_guai_hou_flag = 1;
                    cout << "找到左后拐点" << endl;
                    break;
                    // cout << "左点" << left_guai_hou_point_x << " " << left_guai_hou_point_y << endl;
                }
            }
        }
        // 完整找到四个拐点
        if (left_guai_hou_flag == 1 && right_guai_hou_flag == 1)
        {

            cout << "检测到四个拐点" << endl;
            draw_Left_Lines(img);
            draw_right_Lines(img);
            search_neighborhood(img);
            show_edge(img);
            show_middle_line(img);
            four_guai_flag = 1;
        }

        // 找到三个拐点
        if (left_guai_hou_flag == 1 || right_guai_hou_flag == 1)
        {
            // 暂时使用这个条件
            if (four_guai_flag != 1)
            {
                cout << "检测到三个拐点" << endl;
                if (left_guai_hou_flag == 1)
                { // 右边的点
                    Point p1 = Point(left_guai_qian_point_y, left_guai_qian_point_x);
                    Point p2 = Point(left_guai_hou_point_y, left_guai_hou_point_x);

                    line(img, p2, p1, 0, 2, cv::LINE_8); // 在背景图绘画直线
                                                         // Point p3 = Point(right_guai_hou_point_y, right_guai_hou_point_x);
                                                         // Point p4 = Point(right_guai_use_y, right_guai_use_x);
                                                         // line(img, p3, p4, 0, 2, cv::LINE_8); // 在背景图绘画直线

                    cout << right_guai_qian_point_x << "," << right_guai_qian_point_y << endl;
                    cout << right_guai_use_x << "," << right_guai_use_y << endl;
                    double k = (double)(right_guai_use_y - right_guai_qian_point_y) / (double)(right_guai_use_x - right_guai_qian_point_x);
                    double b = right_guai_qian_point_x - right_guai_qian_point_y * k;
                    int y = b;
                    cout << k << "," << y << "," << b << endl;

                    Point p7(right_guai_qian_point_y, right_guai_qian_point_x);
                    Point p8(0, y);
                    line(img, p7, p8, 155, 2, cv::LINE_8); // 在背景图绘画直线
                }

                // 只写了左边缺失点的情况，右边缺点需要再写
                //  if (right_guai_hou_flag == 1)
                //  {
                //      Point p5 = Point(right_guai_qian_point_y, right_guai_qian_point_x);
                //      Point p6 = Point(right_guai_hou_point_y, right_guai_hou_point_x);
                //      // cout<<"dian"<<right_guai_qian_point_y<<" "<< right_guai_qian_point_x<<endl;

                //     // img.at<uchar>(right_guai_qian_point_x, right_guai_qian_point_y) = 155;
                //     line(img, p5, p6, 0, 2, cv::LINE_8); // 在背景图绘画直线
                //     double k=(left_guai_use_y-left_guai_hou_point_y)/(left_guai_use_x-left_guai_hou_point_x);
                //     double b=left_guai_use_y-left_guai_use_x*k;
                //     int y=150*k+b;
                //     Point p7(left_guai_hou_point_y,left_guai_hou_point_x)  ;
                //     Point p8(y,150)  ;
                //     line(img, p7, p8, 0, 2, cv::LINE_8); // 在背景图绘画直线
                // }
            }
        }
    }
}

// 圆环检测
void detect_round(Mat img)
{
    int straight_flag = 0, round_flag = 0; // 圆环的标志物

    for (int i = 60; i > 0; i -= 2)
    { // 斜率为0是直线上升，线正数为向右上方向，左线斜率由正转负为拐点，右线斜率由负变正为拐点
        if ((double)(R_edge[i - 6].row - R_edge[i].row) / (double)(R_edge[i - 6].col - R_edge[i].col) - (double)(R_edge[i - 9].row - R_edge[i - 3].row) / (double)(R_edge[i - 9].col - R_edge[i - 3].col) < -10)
        {
            // cout << i << " " << (double)(R_edge[i - 6].row - R_edge[i].row) / (double)(R_edge[i - 6].col - R_edge[i].col) - (double)(R_edge[i - 9].row - R_edge[i - 3].row) / (double)(R_edge[i - 9].col - R_edge[i - 3].col) << endl;
            right_guai_hou_point_x = R_edge[i].row;
            right_guai_hou_point_y = R_edge[i].col;
            img.at<uchar>(right_guai_hou_point_x, right_guai_hou_point_y) = 155;
            right_guai_hou_flag = 1;
            // cout << "找到左后拐点" << endl;
            //  cout << "右点" << right_guai_hou_point_x << " " << right_guai_hou_point_y << endl;
            break;
        }
    }
    for (int i = 10; i < L_search_amount - 40; i += 2)
    {
        if ((double)(L_edge[i - 6].row - L_edge[i].row) / (double)(L_edge[i - 6].col - L_edge[i].col) - (double)(L_edge[i - 9].row - L_edge[i - 3].row) / (double)(L_edge[i - 9].col - L_edge[i - 3].col) < 5 && (double)(L_edge[i - 6].row - L_edge[i].row) / (double)(L_edge[i - 6].col - L_edge[i].col) - (double)(L_edge[i - 9].row - L_edge[i - 3].row) / (double)(L_edge[i - 9].col - L_edge[i - 3].col) > -5)
            straight_flag++;
    }
    if (straight_flag < 25)
        cout << "直线不够直" << endl;
    else
    {
        round_flag = 1;
        search_front_neighborhood(img);
        show_front_edge(img);
        for (int i = 60; i > 15; i -= 2)
        { // 斜率为0是直线上升，线正数为向右上方向，左线斜率由正转负为拐点，右线斜率由负变正为拐点
            if (((double)(R_front_edge[i - 4].row - R_front_edge[i].row) / (double)(R_front_edge[i].col - R_front_edge[i - 4].col) - (double)(R_front_edge[i - 3].row - R_front_edge[i - 7].row) / (double)(R_front_edge[i - 3].col - R_front_edge[i - 7].col)) < 2)

            {
                // cout<<i<<" "<<((double)(R_front_edge[i - 4].row - R_front_edge[i].row) / (double)(R_front_edge[i].col - R_front_edge[i - 4].col) -(double)(R_front_edge[i - 3].row - R_front_edge[i - 7].row) / (double)(R_front_edge[i - 3].col - R_front_edge[i - 7].col) )<<endl;
                right_guai_qian_point_x = R_front_edge[i - 3].row;
                right_guai_qian_point_y = R_front_edge[i - 3].col;
                img.at<uchar>(right_guai_qian_point_x, right_guai_qian_point_y) = 155;
                right_guai_qian_flag = 1;
                right_guai_use_x = R_front_edge[i - 5].row;
                right_guai_use_y = R_front_edge[i - 5].col;
                // cout << "右" << right_guai_qian_point_x << " " << right_guai_qian_point_y << endl;
                cout << "找到左前拐点" << endl;
                break;
            }
        }
    }
}

// 250是最右，1250是最左，750是中间
void deviation_direction(void)
{
    // 车辆的位置
    int car_y = 130;
    int sum_short = 0;
    int sum_middle = 0;
    int sum_long = 0;

    for (int i = 0; i < 20; i++)
        sum_short += middle_line[i][1];
    for (int i = 20; i < 40; i++)
        sum_middle += middle_line[i][1];
    for (int i = 40; i < 60; i++)
        sum_long += middle_line[i][1];
    cout << "短" << sum_short / 20 << "中" << sum_middle / 20 << "长" << sum_long / 20 << endl;
}
void deviation_speed()
{
}

// 标志位清除
void destroy_flag(void)
{
    left_findflag = 0;
    right_findflag = 0;
}