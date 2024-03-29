#pragma once
#include "main.h"
#define BLACK 0
#define line_white 255
#define L_search_amount 110 // 左右边界搜点时最多允许的点
#define R_search_amount 110
#define line_detect_front 100
// void To_Array(int a[picture_long][picture_width],Mat mat);
void search_neighborhood(cv::Mat img);
void show_edge(cv::Mat img);
void show_middle_line(cv::Mat img);
void ten_word_detect(cv::Mat img);
void search_front_neighborhood(cv::Mat img);
void draw_boundary_line(cv::Mat img);
void detect_round(cv::Mat img);
void deviation_direction(void);