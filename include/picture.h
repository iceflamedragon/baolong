#pragma once
#include "main.h"
#include <vector>
using namespace std;
void threshold_otsu(cv::Mat &mat, cv::Mat &mat_thresh, int num);

vector<vector<uchar>> decode(cv::Mat img);
