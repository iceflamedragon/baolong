#pragma once
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <picture.h>
#include <string.h>
#include <line.h>

#include "main.h"
#include "picture.h"
#include "line.h"
#include <SerialPort.h>

#include <cstdlib>
#include <fstream>
#include <unistd.h>

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <stdlib.h>
#include <thread>

using namespace std;
using namespace cv;

#define picture_width 160
#define picture_long 120
typedef unsigned char uint8_t;       // 给无符号char，取别名为uint8_t
typedef unsigned short int uint16_t; // 给无符号短整型short int，取别名为uint16_t
typedef unsigned int uint32_t;       // 给无符号整型short int，取别名为uint32_t
typedef signed char int8_t;          // 给有符号char，取别名为int8_t
typedef signed short int int16_t;    // 给有符号短整型short int，取别名int16_t
typedef signed int int32_t;          // 给有符号整型short int，取别名int32_t
