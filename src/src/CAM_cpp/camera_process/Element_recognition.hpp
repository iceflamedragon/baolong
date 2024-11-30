/*
 * Element_recognition.h
 *
 *  Created on: 2023年6月21日
 *      Author: Admin
 */

#ifndef CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#define CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_
#include "../global.hpp"
// #include "Binarization.hpp"
// #include "scan_line.hpp"
// #include "patch_line.hpp"
// #include "act_persp.hpp"
// #include "garage.hpp"
// #include "circle.hpp"
// #include "cross.hpp"
// #include "slope.hpp"
// #include "broken_circuit.hpp"
// #include "black_obstacle.hpp"
using namespace std;
typedef enum
{
    None=0,         //无元素      0
    Left_ring,      //左环        1
    Right_ring,     //右环        2
    Slope,          //斜坡        3
    broken_circuit, //断路        4
    obstacle,       //障碍        5
    ingarage,       //入库        6
    outgarage,      //出库        7
    crossing,        //十字        8
    black_obstacle,  //小型黑色路障  9
    zebra           //斑马线     10

}Element_range;
struct Element_struct
{
     uint8_t Element_count;//元素计数
     uint8_t Element_data[21];
     uint8_t loop_count;//圆环计数
     uint8_t loop_data[4];//圆环数据
};
struct watch_o
{
    //int base_line;

    int cam_times;//大津法计数
    uint8_t brightness; //大津法所得图像亮度
    uint8_t threshold;  //图像二值化阈值


    int watch_line;
    int watch_lost;//摄像头所能看到赛道的最远端
    int loop_flag;

    int cpu0_fps;
    int cpu1_fps;
    short gray;

    //统计丢线相关变量  使用位置：void count_lost(void)
    int cross;  //统计丢线算法中，左边线与右边线都丢线的行数
    int left_lost; //统计丢线算法中，左边线丢线的行数
    int right_lost;//统计丢线算法中，右边线丢线的行数
    int left_near_lost;//统计丢线算法中,左边线开始丢线的行数
    int right_near_lost;//统计丢线算法中,右边线开始丢线的行数

    //直线标志相关变量      使用位置：void Straight_check(void);
    int RLStraight;//左右直线 0：无直线 1左边线是直线，右边线不是直线 2：右边线是直线左边线不是 3：左右边线都是直线

    /*圆环状态0:无环
    1：检测到左环第一个角点，此时补左侧第一条线保持直行
    2：正在入环，此时右侧补线入环
    3：从陀螺仪积分一定值，此时完全入环
    4：陀螺仪积分完成，准备出环
    5：出圆环后直行 
    6：检测到右环第一个角点,此时补左侧第一条线保持直行
    7：正在入环，此时左侧补线入环
    8：从陀螺仪积分一定值，此时完全入环
    9：陀螺仪积分完成，准备出环
    10：出圆环后直行*/
    int InLoop;
    //出环标记变量 1:出环时进入直道与圆环交界处
    int OutLoop;

    //圆环标志位
    int InLoopAngleL;  //入左环前直行的第一个角所在行（直道与圆环交接的角点）
    int InLoopAngleR;  //入右环前直行的第一个角所在行（直道与圆环交接的角点）
    int InLoopCirc;   //圆环上凸弧
    int InLoopAngle2; //开始转向入环时前方的角点所在行（直道与圆环交接的角点）
    int InLoopAngle2_x;
    int InLoopAngle2_y;//开始转向入环时前方的角点所在列
    int OutLoopAngle2; //出环后直行时前方的角点所在行（直道与圆环交接的角点）
    int OutLoopAngle1; //出环时边上的角点（出左环时在右侧，出右环时在左侧）
    int OutLoop_turn_point_x;//转向点横坐标，根据该点进行补线
    //int OutLoopRight;  //出左环左转时右前方角点横坐标
    //int OutLoopRightY; //出左环左转时右前方角点纵坐标
    //int OutLoopLeft;   //出右环右转时左前方角点横坐标
    //int OutLoopLeftY;  //出右环右转时左前方角点纵坐标
    int OutLoop2;
    int dis_Loop;
    int top_x;//赛道左右两条直线交汇处的横坐标值（此时y=115）用于补线
    float fix_slope;//补线斜率
    //说明：在透视图，任意两条平行线交汇于用一点，根据实验得出结论，交汇的行数近似固定为115行(即此行为地平线)，根据此来进行补线




    //是否进入三岔路标志——21区
    int Junc_flag; //0否1左2右3进入但不知道方向
    int DeltaR;    //暂时用作显示
    int DeltaL;    //暂时用作显示

    int JuncLineL; //三岔路角点左所在行
    int JuncLineR; //三岔路角点右所在行
    int JuncLine;  //取平均
    int JuncAngle;
    int JuncAngleX;  //取平均
    int JuncTime;  //三岔路时间
    int junc_linefirstcontinual;//进入坡道后判断从下方开始连续的赛道行数
    int junc_func_linefirstcontinual;//进入坡道后判断从下方开始连续的逻辑变量
    //丁字路标志
    int T_flag;//1左2右3左出4右出
    int T_Clear_Flag;//T字在斑马线处清零标志位
    int T_Angle;//上角点
    int T_outangleL;
    int T_outangleLX;
    int T_outangleR;
    int T_outangleRX;
    int T_outFlag;

    int zebra_flag;
    int zebra_flag2;
    int ZebraLine;
    int ZebraRowL;
    int ZebraRowR;
    int ZebraInLine;
    int8_t zebra_stop;
    int stop_count;
    int Garge_line;
    int Gargeout_Flag;
    int Zebra_Angle;
    int Zebra_Angle2;
    int Zebra_Angle2_x;
    //十字的标志位

    int cross_flag;
    int cross_RD_angle;//十字右下角点
    int cross_LD_angle;//十字左下角点

    int cross_line;
    uint32_t DectectStart;
    int cross_lost;
    int cross_AngleL;
    int cross_AngleR;
    int cross_AngleL_x;
    int cross_AngleR_x;

    //直线检测相关
    int16_t StrNumL;      //直线检测算法所得到的的左侧小段直线段的数目     使用位置：void Straight_check(void);
    int16_t StrNumR;      //直线检测算法所得到的的右侧小段直线段的数目
    float slopeL_max;   //左边线斜率最大值
    float slopeL_min;   //左边线斜率最小值
    float slopeR_max;   //右边线斜率最大值
    float slopeR_min;   //左边线斜率最小值
    float d_angle_left; //左边线角度变化
    float d_angle_right;//右边线角度变化

    int April_flag;
    int AprilLine;
    uint32_t AprilStart;

    uint32_t FruitStart;
    uint32_t LaserStart;
    uint32_t AnimalStart;

    char servo_flag;
    uint32_t ServoStart;

    int distance;





    int del_dir;
    int outloop7_angleflag;

    //路程积分相关
    //int Dis_Flag;//路程积分标志位，置1时开启路程积分，积分完成后自动置2
    int16_t distanceThres;     //积分阈值，路程积分到此值时结束
    char Yaw_flag;   //角度积分标志 0：未开始积分 1：正在进行角度积分 2：角度积分完成

    char Beep_flag;
    int  Beep_time;

    int watchX;
    int watchY;
    int watchleft;//左侧切点所在行
    int watchright;//右侧切点所在行
    float watch_length;//模长，切记不能为零，不然会出bug
    ////国赛跑三圈
    int8_t turn_count;
    //斑马线计数
    int zebra_count;
    //断路行计数标记
    int broken_circuit_line_count;
    //检测断路标记次数
    uint8_t broken_circuit_time_count;
    //断路标记为  1:开始进入断路  0：不在断路 2:完全进入断路
    uint8_t broken_circuit_flag;
    //斜坡标记位
    uint8_t slope_flag;
    //小型黑色路障标记位
    uint8_t black_obstacle_flag;
    uint8_t left_obstacle_flag;
    uint8_t right_obstacle_flag;
    int black_obstacle_line;
    int left_obstacle_x;
    int right_obstacle_x;
    //前方障碍物距离
    float forward_distance;
    uint8_t obstacle_flag;  //避障标记位
    uint8_t obstacle_angle; //障碍物角点
    uint8_t Line_patrol_mode;//巡线模式  0:中线 1：左线 2：右线
    uint8_t out_garage_flag;//出库标记位 1：左出库前直行 2：向左打角出库 3：右出库前直行 4：向右打角出库  0:出库结束，正常行驶
    //停车标记位
    uint8_t garage_stop;
    uint8_t garage_flag;
    //误差计算相关
    uint8_t angle_far_line;
    uint8_t angle_near_line;
    int track_count;
    int track_count_far;

    //扫线优势，当出现分叉时，0：以最大的白块为赛道 1：以左侧为基准决定赛道 2：以右侧为基准决定赛道
    uint8_t scan_line_advantage;

};
extern struct watch_o watch;
extern Element_range Element;
extern struct Element_struct Element_rem;
void Element_recognition();
void enter_element(Element_range element);
void out_element();
void element_init();
void clear_all_flags();
void enter_task();
void count_lost();
#include "Binarization.hpp"
#include "scan_line.hpp"
#include "patch_line.hpp"
#include "act_persp.hpp"
#include "garage.hpp"
#include "circle.hpp"
#include "cross.hpp"
#include "slope.hpp"
#include "broken_circuit.hpp"
#include "black_obstacle.hpp"
#endif /* CODE_CAMERA_PROCESS_ELEMENT_RECOGNITION_H_ */
