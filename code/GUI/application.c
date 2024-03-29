/*
 * application.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "application.h"
struct setpara_STRUCT setpara;         //可调量(即祖传代码的所有参量)
void GUI_Init()//GUI初始化函数,放在while循环之前
{
    UI_init();   //UI初始化函数，获取可调参数的相关信息
    init_setpara();//参数初始化，用此函数直接把所有的参数赋值(包括不在调参列表上的)，

//    TSUI_FlashSave(); //保存所有在调参列表上的参数，首次烧录或改变参数列表时，此时flash无数据时使用此函数，烧完之后记得注释掉这行再重新烧录一遍

//    TSUI_FlashRead();   //如果改了参数表数据和参数会对不上，必须注释，读取调参列表中的参数（此函数会覆盖掉之前参数初始化函数给调参列表上的参数赋的值）（即调参列表之外的参数用参数初始化函数赋的值，调参列表上的用内部flash的值）

}
//扫描函数，放在定时器中断里
void GUI_scan()
{
    key_scan(); //按键扫描函数
    ec11_scan();
    TSUI_ButtonEvent();//由扫描函数获得的数据计算当前的状态
}

void GUI_Display()//显示函数，此函数要放在main函数中的while内
{
    ips_show_img();
    ips_display_page();
    show_changeable();//GUI显示函数
}
int a ;
struct paralist_s paralist[100]=   //可调参数列表
{
   //格式： {参数地址,参数名称,调节精度}, 注：1.参数只能是整形参数 2.参数名称只能是字符串且最多16个字符 3.调节精度最小为1  4.受到多种原因的限制，最多只能同时调100个参数（序号0—99）

         {&setpara.com_speed_PID.kp,"speed_kp",1},       //速度环纯p
        {&setpara.com_speed_PID.ki,"speed_ki",1},//速度环KI
        {&setpara.com_turn_PID.kp,"turn_kp",1},     //转向PID
        {&setpara.com_turn_PID.kd,"turn_kd",1},
        {&setpara.speed_max,"speed_max",1},
         {&setpara.SteerMid,"SteerMid",1},
         {&a,"111111111111111",1},
        {&setpara.speed_min,"speed_min",1},
        {&setpara.speed_adjust_kp,"speed_adjust_k",1},
//        {&setpara.obstacle_speed,"obs_speed",1},
//        {&setpara.broken_target_speed,"broken_speed",1},
//        {&setpara.garage_target_speed,"garage_speed",1},
//        {&setpara.loop_target_speed,"loop_speed",1},
//        {&setpara.big_loop_speed,"big_loop_speed",1},
//        {&a,"111111111111111",1},
//        {&setpara.loop_turn_PID.kp,"loop_turn_kp",1},
//        {&setpara.loop_turn_PID.kd,"loop_turn_kd",1},
//        {&setpara.big_loop_PID.kp,"big_loop_kp",1},
//        {&setpara.big_loop_PID.kd,"big_loop_kd",1},
//        {&setpara.adc_turn_PID.kp, "adc_turn_kp", 1},
//        {&setpara.adc_turn_PID.kd, "adc_turn_kd", 1},
//        {&setpara.garage_turn_PID.kp,"garage_kp",1},
//        {&setpara.garage_turn_PID.kd,"garage_kd",1},
//          {&setpara.gyro_PID.kp,"gyro_kp",1},
//          {&setpara.gyro_PID.ki,"gyro_ki",1},
//          {&setpara.gyro_PID.kd,"gyro_kd",1},
//          {&a,"111111111111111",1},
//          {&setpara.stop_PID.kp,"garage_stop_kp",10},
//          {&setpara.stop_PID.ki,"garage_stop_ki",1},
//          {&setpara.obstacle_turn1_PID.kp,"obs_kp1",1},
//          {&setpara.obstacle_turn1_PID.kd,"obs_kd1",1},
//          {&setpara.obstacle_turn2_PID.kp,"obs_kp2",1},
//          {&setpara.obstacle_turn2_PID.kd,"obs_kd2",1},
//          {&a,"111111111111111",1},
//          {&setpara.camwf, "CamWF", 1},
//          {&setpara.camwl, "CamWL", 1},
//          {&setpara.camwr, "CamWR", 1},
//          {&setpara.far_line, "CamFar", 1},
          //{&setpara.TextRow,  "TextRow", 1},
//          {&setpara.threshold_max,"threshold_max",1},
//          {&setpara.threshold_min,"threshold_min",1},
          //{&setpara.begin_pwm,"begin_pwm",10},
          //{&setpara.begin_time,"begin_time",10},
//          {&setpara.adc_L_gain,"adc_L_gain",1},
//          {&setpara.adc_M_gain,"adc_M_gain",1},
//          {&setpara.adc_R_gain,"adc_R_gain",1},
//         {&setpara.adc_protect,"adc_protect",1},
//         {&setpara.fan_speed,"fan_speed",1},
          //{&setpara.swj_adjust,"swj_adjust",1},
          {0}                                       //这个{0}作为结尾，尽量不要删
};
struct paralist_s paralist2[100]=   //可调参数列表
{
    {&setpara.distanceThres_before_broken_circuit,"be_broken_dis",100},
    {&setpara.garage_begin_distance,"be_garage_dis",1000},
    {&setpara.obstacle_dis,"obstacle_dis",100},
    {&setpara.obstacle_dis2,"obstacle_dis2",100},
    {&setpara.loop_out_distance,"loop_out_dist",100},
    {&setpara.big_loop_out_distance,"big_loop_dist",100},
    {&setpara.garage_stop_distance,"gar_stop_dis",100},
    {&setpara.start_mode,"start_mode",1},
    {&setpara.stop_time,"stop_time",100},
    {&setpara.set_element[1],"element1",1},
    {&setpara.set_element[2],"element2",1},
    {&setpara.set_element[3],"element3",1},
    {&setpara.set_element[4],"element4",1},
    {&setpara.set_element[5],"element5",1},
    {&setpara.set_element[6],"element6",1},
    {&setpara.set_element[7],"element7",1},
    {&setpara.set_element[8],"element8",1},
    {&setpara.set_element[9],"element9",1},
    {&setpara.set_element[10],"element10",1},
    {&setpara.set_element[11],"element11",1},
    {&setpara.set_element[12],"element12",1},
    {&setpara.set_element[13],"element13",1},
    {&setpara.set_element[14],"element14",1},
    {&setpara.set_element[15],"element15",1},
    {&setpara.set_element[16],"element16",1},
    {&setpara.set_element[17],"element17",1},
    {&setpara.set_element[18],"element18",1},
    {&setpara.set_element[19],"element19",1},
    {&setpara.set_element[20],"element20",1},
    {&setpara.loop_data[0],"loop0",1},
    {&setpara.loop_data[1],"loop1",1},
     {&setpara.loop_data[2],"loop2",1},
     {&setpara.loop_data[3],"loop3",1},
     {&setpara.obstacle_dir,"obstacle_dir",1},//0左1右
     {&a,"111111111111111",1},
     {&setpara.out_garage_distance,"out_garage_dis",100},
     {&setpara.out_garage_angle,"out_garage_angle",1},
     {&setpara.out_garage_err,"out_garage_err",1},
     {&a,"111111111111111",1},
     {&setpara.garage_dis,"garage_dis",100},
     {&setpara.garage_angle,"garage_angle",1},
     {&setpara.garage_err,"garage_err",1},
     {&a,"111111111111111",1},

     {&setpara.distanceThres_in_broken_circuit,"in_broken_dis",100},
     {&setpara.distanceThres_out_broken_circuit,"out_broken_dis",100},
     {&a,"111111111111111",1},
     {&setpara.obstacle_dis3,"obstacle_dis3",100},
     {&setpara.obstacle_dis4,"obstacle_dis4",100},
     {&setpara.obstacle_angle1,"obs_angle1",1},
     {&setpara.obstacle_angle2,"obs_angle2",1},
     {&setpara.obstacle_detect_distance,"obs_dec_dis",10},
     {&setpara.broken_circuit_slow_distane,"broken_slow_dis",100},
     {&setpara.steer_adjust,"steer_adjust",1},
     //{&setpara.steer_pwm,"steer_pwm",100},
     //{&adjust.take_picture,"picture",1},
     //{&setpara.loop_angle_out,"loop_angle_out",1},
     //{&setpara.big_loop_out,"big_loop_out",1},
     {0}
};

void init_setpara()//各个参数的初始化，不在参数表上的参数仍然可以正常初始化与使用，只是不可调
{
//************************************第一页********************************
    mycar.steer_pwm=747;

    setpara.speed_max=85;
    setpara.speed_min=15;
    setpara.speed_adjust_kp=55;
    setpara.obstacle_speed=50;
    setpara.SteerMid=747;
    setpara.broken_target_speed=50;
    setpara.garage_target_speed=45;
    setpara.loop_target_speed=62;
    setpara.big_loop_speed=90;

    setpara.com_speed_PID.kp=1500;       //速度环pi
    setpara.com_speed_PID.ki=0;

    setpara.speed_down_PID.kp=230;      //减速kp
    setpara.com_turn_PID.kp=200;         //转弯PID
    setpara.com_turn_PID.kd=0;
//    setpara.gyro_PID.kp=118;
//
//    setpara.gyro_PID.ki=110;
//    setpara.gyro_PID.kd=0;
//    setpara.adc_turn_PID.kp=50;
//    setpara.adc_turn_PID.kd=20;
    setpara.begin_pwm=2000;
    setpara.begin_time=5000;
    setpara.stop_time=15000;
    setpara.loop_turn_PID.kp=14;
    setpara.loop_turn_PID.kd=13;
    setpara.big_loop_PID.kp=13;
    setpara.big_loop_PID.kd=11;
    setpara.stop_PID.kp=180;
    setpara.stop_PID.ki=30;
//    setpara.obstacle_turn1_PID.kp=3;
//    setpara.obstacle_turn1_PID.kd=3;
//    setpara.obstacle_turn2_PID.kp=3;
//    setpara.obstacle_turn2_PID.kd=3;
//    setpara.camwf=-41;
//    setpara.camwl=76;
//    setpara.camwr=105;
//    setpara.far_line=80; setpara.stop_time=2100;
      //{&setpara.TextRow,  "TextRow", 1},
//    setpara.threshold_max=120;
//    setpara.threshold_min=80;
//      //{&setpara.begin_pwm,"begin_pwm",10},
//      //{&setpara.begin_time,"begin_time",10},
//    setpara.adc_L_gain=100;
//    setpara.adc_M_gain=100;
//    setpara.adc_R_gain=100;
//    setpara.adc_protect=50;
      //{&setpara.swj_adjust,"swj_adjust",1},
//************************************第二页************************
//    setpara.start_mode=1;

//    setpara.set_element[1]=1;
//    setpara.set_element[2]=8;
//    setpara.set_element[3]=8;
//    setpara.set_element[4]=4;
//    setpara.set_element[5]=3;
//    setpara.set_element[6]=5;
//    setpara.set_element[7]=6;
//    setpara.set_element[8]=0;
//    setpara.set_element[9]=0;
//    setpara.set_element[10]=0;
//    setpara.set_element[11]=0;
//    setpara.set_element[12]=0;
//    setpara.set_element[13]=0;
//    setpara.set_element[14]=0;
//    setpara.set_element[15]=0;
//    setpara.set_element[16]=0;
//    setpara.set_element[17]=0;
//    setpara.set_element[18]=0;
//    setpara.set_element[19]=0;
//    setpara.set_element[20]=0;
//    setpara.loop_data[0]=0;
//    setpara.loop_data[1]=0;
//    setpara.loop_data[2]=0;
//    setpara.loop_data[3]=0;
//    setpara.obstacle_dir=0;//0左1右
//    setpara.out_garage_distance=2200;
//    setpara.out_garage_angle=30;
//    setpara.out_garage_err=8;
//    setpara.garage_dis=4000;
//    setpara.garage_angle=25;
//    setpara.garage_err=25;
//    setpara.garage_stop_distance=2000;
//    setpara.distanceThres_before_broken_circuit=0;
//    setpara.distanceThres_in_broken_circuit=9100;
//    setpara.distanceThres_out_broken_circuit=5000;
//    setpara.obstacle_dis=24000;
//    setpara.obstacle_dis2=2500;
//    setpara.obstacle_dis3=5700;
//    setpara.obstacle_dis4=0;
//    setpara.obstacle_angle1=42;
//    setpara.obstacle_angle2=30;
//    setpara.loop_out_distance=6000;
//    setpara.big_loop_out_distance=8200;
//    setpara.garage_begin_distance=180000;
//    setpara.obstacle_detect_distance=800;
//    setpara.garage_turn_PID.kp=15;
//    setpara.garage_turn_PID.kd=10;
//    setpara.broken_circuit_slow_distane=200;

    //mycar.steer_pwm=4000;
    //setpara.loop_angle_out=60;
    //setpara.big_loop_out=60;
 /*   setpara.speed_max=60;
    setpara.speed_min=54;
    setpara.speed_adjust_kp = 37;
    setpara.loop
    setpara.com_speed_PID.kp=170;
    setpara.com_speed_PID.ki=45;
    setpara.com_speed_PID.kd=0;
    setpara.com_turn_PID.kp=19;
    setpara.com_turn_PID.ki=0;
    setpara.com_turn_PID.kd=13;
    setpara.gyro_PID.kp=119;
    setpara.gyro_PID.ki=0;
    setpara.gyro_PID.kd=98;
    setpara.adc_turn_PID.kp=60;
    setpara.adc_turn_PID.ki=0;
    setpara.adc_turn_PID.kd=30;
    setpara.garage_turn_PID.kp=400;
    setpara.garage_turn_PID.ki=0;
    setpara.garage_turn_PID.kd=100;
    setpara.stop_PID.kp=500;
    setpara.stop_PID.ki=150;
    setpara.stop_PID.kd=0;
    setpara.obstacle_turn1_PID.kp=25;
    setpara.obstacle_turn1_PID.kd=5;
    setpara.obstacle_turn2_PID.kp=28;
    setpara.obstacle_turn2_PID.kd=6;
    setpara.obstacle_turn3_PID.kp=0;
    setpara.obstacle_turn3_PID.kd=0;
    setpara.start_mode=1;
    setpara.out_garage_distance=1000;
    setpara.out_garage_angle=30;
    setpara.out_garage_err=8;
    setpara.obstacle_dis=16000;
    setpara.obstacle_dis2=5400;
    setpara.obstacle_dis3=4500;
    setpara.distanceThres_before_broken_circuit=27000;
    setpara.distanceThres_in_broken_circuit=11600;
    setpara.distanceThres_out_broken_circuit=5000;
    setpara.obstacle_dir=0;
    setpara.garage_dis=8700;
    setpara.garage_angle=70;
    setpara.garage_err=15;
    setpara.obstacle_angle1=20;
    setpara.obstacle_angle2=33;
    setpara.com_target_speed=40;
    setpara.loop_target_speed=50;
    setpara.loop_speed_PID=setpara.com_speed_PID;
    setpara.loop_turn_PID.kp=25;
    setpara.loop_turn_PID.kd=20;
    setpara.threshold_min=80;
    setpara.threshold_max=120;
    setpara.camwl=78;
    setpara.camwr=110;
    setpara.camwf=-31;

    setpara.speed_adjust_kp;
    setpara.stop_time=500;
    setpara.TextRow=1;
    setpara.adc_L_gain=100;
    setpara.adc_M_gain=100;
    setpara.adc_R_gain=100;
    setpara.out_garage_angle=60;
    setpara.out_garage_err=14;
    setpara.out_garage_distance=1000;
    setpara.broken_target_speed=40;*/


/************以下为祖传代码涉及到的参数，可以参考一下******************///觉得不需要下面的可以全部注释
//   setpara.rolling_angle_mid = 0;
//    setpara.LoopWl = 87;
//    setpara.LoopWr = 102;
//    setpara.inter_d = -35;
//    setpara.inter_i = -54;
//    setpara.inter_p = -102;
//    setpara.outer_p = -150;
//    setpara.Pos_kp = 100;
//    setpara.Pos_ki = 40;
//    setpara.CamAveSpeed = 60;
//    setpara.delmax_dir = 40;
//    setpara.Bal_kp = 1;
//    setpara.mode        = 2     ; // 0前进  1左出库2右出库
//    setpara.settime     = 1210    ;
//    setpara.outer_p     = -156;
//    setpara.jun_wl = 87;
//    setpara.jun_wr = 102;
//    setpara.ad_kp = 60;
//    setpara.ad_ki = 0;
//    setpara.ad_kd = 20;
///////////速度相关
//    setpara.CamAveSpeed = 120   ;
//    setpara.SpeedKp     = 300   ;
//    setpara.SpeedKi     = 60    ;
//    setpara.SpeedKd     = 150   ;
//    setpara.diffK       = 40    ;
//    setpara.SpeedUP     = 100   ;
//    setpara.SpeedDOWN   = 85    ;
///////////舵机相关
//    setpara.SteerMid    = 735;
//    setpara.MaxTurn     = 150   ;
//    setpara.CamKp       = 14    ;
//    setpara.CamKd       = 2    ;
/////////属性相关
///*
//    setpara.camcfg.pitch        = 540   ;
//    setpara.camcfg.forward_far  = 105   ;   //视野不能超过120
//    setpara.camcfg.forward_near = 26     ;   //我觉得入环不行和这个参大小无关
//    setpara.camcfg.wf           = -31    ;   //13cm
//    setpara.camcfg.wl           = 78    ;
//    setpara.camcfg.wr           = 110    ; // 右轮位置，不能调大，影响入环
//    setpara.camcfg.rotate_x     = -4    ;
//    setpara.camcfg.rotate_y     = 2     ;
//    setpara.camcfg.shift        = 270   ;
/////////视觉相关
//    setpara.camcfg.manbri       = 0     ;
//    setpara.camcfg.exptime      = 153   ;
//    setpara.exp_ki              = 1     ;
//    setpara.camcfg.autogain     = 30    ;
//*/
///////////车库相关
//    setpara.garage_outangle     = 250   ; // 出库角度，+左-右（可优化多加一个12标志，不用拧这么久
//    setpara.garage_outtime      = 3     ;   // 出库锁角度时间，×0.01s
//     setpara.ZebraCount          = 3    ;//必须给值
//    setpara.stop_kp             = 100   ;
//    setpara.stop_kd             = 25    ;
//    // setpara.StopSpeed           = 70    ;
///////////圆环相关
//    setpara.CamLoopKp    = 13    ;
//    setpara.CamLoopKd    = 2    ;
//
//    setpara.InLoopLine   = 80    ;
//    setpara.WhenInLoop   = 83    ;
//    //setpara.InLoopRow    = 67    ;
//    setpara.LoopThres    = 8     ;
//    setpara.OutLoopThres = 40    ;
//    setpara.distanceThres_in_loop=6000;
//    setpara.distanceThres_out_loop=15000;
//    setpara.distanceThres_in_broken_circuit=9000;
//    setpara.distanceThres_out_broken_circuit=5000;
////////////三岔路相关
//    setpara.StrThres     = 10    ;
//    setpara.TempRL       = 2     ; //三岔路临时变量，左1右2，后期由OpenMV发送
//    setpara.JuncProcess  = 20     ;
//    setpara.Jun_out_distance = 10    ;
//    //setpara.WhenInLoop = 30;
//    setpara.OutJunc2Lambda = -800;
//    setpara.OutJunc2speed = 100;
//    //setpara.InLoopLine=90;
//    //setpara.JuncSpeed    = -2    ;
//    setpara.DetectTime   = 30    ;
//    setpara.number       = 1     ;
////////////二维码相关
//    // setpara.AprilCount   = 5     ;
//    // setpara.AprilStop    = 78    ;
//    // setpara.AprilSpeed   = 25    ;
//    // setpara.AprilTime    = 110   ;
//    // setpara.BackTime     = 15    ;
//    // setpara.LaserTime    = 80    ;
///////////测试相关
//    setpara.TextLine     = 33    ;
//    setpara.TextRow      = 2     ;
//    // setpara.TextDistance = 0     ;
///////////坡道
//    setpara.SlopeDown = 3;
//   // fuzzy.s_output.rolling_angle = 0;
//    setpara.loop_dLambda_dir = 55;
//    setpara.jun_dLambda_dir = 20;
//    setpara.jun_CamKp = 12;
//    setpara.jun_CamKd = 2;
//    //setpara.cross_base = 80;
////////其他参数
//    setpara.rolling_angle_mid = 0;
//    setpara.steer_far = 110;
//    setpara.trans = 3;
//    setpara.delmax_dir = 80;
///*
//    setpara.LoopOutAngle = 300;
//    setpara.fuzzy_s.s_ku = 700;
//    setpara.fuzzy_s.s_kp = 60;
//    setpara.fuzzy_s.s_ki = 30;
//    setpara.fuzzy_s.s_kd = 10;
//*/
//    setpara.Loop_deldir = 67;
//    setpara.Junc_deldir = 80;
//    setpara.O_steerfar = 60;
//    setpara.Slopedistance = 100;
//
////出库
//    setpara.garage_outangle = 121;
//    setpara.garage_outtime = 10;
}


