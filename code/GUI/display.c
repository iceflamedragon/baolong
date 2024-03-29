/*
 * display.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "display.h"
#pragma section all "cpu0_dsram"
struct diaplay_flag page[20];
uint8 ips_show_page[20];
uint8 imo[120][188];//调试图，显示了当前赛道边界，切线等重要标记位
void ips_display_flag(uint16 colum, uint16 row, const char dat[])
{
    u8 i=19;
    while(i--)ips_show_page[i]=' ';                   //清除上次显示在字符串中残留的字符
    uint16 j = 0;
    sprintf(ips_show_page,dat);
    i=19;
    while(i--)if(ips_show_page[i]==0)ips_show_page[i]=' ';//清除字符串中的'\0'，防止字符串断开   （sprintf函数会自动写入'\0'）
    ips_show_page[19]=0;                   //填入'\0'，提前截止字符串
    GUI_display_string(colum,row,ips_show_page);         //显示字符串

}
#define flag_leftmost_column 189
void ips_display_page()
{
    //ips200_display_font=IPS200_8X16_FONT;
    sprintf(ips_show_page,"page %d  ",tsui.flag_page);
    GUI_display_string(flag_leftmost_column,0,ips_show_page);         //显示字符串
    ips_display_flag(flag_leftmost_column, 1, page[tsui.flag_page].line1);
    ips_display_flag(flag_leftmost_column, 2, page[tsui.flag_page].line2);
    ips_display_flag(flag_leftmost_column, 3, page[tsui.flag_page].line3);
    ips_display_flag(flag_leftmost_column, 4, page[tsui.flag_page].line4);
    ips_display_flag(flag_leftmost_column, 5, page[tsui.flag_page].line5);
    ips_display_flag(flag_leftmost_column, 6, page[tsui.flag_page].line6);
    ips_display_flag(flag_leftmost_column, 7, page[tsui.flag_page].line7);
    ips_display_flag(flag_leftmost_column, 8, page[tsui.flag_page].line8);
    ips_display_flag(flag_leftmost_column, 9, page[tsui.flag_page].line9);
    ips_display_flag(flag_leftmost_column, 10, page[tsui.flag_page].line10);
    ips_display_flag(flag_leftmost_column, 11, page[tsui.flag_page].line11);
    ips_display_flag(flag_leftmost_column, 12, page[tsui.flag_page].line12);
    ips_display_flag(flag_leftmost_column, 13, page[tsui.flag_page].line13);
    ips_display_flag(flag_leftmost_column, 14, page[tsui.flag_page].line14);
}
//显示的位置，调整的位置
#define variable_length_max 19
#define display_length_max 31
#define display_top_row 2
#define display_bottom_row 18
#define display_adjust_row 5
#define display_leftmost_column 0
uint8 IPS_Display_string[40];//IPS要显示的字符串
void GUI_display_setpara(u8 num,u16 colum,u16 row,u8 *display_char,int data)
{
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string));//清除上次显示在字符串中残留的字符
    sprintf(IPS_Display_string,"%-3d",num);                 //前三位显示数字，数字格式：至少占3列且向左对齐
    sprintf(IPS_Display_string+3,display_char);             //从第四位开始将参数的名称写入字符串
    IPS_Display_string[strlen(IPS_Display_string)]=' ';//清除字符串中的'\0'，防止字符串断开   （sprintf函数会自动写入'\0'）
    sprintf(IPS_Display_string+4+variable_length_max,"=%-6d",data);//预留出（变量名称最大显示长度）的空间，将变量写入后面的位置 数字格式：至少占6列且向左对齐
    IPS_Display_string[strlen(IPS_Display_string)]=' ';     //清除字符串中的'\0'，防止字符串断开  （sprintf函数会自动写入'\0'）
    IPS_Display_string[display_length_max]=0;                   //填入'\0'，提前截止字符串
    GUI_display_string(colum,row,IPS_Display_string);               //显示字符串
}
void GUI_display_blank_roll(u16 colum,u16 row)
{
    u8 i=29;
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string)); //清除上次显示在字符串中残留的字符
    IPS_Display_string[display_length_max]=0;          //填入'\0'，提前截止字符串
    GUI_display_string(colum,row,IPS_Display_string);               //显示字符串
}
void GUI_display_adjusting(u8 num,u16 colum,u16 row,u8 *display_char,int data,u8 mode)
{
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string)); //清除上次显示在字符串中残留的字符
    sprintf(IPS_Display_string,"%-3d",num);                 //前三位显示数字，数字格式：至少占3列且向左对齐
    sprintf(IPS_Display_string+3,display_char);             //从第四位开始将参数的名称写入字符串
    IPS_Display_string[strlen(IPS_Display_string)]=' ';//清除字符串中的'\0'，防止字符串断开  （sprintf函数会自动写入'\0'）
    sprintf(IPS_Display_string+4+variable_length_max,"=%-6d",data);//预留出（变量名称最大显示长度）的空间，将变量写入后面的位置数字格式：至少占6列且向左对齐

    IPS_Display_string[strlen(IPS_Display_string)]=' ';//清除字符串中的'\0'，防止字符串断开  （sprintf函数会自动写入'\0'）
    if(!mode)   //参数选择 模式
        {
            IPS_Display_string[2]='[';                          //在相应的位置加入'[' 与 ']'来表示要调的是什么
            IPS_Display_string[3+variable_length_max]=']';
        }
    else       //参数改变 模式
        {
            IPS_Display_string[3+variable_length_max]='[';    //在相应的位置加入'[' 与 ']'来表示要调的是什么
            IPS_Display_string[display_length_max-1]=']';
        }
    IPS_Display_string[display_length_max]=0;                   //填入'\0'，提前截止字符串
    GUI_display_string(colum,row,IPS_Display_string);         //显示字符串
}

void show_changeable()//UI界面显示函数（显示的是整个界面）
{
    int row=0,num=0;                    //row:行数  num:变量的序号
    static int temp_para_select=0;
    static u8 display_mode_last=0;
    static u16 display_data_last=0;
    static u8 para_max;
    struct paralist_s*adjust_paralist;
    temp_para_select = tsui.paraSelect;
    display_mode_last = tsui.paraStatus;
    if(tsui.para_page==1)
    {
        adjust_paralist= paralist2;
        para_max=tsui.paraMax2;
    }
    else
    {
        adjust_paralist= paralist;
        para_max=tsui.paraMax;
    }
    display_data_last=*adjust_paralist[temp_para_select].para;
    for(row=display_top_row;row<=display_bottom_row;row++)  //从顶行到底行，一行行地显示
    {
        num=temp_para_select-(display_adjust_row-row);      //计算出这行显示的参数的序号
        if(row==display_adjust_row)                         //如果这一行正好是要调整的参数
        {
            GUI_display_adjusting(num,display_leftmost_column,row,adjust_paralist[num].label,*adjust_paralist[num].para,tsui.paraStatus);
        }
        else                                                //如果这一行不是要调整的参数
        {
            if(num>para_max||num<0)GUI_display_blank_roll(display_leftmost_column,row);
            //如何这行的参数序号小于0或大于参数总数，说明这行没有参数要显示，因此显示一堆空格清除之前显示的内容

            else GUI_display_setpara(num,display_leftmost_column,row,adjust_paralist[num].label,*adjust_paralist[num].para);//显示参数

        }
    }

}


//画水平线，相比连续画点更省时间
void ips200_draw_horizon(int16 xs, int16 ys, int16 xe, int16 ye, int16 color)
{
/*    if(ys>=0&&xs>=0&&xe>=0&&ye>=0)
    {
        ips200_set_region(xs, ys, xe, ye);
        for (uint16 i = 0; i < (xe - xs +1); i++)
        {
            ips200_write_16bit_data(color);
        }
    }*/
}

//画竖直线，相比连续画点更省时间
void ips200_draw_vertical(int16 xs, int16 ys, int16 xe, int16 ye, int16 color)
{
/*    if(ys>=0&&xs>=0&&xe>=0&&ye>=0)
    {
    ips200_set_region(xs, ys, xe, ye);
    for (uint16 i = 0; i < (ye - ys + 1); i++)
        ips200_write_16bit_data(color);
    }*/
}
void IPS_ShowEdge(uint8 *p, uint16 width, uint16 height)
{
    int16 i, j;
    IPS200_CS(0);
    ips200_address_set(0, 0, width - 1, height - 1);
    //改变读取像素点的顺序
    for (j = height - 1; j >= 0; j--)
    //for (j = 0; j <height; j++);
    {
        for (i = 0; i < width; i++)
        {
            if (*(p + j * width + i) == 1)
                ips200_write_16bit_data(RGB565_RED);
            else if (*(p + j * width + i) == 2)
                ips200_write_16bit_data(RGB565_GREEN);
            else if (*(p + j * width + i) == 3)
                ips200_write_16bit_data(RGB565_YELLOW);
            else if (*(p + j * width + i) == 4)
                ips200_write_16bit_data(RGB565_GRAY);
            else if (*(p + j * width + i) == 5)
                ips200_write_16bit_data(RGB565_BLUE);
            else if (*(p + j * width + i) == 6)
                ips200_write_16bit_data(RGB565_WHITE);
            else
                ips200_write_16bit_data(RGB565_BLACK);
        }
    }
    IPS200_CS(1);
}
//void ips200_write_16bit_data (const uint16 dat);
void IPS_ShowEdge2(uint8 *p,uint8 *q, uint16 width, uint16 height)
{
    int16 i, j;
    IPS200_CS(0);
    ips200_address_set(0, 0, width - 1, height - 1);
    //改变读取像素点的顺序
    for (j = height - 1; j >= 0; j--)
    //for (j = 0; j <height; j++);
    {
        for (i = 0; i < width; i++)
        {
            if (*(p + j * width + i) == 1)
                ips200_write_16bit_data(RGB565_RED);
            else if (*(p + j * width + i) == 2)
                ips200_write_16bit_data(RGB565_GREEN);
            else if (*(p + j * width + i) == 3)
                ips200_write_16bit_data(RGB565_YELLOW);
            else if (*(p + j * width + i) == 4)
                ips200_write_16bit_data(RGB565_GRAY);
            else if (*(p + j * width + i) == 5)
                ips200_write_16bit_data(RGB565_BLUE);
            else if (*(p + j * width + i) == 6)
                ips200_write_16bit_data(RGB565_WHITE);
            else if (*(q + (119-j) * width + i) >128)
                ips200_write_16bit_data(RGB565_GRAY);
            else
                ips200_write_16bit_data(RGB565_BLACK);
        }
    }
    IPS200_CS(1);

}
//图像显示函数
void ips_show_img()
{
//    sprintf(page[5].line11,"page=%d",tsui.img_showmode);
//    if(tsui.img_showmode==0)//显示黑白图
//    {
//        ips200_show_gray_image(0, 0, (const uint8 *)Grayscale, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//        //paint_patch_line(0);
//    }
//    else if(tsui.img_showmode==1)//显示原图
//    {
//        ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, 188, 120, 188, 120, 0);
//    }
//    else if(tsui.img_showmode==2)//显示扫线图
//    {
//        //ips200_show_gray_image(0, 0, (const uint8 *)Grayscale, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//        IPS_ShowEdge2((uint8_t *)imo,(uint8_t *)Grayscale, 188, 120);
//        memset(imo, 0, 120 * 188);
//    }
//    else if(tsui.img_showmode==3)//显示补线图
//    {
//        //ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, 188, 120, 188, 120, 0);
//        IPS_ShowEdge2((uint8_t *)imo,(uint8_t *)Grayscale, 188, 120);
//        memset(imo, 0, 120 * 188);
//
//        switch(setpara.TextRow)
//        {
//            case 1:
//    //////////////////圆环
////                ips200_draw_horizon(0, 119 - watch.InLoopAngleL, 188, 119 - watch.InLoopAngleL, RGB565_PURPLE);
////                ips200_draw_horizon(0, 119 - watch.InLoopCirc, 188, 119 - watch.InLoopCirc, RGB565_BLUE);
////                ips200_draw_horizon(0, 119 - watch.InLoopAngle2, 188, 119 - watch.InLoopAngle2, RGB565_YELLOW);
////                ips200_draw_horizon(0, 119 - watch.OutLoopAngle2, 188, 119 - watch.OutLoopAngle2, RGB565_YELLOW);
//                break;
//            case 2:
//    //////////////////三岔路
///*                ips200_draw_horizon(0, 119 - watch.JuncLine, IMG_H, 119 - watch.JuncLine, RGB565_PURPLE);
//                ips200_draw_horizon(0, 119 - watch.JuncAngle, IMG_H, 119 - watch.JuncAngle, RGB565_BLUE);
//                ips200_draw_vertical(watch.JuncAngleX, 0, watch.JuncAngleX, 119, RGB565_BLUE);
//                ips200_draw_horizon(0, 119 - watch.JuncLineR, IMG_H, 119 - watch.JuncLineR, RGB565_YELLOW);
//                ips200_draw_horizon(0, 119 - watch.JuncLineL, IMG_H, 119 - watch.JuncLineL, RGB565_BROWN);*/
//                // ips200_draw_horizon(0, 119 - setpara.WhenJuncStop, 160, 119 - setpara.WhenJuncStop, BROWN);
//                break;
//            case 3:
//    //////////////////二维码
////                ips200_draw_horizon(0, 119 - watch.AprilLine, 160, 119 - watch.AprilLine, RGB565_PURPLE);
////                ips200_draw_horizon(0, 119 - setpara.AprilStop, 160, 119 - setpara.AprilStop, BLUE);
//                break;
//            case 4:
//    //////////////////车库
////                 ips200_draw_horizon(0, 119 - setpara.cross_base, 160, 119 - setpara.cross_base, RGB565_BROWN);
////                 ips200_draw_vertical(watch.ZebraRowL, 0, watch.ZebraRowL, 119, RGB565_PURPLE);
////                 ips200_draw_vertical(watch.ZebraRowR, 0, watch.ZebraRowR, 119, RGB565_YELLOW);
//            default:
//                break;
//        }
//    }
//    else if(tsui.img_showmode==4)//显示逆透视边线图
//    {
//        IPS_ShowEdge((uint8_t *)imo, 188, 120);
//        memset(imo, 0, 120 * 188);
//    }


}
#pragma section all restore
