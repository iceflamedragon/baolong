/*
 * display.h
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */

#ifndef CODE_GUI_DISPLAY_H_
#define CODE_GUI_DISPLAY_H_
#include "GUI.h"
#define IPS_display_max_page 6
extern uint8 imo[120][188];
//标记变量显示
struct diaplay_flag
{
    uint8 line1[30];
    uint8 line2[30];
    uint8 line3[30];
    uint8 line4[30];
    uint8 line5[30];
    uint8 line6[30];
    uint8 line7[30];
    uint8 line8[30];
    uint8 line9[30];
    uint8 line10[30];
    uint8 line11[30];
    uint8 line12[30];
    uint8 line13[30];
    uint8 line14[30];
};
void ips_display_flag(uint16 colum, uint16 row, const char dat[]);
void ips_display_page();
void GUI_display_setpara(u8 num,u16 colum,u16 row,u8 *display_char,int data);
void GUI_display_blank_roll(u16 colum,u16 row);
void GUI_display_adjusting(u8 num,u16 colum,u16 row,u8 *display_char,int data,u8 mode);
void show_changeable();
void ips_show_img();
void ips200_draw_horizon(int16 xs, int16 ys, int16 xe, int16 ye, int16 color);
void ips200_draw_vertical(int16 xs, int16 ys, int16 xe, int16 ye, int16 color);
void IPS_ShowEdge2(uint8 *p,uint8 *q, uint16 width, uint16 height);
extern struct diaplay_flag page[20];

#endif /* CODE_GUI_DISPLAY_H_ */
