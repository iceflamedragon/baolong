/*
 * GUI.h
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */

#ifndef CODE_GUI_GUI_H_
#define CODE_GUI_GUI_H_
#include "global.h"
#include "port.h"
#include "display.h"
#include "application.h"
typedef enum
{
  NONE=0, //无指令
  PRESS,  //旋转编码器按下
  Press2,//选择编码器连按两下
  UP,     //拨轮上拨（key1按下）
  OK,     //拨轮按下（key3按下）
  DOWN,   //拨轮下拨（key2按下）
  CW,     //旋转编码器顺时针旋转
  CCW     //旋转编码器逆时针旋转
}TSUI_ButtonEventEnum;

typedef struct TSUI_TypeDef
{
  uint8 paraStatus; //1：参数改变  0：参数选择
  uint8 paraSelect;//参数选择，记录当前选中的参数的序号
  uint8 paraMax;   //可调参数的总数
  uint8 paraMax2;   //第二页可调参数的总数
  uint8 para_page; //可调参数页码
  uint8 showPage;     //与显示图像的页码
  uint8 flag_page;    //标记变量的页数
  uint8 img_showmode;
  TSUI_ButtonEventEnum buttonevent;   //当前的指令
}TSUI_TypeDef;

extern TSUI_TypeDef tsui;
void UI_init();
void TSUI_ButtonEvent();
void TSUI_FlashRead();
void TSUI_FlashSave();
uint8  Get_maxpage(struct paralist_s* adjust_paralist);




#endif /* CODE_GUI_GUI_H_ */
