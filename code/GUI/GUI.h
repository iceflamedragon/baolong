/*
 * GUI.h
 *
 *  Created on: 2023��6��20��
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
  NONE=0, //��ָ��
  PRESS,  //��ת����������
  Press2,//ѡ���������������
  UP,     //�����ϲ���key1���£�
  OK,     //���ְ��£�key3���£�
  DOWN,   //�����²���key2���£�
  CW,     //��ת������˳ʱ����ת
  CCW     //��ת��������ʱ����ת
}TSUI_ButtonEventEnum;

typedef struct TSUI_TypeDef
{
  uint8 paraStatus; //1�������ı�  0������ѡ��
  uint8 paraSelect;//����ѡ�񣬼�¼��ǰѡ�еĲ��������
  uint8 paraMax;   //�ɵ�����������
  uint8 paraMax2;   //�ڶ�ҳ�ɵ�����������
  uint8 para_page; //�ɵ�����ҳ��
  uint8 showPage;     //����ʾͼ���ҳ��
  uint8 flag_page;    //��Ǳ�����ҳ��
  uint8 img_showmode;
  TSUI_ButtonEventEnum buttonevent;   //��ǰ��ָ��
}TSUI_TypeDef;

extern TSUI_TypeDef tsui;
void UI_init();
void TSUI_ButtonEvent();
void TSUI_FlashRead();
void TSUI_FlashSave();
uint8  Get_maxpage(struct paralist_s* adjust_paralist);




#endif /* CODE_GUI_GUI_H_ */
