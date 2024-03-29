/*
 * port.h
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */

#ifndef CODE_GUI_PORT_H_
#define CODE_GUI_PORT_H_
#include "GUI.h"
/*********flash��غ궨��*****************/
#define EEPROM_SAVE_SECTOR 11    //���������ҳ��
#define EEPROM_BACKUP_SECTOR 10  //���ò��������ҳ��
#define EEPROM_BACKBACKUP_SECTOR 9 //�����������ݴ洢ҳ��
#define EEPROM2_SAVE_SECTOR 8    //���������ҳ�루�ڶ�ҳ��
#define EEPROM2_BACKUP_SECTOR 7  //���ò��������ҳ�루�ڶ�ҳ��
/********************************************/

/**********gpio��غ궨��***********/
//�ö�����ݲ��ְ�ť���ϲ����������൱��KEY1_PIN���²����������൱��KEY2_PIN���������������൱��KEY3_PIN
#define KEY1_PIN       gpio_get_level(KEY_UP_PROT)             //up�����
#define KEY2_PIN       gpio_get_level(KEY_DOWN_PROT)          //down�⼸������Ҫ�ĳ��Լ��İ�������Ӧ�����ţ���ʽҲҪ���Լ��Ŀ�һ��
#define KEY3_PIN       gpio_get_level(KEY_PUSH_PROT)          //push���� ͣ��
#define KEY4_PIN 1//gpio_get_level(P22_2)                   //���������Ӧ������Ϊ1

#define PRESS_PIN gpio_get_level(PRESS_PROT)
#define CW_PIN    gpio_get_level(CW_PROT)
#define CCW_PIN   gpio_get_level(CCW_PROT)



//IO�ڵ�ƽ��ȡ����
#define GUI_gpio_get_level(gpio_pin)   gpio_get_level(gpio_pin)    //�ĳ��Լ��Ŀ��ж�Ӧ��ȡ���ŵĺ���
/*********************************************/

/***********�ӳٺ����Ľӿں���****/
#define GUI_delay_ms(nms)   system_delay_ms(nms)                   //�ĳ��Լ��Ŀ��е��ӳٺ���
/***********************************/


//�������������Բ��ö�
void GUI_flash_erase_page(u32 page);
void GUI_flash_read_page( u32 page_num, u32 *buf, u16 len);
void GUI_flash_write_page(u32 page_num, const u32 *buf, u16 len);

void GUI_display_string(u16 x,u16 y,const char dat[]);
void GUI_draw_point(uint16 x, uint16 y, const uint16 color);
void GUI_draw_line(uint16 x_start, uint16 y_start, uint16 x_end, uint16 y_end, const uint16 color);
void key_scan();
void ec11_scan();




#endif /* CODE_GUI_PORT_H_ */
