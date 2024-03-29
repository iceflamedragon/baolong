/*
 * port.c
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */
#include "port.h"
/*******��flash��صĽӿں���**********************************************************/

//----------------------------------------------------------------------
//flash ����һҳ��ȫ�����ݵĺ���
// ����˵��    ҳ����
//----------------------------------------------------------------------
//���ʵ�ڲ�����ֲflash���԰���������������������ȫע�͵�������ʹ��flash�洢���ܣ�ע�ⲻ��ע������������ֻע�������ݣ�
void GUI_flash_erase_page(u32 page_num)
{
    flash_erase_page(0,page_num);
}

//----------------------------------------------------------------------
//flash ��ȡһҳ���ݵĺ���
// ����˵��   page_num��ҳ����
// ����˵��   buf����ȡ����������ŵĵ�ַ�������ַ��
// ����˵��   len����������
//----------------------------------------------------------------------
void GUI_flash_read_page( u32 page_num, u32 *buf, u16 len)
{
    flash_read_page(0,page_num,buf,len);
}

//----------------------------------------------------------------------
//flash д��һҳ���ݵĺ���
// ����˵��   page_num��ҳ����
// ����˵��   buf��Ҫд�����������ŵĵ�ַ�������ַ��
// ����˵��   len����������
//----------------------------------------------------------------------
void GUI_flash_write_page(u32 page_num, const u32 *buf, u16 len)
{
    flash_write_page (0,page_num,buf,len);
}

/***********************************************************************************/

/*******��ʾ������صĽӿں���***********************************************************/

//----------------------------------------------------------------------
// �ַ�����ʾ����
// ����˵��     x               ����x�������㣬��λΪһ����
// ����˵��     y               ����y�������㣬��λΪһ��
// ����˵��     dat             ��Ҫ��ʾ���ַ���
//----------------------------------------------------------------------
void GUI_display_string(u16 x,u16 y,const char dat[])//x�ĵ�λΪ���أ�y�ĵ�λΪһ�У�ǧ��ҪŪ��
{
    ips200_show_string(x,y*12,dat);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ����ĺ���
// ����˵��     x               ����x�������� [0, ips200_x_max-1]
// ����˵��     y               ����y�������� [0, ips200_y_max-1]
// ����˵��     color           ��ɫ��ʽ RGB565 ���߿���ʹ�� zf_common_font.h �� rgb565_color_enum ö��ֵ��������д��
// ���ز���     void
//-------------------------------------------------------------------------------------------------------------------
void GUI_draw_point(uint16 x, uint16 y, const uint16 color)
{
    ips200_show_string (x,y,color);
}


/***********************************************************************************/
#define long_press_time 1000
//����ɨ�躯��
void key_scan()
{
    static uint8 key_press;
    static uint16 key_press_time;
    static uint16 key_interval_time;
    if((key_press==0)&&(KEY1_PIN==0||KEY2_PIN==0||KEY3_PIN==0||PRESS_PIN==0))
   {
       //GUI_delay_ms(1);
       key_press=1;//��⵽�а�ťѹ��
       if(KEY1_PIN==0)
           {
               tsui.buttonevent=UP;          //����ָ��
               //tsui.buttonevent=CCW;
           }
       else if(KEY2_PIN==0)
           {
               tsui.buttonevent=DOWN;          //����ָ��
               //tsui.buttonevent=CW;
           }
       else if(KEY3_PIN==0)
           {
               tsui.buttonevent=OK;          //����ָ��
               //tsui.buttonevent=PRESS;
           }
       else if(PRESS_PIN==0)
           {
               tsui.buttonevent=PRESS;   //����ָ��
//               if(mycar.car_running==1)car_stop();
//               adjust.take_picture=1;
               if(key_interval_time>0)
               {
                   tsui.buttonevent=Press2;
                   key_interval_time=0;
               }
               else if(key_interval_time==0)key_interval_time=400;
           }
   }
    else if(key_press==1)
   {
       if(KEY1_PIN==0)//���key1���ڰ���״̬��up�����
       {
           key_press_time++;            //�����ۼ�
           if(key_press_time>long_press_time)//���³���һ��ʱ��
           {
               TSUI_FlashSave();
           }
       }
       else if(KEY2_PIN==0)//���key2���ڰ���״̬
       {
           key_press_time++;            //�����ۼ�
           if(key_press_time>long_press_time)//���³���һ��ʱ��
           {

           }
       }
       else if(KEY3_PIN==0)//���key3���ڰ���״̬   //push
       {
           key_press_time++;            //�����ۼ�
           if(key_press_time==long_press_time)//���³���һ��ʱ��
           {
//               TSUI_FlashSave();                    //�洢����
               if(mycar.car_running==0)
                   {
//                       car_begin();
                   }
               tsui.buttonevent=PRESS;              //����ָ���ʾ����ɹ������Ƿ������ǿ��Ƿ�洢�ɹ����Ե�Ƭ����˵ûʲô���壩
           }
       }
       else if(PRESS_PIN==0)//������ڰ���״̬
       {
           key_press_time++;            //�����ۼ�
           if(key_press_time==long_press_time)//���³���һ��ʱ��
           {
               tsui.buttonevent=Press2;              //����ָ���ʾ����ɹ������Ƿ������ǿ��Ƿ�洢�ɹ����Ե�Ƭ����˵ûʲô���壩
//               TSUI_FlashSave();
//               car_begin();

           }
       }
       else                                 //�����ɿ�
       {
           key_press=0;                     //�����ⰴ���Ƿ��µı���
           key_press_time=0;                //�����ʱ��
       }
   }
    if(key_interval_time!=0)key_interval_time--;
    //sprintf(page[0].line14,"interval_time=%d ",key_interval_time);
}
//ec11ɨ�躯�������ڶ�ʱ���ж���
void ec11_scan()
{
    static uint8 CW_PIN_level,CCW_PIN_level,CW_PIN_level_last,CCW_PIN_level_last;
    CW_PIN_level=CW_PIN;
    CCW_PIN_level=CCW_PIN;
    if(CW_PIN_level==0&&CW_PIN_level_last==1)
    {
        if(CCW_PIN_level==1)tsui.buttonevent=CCW;
        else if(CCW_PIN_level==0)tsui.buttonevent=CW;
    }
    CW_PIN_level_last=CW_PIN_level;
    CCW_PIN_level_last=CCW_PIN_level;
}





