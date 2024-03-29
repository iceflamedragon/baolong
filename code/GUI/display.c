/*
 * display.c
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */
#include "display.h"
#pragma section all "cpu0_dsram"
struct diaplay_flag page[20];
uint8 ips_show_page[20];
uint8 imo[120][188];//����ͼ����ʾ�˵�ǰ�����߽磬���ߵ���Ҫ���λ
void ips_display_flag(uint16 colum, uint16 row, const char dat[])
{
    u8 i=19;
    while(i--)ips_show_page[i]=' ';                   //����ϴ���ʾ���ַ����в������ַ�
    uint16 j = 0;
    sprintf(ips_show_page,dat);
    i=19;
    while(i--)if(ips_show_page[i]==0)ips_show_page[i]=' ';//����ַ����е�'\0'����ֹ�ַ����Ͽ�   ��sprintf�������Զ�д��'\0'��
    ips_show_page[19]=0;                   //����'\0'����ǰ��ֹ�ַ���
    GUI_display_string(colum,row,ips_show_page);         //��ʾ�ַ���

}
#define flag_leftmost_column 189
void ips_display_page()
{
    //ips200_display_font=IPS200_8X16_FONT;
    sprintf(ips_show_page,"page %d  ",tsui.flag_page);
    GUI_display_string(flag_leftmost_column,0,ips_show_page);         //��ʾ�ַ���
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
//��ʾ��λ�ã�������λ��
#define variable_length_max 19
#define display_length_max 31
#define display_top_row 2
#define display_bottom_row 18
#define display_adjust_row 5
#define display_leftmost_column 0
uint8 IPS_Display_string[40];//IPSҪ��ʾ���ַ���
void GUI_display_setpara(u8 num,u16 colum,u16 row,u8 *display_char,int data)
{
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string));//����ϴ���ʾ���ַ����в������ַ�
    sprintf(IPS_Display_string,"%-3d",num);                 //ǰ��λ��ʾ���֣����ָ�ʽ������ռ3�����������
    sprintf(IPS_Display_string+3,display_char);             //�ӵ���λ��ʼ������������д���ַ���
    IPS_Display_string[strlen(IPS_Display_string)]=' ';//����ַ����е�'\0'����ֹ�ַ����Ͽ�   ��sprintf�������Զ�д��'\0'��
    sprintf(IPS_Display_string+4+variable_length_max,"=%-6d",data);//Ԥ�������������������ʾ���ȣ��Ŀռ䣬������д������λ�� ���ָ�ʽ������ռ6�����������
    IPS_Display_string[strlen(IPS_Display_string)]=' ';     //����ַ����е�'\0'����ֹ�ַ����Ͽ�  ��sprintf�������Զ�д��'\0'��
    IPS_Display_string[display_length_max]=0;                   //����'\0'����ǰ��ֹ�ַ���
    GUI_display_string(colum,row,IPS_Display_string);               //��ʾ�ַ���
}
void GUI_display_blank_roll(u16 colum,u16 row)
{
    u8 i=29;
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string)); //����ϴ���ʾ���ַ����в������ַ�
    IPS_Display_string[display_length_max]=0;          //����'\0'����ǰ��ֹ�ַ���
    GUI_display_string(colum,row,IPS_Display_string);               //��ʾ�ַ���
}
void GUI_display_adjusting(u8 num,u16 colum,u16 row,u8 *display_char,int data,u8 mode)
{
    memset(IPS_Display_string,' ',sizeof(IPS_Display_string)); //����ϴ���ʾ���ַ����в������ַ�
    sprintf(IPS_Display_string,"%-3d",num);                 //ǰ��λ��ʾ���֣����ָ�ʽ������ռ3�����������
    sprintf(IPS_Display_string+3,display_char);             //�ӵ���λ��ʼ������������д���ַ���
    IPS_Display_string[strlen(IPS_Display_string)]=' ';//����ַ����е�'\0'����ֹ�ַ����Ͽ�  ��sprintf�������Զ�д��'\0'��
    sprintf(IPS_Display_string+4+variable_length_max,"=%-6d",data);//Ԥ�������������������ʾ���ȣ��Ŀռ䣬������д������λ�����ָ�ʽ������ռ6�����������

    IPS_Display_string[strlen(IPS_Display_string)]=' ';//����ַ����е�'\0'����ֹ�ַ����Ͽ�  ��sprintf�������Զ�д��'\0'��
    if(!mode)   //����ѡ�� ģʽ
        {
            IPS_Display_string[2]='[';                          //����Ӧ��λ�ü���'[' �� ']'����ʾҪ������ʲô
            IPS_Display_string[3+variable_length_max]=']';
        }
    else       //�����ı� ģʽ
        {
            IPS_Display_string[3+variable_length_max]='[';    //����Ӧ��λ�ü���'[' �� ']'����ʾҪ������ʲô
            IPS_Display_string[display_length_max-1]=']';
        }
    IPS_Display_string[display_length_max]=0;                   //����'\0'����ǰ��ֹ�ַ���
    GUI_display_string(colum,row,IPS_Display_string);         //��ʾ�ַ���
}

void show_changeable()//UI������ʾ��������ʾ�����������棩
{
    int row=0,num=0;                    //row:����  num:���������
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
    for(row=display_top_row;row<=display_bottom_row;row++)  //�Ӷ��е����У�һ���е���ʾ
    {
        num=temp_para_select-(display_adjust_row-row);      //�����������ʾ�Ĳ��������
        if(row==display_adjust_row)                         //�����һ��������Ҫ�����Ĳ���
        {
            GUI_display_adjusting(num,display_leftmost_column,row,adjust_paralist[num].label,*adjust_paralist[num].para,tsui.paraStatus);
        }
        else                                                //�����һ�в���Ҫ�����Ĳ���
        {
            if(num>para_max||num<0)GUI_display_blank_roll(display_leftmost_column,row);
            //������еĲ������С��0����ڲ���������˵������û�в���Ҫ��ʾ�������ʾһ�ѿո����֮ǰ��ʾ������

            else GUI_display_setpara(num,display_leftmost_column,row,adjust_paralist[num].label,*adjust_paralist[num].para);//��ʾ����

        }
    }

}


//��ˮƽ�ߣ�������������ʡʱ��
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

//����ֱ�ߣ�������������ʡʱ��
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
    //�ı��ȡ���ص��˳��
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
    //�ı��ȡ���ص��˳��
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
//ͼ����ʾ����
void ips_show_img()
{
//    sprintf(page[5].line11,"page=%d",tsui.img_showmode);
//    if(tsui.img_showmode==0)//��ʾ�ڰ�ͼ
//    {
//        ips200_show_gray_image(0, 0, (const uint8 *)Grayscale, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//        //paint_patch_line(0);
//    }
//    else if(tsui.img_showmode==1)//��ʾԭͼ
//    {
//        ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, 188, 120, 188, 120, 0);
//    }
//    else if(tsui.img_showmode==2)//��ʾɨ��ͼ
//    {
//        //ips200_show_gray_image(0, 0, (const uint8 *)Grayscale, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
//        IPS_ShowEdge2((uint8_t *)imo,(uint8_t *)Grayscale, 188, 120);
//        memset(imo, 0, 120 * 188);
//    }
//    else if(tsui.img_showmode==3)//��ʾ����ͼ
//    {
//        //ips200_show_gray_image(0, 0, (const uint8 *)mt9v03x_image, 188, 120, 188, 120, 0);
//        IPS_ShowEdge2((uint8_t *)imo,(uint8_t *)Grayscale, 188, 120);
//        memset(imo, 0, 120 * 188);
//
//        switch(setpara.TextRow)
//        {
//            case 1:
//    //////////////////Բ��
////                ips200_draw_horizon(0, 119 - watch.InLoopAngleL, 188, 119 - watch.InLoopAngleL, RGB565_PURPLE);
////                ips200_draw_horizon(0, 119 - watch.InLoopCirc, 188, 119 - watch.InLoopCirc, RGB565_BLUE);
////                ips200_draw_horizon(0, 119 - watch.InLoopAngle2, 188, 119 - watch.InLoopAngle2, RGB565_YELLOW);
////                ips200_draw_horizon(0, 119 - watch.OutLoopAngle2, 188, 119 - watch.OutLoopAngle2, RGB565_YELLOW);
//                break;
//            case 2:
//    //////////////////����·
///*                ips200_draw_horizon(0, 119 - watch.JuncLine, IMG_H, 119 - watch.JuncLine, RGB565_PURPLE);
//                ips200_draw_horizon(0, 119 - watch.JuncAngle, IMG_H, 119 - watch.JuncAngle, RGB565_BLUE);
//                ips200_draw_vertical(watch.JuncAngleX, 0, watch.JuncAngleX, 119, RGB565_BLUE);
//                ips200_draw_horizon(0, 119 - watch.JuncLineR, IMG_H, 119 - watch.JuncLineR, RGB565_YELLOW);
//                ips200_draw_horizon(0, 119 - watch.JuncLineL, IMG_H, 119 - watch.JuncLineL, RGB565_BROWN);*/
//                // ips200_draw_horizon(0, 119 - setpara.WhenJuncStop, 160, 119 - setpara.WhenJuncStop, BROWN);
//                break;
//            case 3:
//    //////////////////��ά��
////                ips200_draw_horizon(0, 119 - watch.AprilLine, 160, 119 - watch.AprilLine, RGB565_PURPLE);
////                ips200_draw_horizon(0, 119 - setpara.AprilStop, 160, 119 - setpara.AprilStop, BLUE);
//                break;
//            case 4:
//    //////////////////����
////                 ips200_draw_horizon(0, 119 - setpara.cross_base, 160, 119 - setpara.cross_base, RGB565_BROWN);
////                 ips200_draw_vertical(watch.ZebraRowL, 0, watch.ZebraRowL, 119, RGB565_PURPLE);
////                 ips200_draw_vertical(watch.ZebraRowR, 0, watch.ZebraRowR, 119, RGB565_YELLOW);
//            default:
//                break;
//        }
//    }
//    else if(tsui.img_showmode==4)//��ʾ��͸�ӱ���ͼ
//    {
//        IPS_ShowEdge((uint8_t *)imo, 188, 120);
//        memset(imo, 0, 120 * 188);
//    }


}
#pragma section all restore
