/*
 * GUI.c
 *
 *  Created on: 2023��6��20��
 *      Author: Admin
 */
#include "GUI.h"

TSUI_TypeDef tsui;
uint8 Get_maxpage(struct paralist_s* adjust_paralist)//��ȡ�ɵ�������������
{
    u16 i=0;
    while(adjust_paralist[i].precision > 0)i++;
    return i - 1;    //���ڲ�����Ŵ�0��ʼ������Ҫ��1
}


void UI_init()//UI��ʼ��
{
    tsui.paraMax = Get_maxpage(&paralist);//��ȡ��������
    tsui.paraMax2= Get_maxpage(&paralist2);
    //TSUI_FlashRead();
}

void TSUI_FlashSave()
{
  u32 data_to_write[100];
  u32 data_to_read[100];
  u32 data_to_write2[100];
  u32 data_to_read2[100];
  u32 para_count = 0;
  u8 i;
  if(tsui.para_page==0)
  {
  for( i=0;i<(tsui.paraMax+1);i++)
    data_to_write[i] = *paralist[i].para;
  GUI_flash_erase_page(EEPROM_SAVE_SECTOR);          //���������浥Ԫ����д��flashǰҪ�Ȳ�����ҳflash��
  GUI_flash_write_page(EEPROM_SAVE_SECTOR,data_to_write,sizeof(data_to_write));//д������
  for( i = 0 ; i < (tsui.paraMax+1) ; i ++){        //���������浥Ԫ�����е�������
      if( data_to_write[i])
          para_count ++;
  }
  if( para_count > (tsui.paraMax+1) / 3)//�϶�Ϊ�����浥Ԫ����û�б�����
  {
      GUI_flash_erase_page(EEPROM_BACKUP_SECTOR);        //�������浥Ԫ�����Ѿ����룬�����氲ȫ����������ݴ��浥Ԫ
      GUI_flash_write_page(EEPROM_BACKUP_SECTOR,data_to_write,sizeof(data_to_write));//�����ݴ��뱸�ݴ��浥Ԫ���������浥Ԫ�ڷ���ʱ��λ���´ο������Ե��ñ��ݴ��浥Ԫ�����ݣ���ֹ���ݶ�ʧ
  }
  para_count=0;
  GUI_flash_read_page(EEPROM_SAVE_SECTOR,data_to_read,(tsui.paraMax+1));
  }
  else if(tsui.para_page==1)
  {
      for( i=0;i<(tsui.paraMax2+1);i++)
        data_to_write2[i] = *paralist2[i].para;
      GUI_flash_erase_page(EEPROM2_SAVE_SECTOR);          //���������浥Ԫ����д��flashǰҪ�Ȳ�����ҳflash��
      GUI_flash_write_page(EEPROM2_SAVE_SECTOR,data_to_write2,sizeof(data_to_write));//д������
      for( i = 0 ; i < (tsui.paraMax2+1) ; i ++){        //���������浥Ԫ�����е�������
          if( data_to_write2[i])
              para_count ++;
      }
      if( para_count > (tsui.paraMax2+1) / 3)//�϶�Ϊ�����浥Ԫ����û�б�����
      {
          GUI_flash_erase_page(EEPROM2_BACKUP_SECTOR);        //�������浥Ԫ�����Ѿ����룬�����氲ȫ����������ݴ��浥Ԫ
          GUI_flash_write_page(EEPROM2_BACKUP_SECTOR,data_to_write2,sizeof(data_to_write));//�����ݴ��뱸�ݴ��浥Ԫ���������浥Ԫ�ڷ���ʱ��λ���´ο������Ե��ñ��ݴ��浥Ԫ�����ݣ���ֹ���ݶ�ʧ
      }
      para_count=0;
      GUI_flash_read_page(EEPROM2_SAVE_SECTOR,data_to_read2,(tsui.paraMax2+1));
    }
  }
/*    for( i =tsui.paraMax+1 ; i>0; i --) //���������浥Ԫ�����е�������
    {
        if( data_to_read[i]==0)
            para_count ++;
        else
            break;
    }
    if(para_count<10)//�϶�Ϊ�����浥Ԫ����û�б�����
    {
        GUI_flash_erase_page(EEPROM_BACKBACKUP_SECTOR);        //�������浥Ԫ�����Ѿ����룬�����氲ȫ����������ݴ��浥Ԫ
        GUI_flash_write_page(EEPROM_BACKBACKUP_SECTOR,data_to_write,sizeof(data_to_write));//�����ݴ��뱸�ݴ��浥Ԫ���������浥Ԫ�ڷ���ʱ��λ���´ο������Ե��ñ��ݴ��浥Ԫ�����ݣ���ֹ���ݶ�ʧ
    }*/


    //else TSUI_FlashRead();

#define Data_mode 0  //0:����ģʽ 1����ȡ������������
void TSUI_FlashRead()  //������flash�Ĳ�����ȡ����
{
  //���������ȡ
    u32 i;
    u32 para_count = 0;
    u32 data_to_read[100];
    u32 data_to_read2[100];
if(Data_mode==0)
{
    GUI_flash_read_page(EEPROM_SAVE_SECTOR,data_to_read,(tsui.paraMax+1));
  for( i = 0 ; i < (tsui.paraMax+1) ; i ++)//���������浥Ԫ�����е�������
  {
      if( data_to_read[i])
          para_count ++;
  }

  if( para_count > (tsui.paraMax+1) / 5)//�϶�Ϊ�����浥Ԫ����û�б�����
  {
      for( i=0;i<(tsui.paraMax+1);i++)
            *paralist[i].para = data_to_read[i];
  }
  else{              //�����浥Ԫ���ݱ�������

      GUI_flash_read_page(EEPROM_BACKUP_SECTOR,data_to_read,(tsui.paraMax+1));//��ȡ���ݴ��浥Ԫ����
      for( i=0;i<(tsui.paraMax+1);i++)
          *paralist[i].para = data_to_read[i];
  }
  para_count=0;
  GUI_flash_read_page(EEPROM2_SAVE_SECTOR,data_to_read2,(tsui.paraMax2+1));
    for( i = 0 ; i < (tsui.paraMax2+1) ; i ++)//���������浥Ԫ�����е�������
    {
        if( data_to_read2[i])
            para_count ++;
    }

    if( para_count > (tsui.paraMax2+1) / 5)//�϶�Ϊ�����浥Ԫ����û�б�����
    {
        for( i=0;i<(tsui.paraMax2+1);i++)
              *paralist2[i].para = data_to_read2[i];
    }
    else{              //�����浥Ԫ���ݱ�������

        GUI_flash_read_page(EEPROM2_BACKUP_SECTOR,data_to_read2,(tsui.paraMax2+1));//��ȡ���ݴ��浥Ԫ����
        for( i=0;i<(tsui.paraMax2+1);i++)
            *paralist2[i].para = data_to_read2[i];
    }

}


/*else if(Data_mode==1)
{
    GUI_flash_read_page(EEPROM_BACKBACKUP_SECTOR,data_to_read,(tsui.paraMax+1));//��ȡ���ݴ��浥Ԫ����
    for( i=0;i<(tsui.paraMax+1);i++)
        *paralist[i].para = data_to_read[i];
}*/

}




void TSUI_ButtonEvent()//���ͬʱ�а�������ת���������Ը�����Ĵ�������ӹ���
{
    struct paralist_s* adjust_paralist;
    uint8 para_num;
    if(tsui.para_page==1)
        {
            adjust_paralist=&paralist2;
            para_num=tsui.paraMax2;
        }
    else
        {
            adjust_paralist=&paralist;
            para_num=tsui.paraMax;
        }
  switch(tsui.buttonevent)
  {
  case OK:                        //key3���£����ְ��£�
      if(tsui.img_showmode==4)tsui.img_showmode=0;
      else tsui.img_showmode++;
    break;
  case DOWN:                     //key2���£������£�
      if(tsui.flag_page==IPS_display_max_page-1)tsui.flag_page=0;
      else tsui.flag_page++;
    break;
  case UP:                    //key1���£������ϲ���
      if(tsui.flag_page==0)tsui.flag_page=IPS_display_max_page-1;
      else tsui.flag_page--;
      break;

  case PRESS:
      tsui.paraStatus =!tsui.paraStatus ;    //״̬ȡ��������ѡ��Ͳ����ı��л�
      break;

    case CW:                //�����˳ʱ����ת
            if(tsui.paraStatus)//�����ı�ģʽ     ע������ġ������ı䡰�롰����ѡ�� ��Ϊģʽ�����ƣ��������ʼӶ���
                *adjust_paralist[tsui.paraSelect].para += adjust_paralist[tsui.paraSelect].precision;//����ѡ�еĲ����������������
            else            //����ѡ��ģʽ
            {
                if(tsui.paraSelect < para_num )//�����ǰѡ�еĲ��������С�ڲ�������
                    tsui.paraSelect ++;             //ѡ����һ������
                else
                    tsui.paraSelect = 0;            //�ص���һ������
            }
   break;
  case CCW:           //�������ʱ����ת
    if(tsui.paraStatus)  //�����ı�ģʽ
      *adjust_paralist[tsui.paraSelect].para -= adjust_paralist[tsui.paraSelect].precision;//����ѡ�еĲ�����ȥ���������
    else                    //����ѡ��ģʽ
    {
      if(tsui.paraSelect > 0)                   //�����ǰѡ�еĲ�������Ŵ���0
        tsui.paraSelect --;                     //ѡ����һ������
      else
        tsui.paraSelect = para_num;         //ֱ��ѡ�����һ������
    }
   break;
  case Press2:
      tsui.para_page=!tsui.para_page;
      tsui.paraSelect=0;
      tsui.paraStatus=0;
      break;
  default:
    break;
  }

  tsui.buttonevent = NONE;//���ָ��

}


