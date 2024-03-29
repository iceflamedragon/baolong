/*
 * GUI.c
 *
 *  Created on: 2023年6月20日
 *      Author: Admin
 */
#include "GUI.h"

TSUI_TypeDef tsui;
uint8 Get_maxpage(struct paralist_s* adjust_paralist)//获取可调整参数的总数
{
    u16 i=0;
    while(adjust_paralist[i].precision > 0)i++;
    return i - 1;    //由于参数编号从0开始，所以要减1
}


void UI_init()//UI初始化
{
    tsui.paraMax = Get_maxpage(&paralist);//获取参数总数
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
  GUI_flash_erase_page(EEPROM_SAVE_SECTOR);          //擦除主储存单元（在写入flash前要先擦除整页flash）
  GUI_flash_write_page(EEPROM_SAVE_SECTOR,data_to_write,sizeof(data_to_write));//写入数据
  for( i = 0 ; i < (tsui.paraMax+1) ; i ++){        //计算主储存单元的已有的数据量
      if( data_to_write[i])
          para_count ++;
  }
  if( para_count > (tsui.paraMax+1) / 3)//认定为主储存单元数据没有被擦除
  {
      GUI_flash_erase_page(EEPROM_BACKUP_SECTOR);        //若主储存单元数据已经填入，主储存安全，则擦除备份储存单元
      GUI_flash_write_page(EEPROM_BACKUP_SECTOR,data_to_write,sizeof(data_to_write));//将数据存入备份储存单元，若主储存单元在发车时复位，下次开机可以调用备份储存单元的数据，防止数据丢失
  }
  para_count=0;
  GUI_flash_read_page(EEPROM_SAVE_SECTOR,data_to_read,(tsui.paraMax+1));
  }
  else if(tsui.para_page==1)
  {
      for( i=0;i<(tsui.paraMax2+1);i++)
        data_to_write2[i] = *paralist2[i].para;
      GUI_flash_erase_page(EEPROM2_SAVE_SECTOR);          //擦除主储存单元（在写入flash前要先擦除整页flash）
      GUI_flash_write_page(EEPROM2_SAVE_SECTOR,data_to_write2,sizeof(data_to_write));//写入数据
      for( i = 0 ; i < (tsui.paraMax2+1) ; i ++){        //计算主储存单元的已有的数据量
          if( data_to_write2[i])
              para_count ++;
      }
      if( para_count > (tsui.paraMax2+1) / 3)//认定为主储存单元数据没有被擦除
      {
          GUI_flash_erase_page(EEPROM2_BACKUP_SECTOR);        //若主储存单元数据已经填入，主储存安全，则擦除备份储存单元
          GUI_flash_write_page(EEPROM2_BACKUP_SECTOR,data_to_write2,sizeof(data_to_write));//将数据存入备份储存单元，若主储存单元在发车时复位，下次开机可以调用备份储存单元的数据，防止数据丢失
      }
      para_count=0;
      GUI_flash_read_page(EEPROM2_SAVE_SECTOR,data_to_read2,(tsui.paraMax2+1));
    }
  }
/*    for( i =tsui.paraMax+1 ; i>0; i --) //计算主储存单元的已有的数据量
    {
        if( data_to_read[i]==0)
            para_count ++;
        else
            break;
    }
    if(para_count<10)//认定为主储存单元数据没有被擦除
    {
        GUI_flash_erase_page(EEPROM_BACKBACKUP_SECTOR);        //若主储存单元数据已经填入，主储存安全，则擦除备份储存单元
        GUI_flash_write_page(EEPROM_BACKBACKUP_SECTOR,data_to_write,sizeof(data_to_write));//将数据存入备份储存单元，若主储存单元在发车时复位，下次开机可以调用备份储存单元的数据，防止数据丢失
    }*/


    //else TSUI_FlashRead();

#define Data_mode 0  //0:正常模式 1：读取二级备用数据
void TSUI_FlashRead()  //将存入flash的参数读取出来
{
  //定义参数读取
    u32 i;
    u32 para_count = 0;
    u32 data_to_read[100];
    u32 data_to_read2[100];
if(Data_mode==0)
{
    GUI_flash_read_page(EEPROM_SAVE_SECTOR,data_to_read,(tsui.paraMax+1));
  for( i = 0 ; i < (tsui.paraMax+1) ; i ++)//计算主储存单元的已有的数据量
  {
      if( data_to_read[i])
          para_count ++;
  }

  if( para_count > (tsui.paraMax+1) / 5)//认定为主储存单元数据没有被擦除
  {
      for( i=0;i<(tsui.paraMax+1);i++)
            *paralist[i].para = data_to_read[i];
  }
  else{              //主储存单元数据被擦除了

      GUI_flash_read_page(EEPROM_BACKUP_SECTOR,data_to_read,(tsui.paraMax+1));//读取备份储存单元数据
      for( i=0;i<(tsui.paraMax+1);i++)
          *paralist[i].para = data_to_read[i];
  }
  para_count=0;
  GUI_flash_read_page(EEPROM2_SAVE_SECTOR,data_to_read2,(tsui.paraMax2+1));
    for( i = 0 ; i < (tsui.paraMax2+1) ; i ++)//计算主储存单元的已有的数据量
    {
        if( data_to_read2[i])
            para_count ++;
    }

    if( para_count > (tsui.paraMax2+1) / 5)//认定为主储存单元数据没有被擦除
    {
        for( i=0;i<(tsui.paraMax2+1);i++)
              *paralist2[i].para = data_to_read2[i];
    }
    else{              //主储存单元数据被擦除了

        GUI_flash_read_page(EEPROM2_BACKUP_SECTOR,data_to_read2,(tsui.paraMax2+1));//读取备份储存单元数据
        for( i=0;i<(tsui.paraMax2+1);i++)
            *paralist2[i].para = data_to_read2[i];
    }

}


/*else if(Data_mode==1)
{
    GUI_flash_read_page(EEPROM_BACKBACKUP_SECTOR,data_to_read,(tsui.paraMax+1));//读取备份储存单元数据
    for( i=0;i<(tsui.paraMax+1);i++)
        *paralist[i].para = data_to_read[i];
}*/

}




void TSUI_ButtonEvent()//如果同时有按键与旋转编码器可以改这里的代码以添加功能
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
  case OK:                        //key3按下（拨轮按下）
      if(tsui.img_showmode==4)tsui.img_showmode=0;
      else tsui.img_showmode++;
    break;
  case DOWN:                     //key2按下（拨轮下）
      if(tsui.flag_page==IPS_display_max_page-1)tsui.flag_page=0;
      else tsui.flag_page++;
    break;
  case UP:                    //key1按下（拨轮上拨）
      if(tsui.flag_page==0)tsui.flag_page=IPS_display_max_page-1;
      else tsui.flag_page--;
      break;

  case PRESS:
      tsui.paraStatus =!tsui.paraStatus ;    //状态取反，参数选择和参数改变切换
      break;

    case CW:                //如果是顺时针旋转
            if(tsui.paraStatus)//参数改变模式     注：这里的“参数改变“与“参数选择” 均为模式的名称，不是名词加动词
                *adjust_paralist[tsui.paraSelect].para += adjust_paralist[tsui.paraSelect].precision;//让所选中的参数加上其调整精度
            else            //参数选择模式
            {
                if(tsui.paraSelect < para_num )//如果当前选中的参数的序号小于参数总数
                    tsui.paraSelect ++;             //选择下一个参数
                else
                    tsui.paraSelect = 0;            //回到第一个参数
            }
   break;
  case CCW:           //如果是逆时针旋转
    if(tsui.paraStatus)  //参数改变模式
      *adjust_paralist[tsui.paraSelect].para -= adjust_paralist[tsui.paraSelect].precision;//让所选中的参数减去其调整精度
    else                    //参数选择模式
    {
      if(tsui.paraSelect > 0)                   //如果当前选中的参数的序号大于0
        tsui.paraSelect --;                     //选择上一个参数
      else
        tsui.paraSelect = para_num;         //直接选中最后一个参数
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

  tsui.buttonevent = NONE;//清除指令

}


