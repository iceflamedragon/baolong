#include <edgeboard_uart.h>

uint8 buff;
uint8 receive_data[30];
uint8   check=0;    //У��λ
int receive_count=0;

#define data_length  11          //һ��������ĳ���
#define adjust_length  7
#define UART_HEAD    0x42           //����֡ͷ
#define UART_TIE     0x0D           //����֡β
#define UART_TO_BORAD   UART_3

enum    UART_ADDR
{
    UART_ADDR_CARCTRL=1,        //�����г�
    UART_ADDR_PID,              //���ص��PID
    UART_ADDR_KEY=6             //����
};
union char_to_int16             // ����int16�Ĺ�����
{
        int16 uart_integer;
        uint8 urat_char[2];

};
void USART_Receive_Call_Back()
{

   uart_query_byte(UART_TO_BORAD,&buff);           //����3����忨ͨ��
////���п���
   receive_data[receive_count]=buff;
   if(receive_data[0]==UART_HEAD)//֡���ڵ�ַ
   {
       if(receive_count>=data_length)
        {
        receive_count=0;
        UART_indata();
        }
        receive_count++;
   }
}

void UART_indata(void)
{
    uint8 flash_save=0;
        if(receive_data[1]==UART_ADDR_CARCTRL)
        {
            for(int i=0;i<9;i++)
            {
                check+=receive_data[i];
            }
            if(check==receive_data[9])//У��λ
            {
                memcpy(&setpara.control_speed,&receive_data[3],4);
//                if(setpara.control_speed>0)mycar.car_running=1;
//                else mycar.car_running=0;
                memcpy(&mycar.steer_pwm,&receive_data[7],2);
            }
        }

        if(receive_data[1]==UART_ADDR_PID)//֡���ڵ�ַ
        {
            for(int i=0;i<9;i++)
            {
            check+=receive_data[i];
            }
            if(check==receive_data[9])//У��λ
            {
                memcpy(&setpara.com_speed_PID.kp,&receive_data[2],2);
                memcpy(&setpara.com_speed_PID.ki,&receive_data[4],2);
                flash_save=receive_data[8];
                if(flash_save)TSUI_FlashSave();
            }
        }
}


//
//void UART_PID_adjust(void)
//{
//    if(receive_data[0]==0xAA&&receive_data[adjust_length-1]==0xDD)//֡ͷ֡β
//    {
//        memcpy(&setpara.com_turn_PID.kp,&receive_data[1],2);
//        memcpy(&setpara.com_turn_PID.kd,&receive_data[3],2);
//        memcpy(&flash,&receive_data[5],1);
//        if(flash==1)TSUI_FlashSave();
//    }
//
//}
void UART_send_car_start(void)
{
   uint8  length,check=0;   //֡��
   length=5;
   check=UART_ADDR_KEY+length;
   uart_write_byte(UART_TO_BORAD,UART_HEAD);//֡ͷ
   uart_write_byte(UART_TO_BORAD,UART_ADDR_KEY);//����
   uart_write_byte(UART_TO_BORAD,length);  //֡��
   uart_write_byte(UART_TO_BORAD,length);
   uart_write_byte(UART_TO_BORAD,check); //У��λ
}
void UART_send_int16(int16 send_int16)
{
   union char_to_int16 senddata;
   senddata.uart_integer=send_int16;
   uart_write_byte(UART_TO_BORAD,senddata.urat_char[0]);
   uart_write_byte(UART_TO_BORAD,senddata.urat_char[1]);

}
void UART_send_data(void)
{
    UART_send_int16(mycar.speed);
    Send_one_DataToVofa(mycar.present_speed);
//    uart_write_byte(UART_0, (mycar.speed>>24));
//    uart_write_byte(UART_0, (mycar.speed>>16));
//    uart_write_byte(UART_0, (mycar.speed>>8));
//    uart_write_byte(UART_0, (mycar.speed>>0));
}



