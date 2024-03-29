/*
 * vofa.c
 *
 *  Created on: 2023��3��6��
 *      Author: Admin
 */
#include "vofa.h"
#define tx_num  32
unsigned char count=0;
unsigned char DataToSend[100];

//�����븡����ת��Ϊ�޷����ַ��ͱ������
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define VOFA_UART UART_0            //vofa����������

void send_adjust_to_vofa(float*anjust)
{
    SendDataToVofa(anjust[0],anjust[1],anjust[2],anjust[3],anjust[4],anjust[5],anjust[6],anjust[7],anjust[8]);
}
//�������ݵĺ�����һ�η���1������,һ��������жϣ����Խ�ʡʱ��
//��������ѡ�� JustFloat
void Send_one_DataToVofa(float _a)
{
    unsigned char _cnt=0;
    //����Ϊ�Ѹ�����ת��Ϊ�ĸ��޷����ַ����ݵĹ���
    DataToSend[_cnt++]=BYTE0(_a);
    DataToSend[_cnt++]=BYTE1(_a);
    DataToSend[_cnt++]=BYTE2(_a);
    DataToSend[_cnt++]=BYTE3(_a);
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x80;
    DataToSend[_cnt++]=0x7f;

    vofa_uart_send_data(DataToSend,_cnt);//���ڷ������ݺ���
    system_delay_us(500);//�ӳٺ�������ֹ����
}

//�������ݵĺ�����һ�η���1������,һ��������жϣ����Խ�ʡʱ��
//��������ѡ�� JustFloat
void Send_two_DataToVofa(float _a,float _b)
{
    unsigned char _cnt=0;
    //����Ϊ�Ѹ�����ת��Ϊ�ĸ��޷����ַ����ݵĹ���
    DataToSend[_cnt++]=BYTE0(_a);
    DataToSend[_cnt++]=BYTE1(_a);
    DataToSend[_cnt++]=BYTE2(_a);
    DataToSend[_cnt++]=BYTE3(_a);
    DataToSend[_cnt++]=BYTE0(_b);
    DataToSend[_cnt++]=BYTE1(_b);
    DataToSend[_cnt++]=BYTE2(_b);
    DataToSend[_cnt++]=BYTE3(_b);
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x80;
    DataToSend[_cnt++]=0x7f;

    vofa_uart_send_data(DataToSend,_cnt);//���ڷ������ݺ���
    system_delay_us(500);//�ӳٺ�������ֹ����
}
//�������ݵĺ�����һ�η���4������
//��������ѡ�� JustFloat
void Send_four_DataToVofa(float _a,float _b,float _c,float _d)
{
    unsigned char _cnt=0;
    DataToSend[_cnt++]=BYTE0(_a);
    DataToSend[_cnt++]=BYTE1(_a);
    DataToSend[_cnt++]=BYTE2(_a);
    DataToSend[_cnt++]=BYTE3(_a);
    DataToSend[_cnt++]=BYTE0(_b);
    DataToSend[_cnt++]=BYTE1(_b);
    DataToSend[_cnt++]=BYTE2(_b);
    DataToSend[_cnt++]=BYTE3(_b);
    DataToSend[_cnt++]=BYTE0(_c);
    DataToSend[_cnt++]=BYTE1(_c);
    DataToSend[_cnt++]=BYTE2(_c);
    DataToSend[_cnt++]=BYTE3(_c);
    DataToSend[_cnt++]=BYTE0(_d);
    DataToSend[_cnt++]=BYTE1(_d);
    DataToSend[_cnt++]=BYTE2(_d);
    DataToSend[_cnt++]=BYTE3(_d);
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x80;
    DataToSend[_cnt++]=0x7f;

    vofa_uart_send_data(DataToSend,_cnt);//���ڷ������ݺ���
    system_delay_us(500);//�ӳٺ�������ֹ����
}
void Send_five_DataToVofa(float _a,float _b,float _c,float _d,float _e)
{
    unsigned char _cnt=0;
    DataToSend[_cnt++]=BYTE0(_a);
    DataToSend[_cnt++]=BYTE1(_a);
    DataToSend[_cnt++]=BYTE2(_a);
    DataToSend[_cnt++]=BYTE3(_a);
    DataToSend[_cnt++]=BYTE0(_b);
    DataToSend[_cnt++]=BYTE1(_b);
    DataToSend[_cnt++]=BYTE2(_b);
    DataToSend[_cnt++]=BYTE3(_b);
    DataToSend[_cnt++]=BYTE0(_c);
    DataToSend[_cnt++]=BYTE1(_c);
    DataToSend[_cnt++]=BYTE2(_c);
    DataToSend[_cnt++]=BYTE3(_c);
    DataToSend[_cnt++]=BYTE0(_d);
    DataToSend[_cnt++]=BYTE1(_d);
    DataToSend[_cnt++]=BYTE2(_d);
    DataToSend[_cnt++]=BYTE3(_e);
    DataToSend[_cnt++]=BYTE1(_e);
    DataToSend[_cnt++]=BYTE2(_e);
    DataToSend[_cnt++]=BYTE3(_e);
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x80;
    DataToSend[_cnt++]=0x7f;

    vofa_uart_send_data(DataToSend,_cnt);//���ڷ������ݺ���
    //system_delay_us(500);//�ӳٺ�������ֹ����
}
//�������ݵĺ�����һ�η���9������
//��������ѡ�� JustFloat
void SendDataToVofa(float _a,float _b,float _c,float _d,float _e,float _f,float _g,float _h,float _i)
{
    unsigned char _cnt=0;
    //����Ϊ�Ѹ�����ת��Ϊ�ĸ��޷����ַ����ݵĹ���
    DataToSend[_cnt++]=BYTE0(_a);
    DataToSend[_cnt++]=BYTE1(_a);
    DataToSend[_cnt++]=BYTE2(_a);
    DataToSend[_cnt++]=BYTE3(_a);
    DataToSend[_cnt++]=BYTE0(_b);
    DataToSend[_cnt++]=BYTE1(_b);
    DataToSend[_cnt++]=BYTE2(_b);
    DataToSend[_cnt++]=BYTE3(_b);
    DataToSend[_cnt++]=BYTE0(_c);
    DataToSend[_cnt++]=BYTE1(_c);
    DataToSend[_cnt++]=BYTE2(_c);
    DataToSend[_cnt++]=BYTE3(_c);
    DataToSend[_cnt++]=BYTE0(_d);
    DataToSend[_cnt++]=BYTE1(_d);
    DataToSend[_cnt++]=BYTE2(_d);
    DataToSend[_cnt++]=BYTE3(_d);
    DataToSend[_cnt++]=BYTE0(_e);
    DataToSend[_cnt++]=BYTE1(_e);
    DataToSend[_cnt++]=BYTE2(_e);
    DataToSend[_cnt++]=BYTE3(_e);
    DataToSend[_cnt++]=BYTE0(_f);
    DataToSend[_cnt++]=BYTE1(_f);
    DataToSend[_cnt++]=BYTE2(_f);
    DataToSend[_cnt++]=BYTE3(_f);
    DataToSend[_cnt++]=BYTE0(_g);
    DataToSend[_cnt++]=BYTE1(_g);
    DataToSend[_cnt++]=BYTE2(_g);
    DataToSend[_cnt++]=BYTE3(_g);
    DataToSend[_cnt++]=BYTE0(_h);
    DataToSend[_cnt++]=BYTE1(_h);
    DataToSend[_cnt++]=BYTE2(_h);
    DataToSend[_cnt++]=BYTE3(_h);
    DataToSend[_cnt++]=BYTE0(_i);
    DataToSend[_cnt++]=BYTE1(_i);
    DataToSend[_cnt++]=BYTE2(_i);
    DataToSend[_cnt++]=BYTE3(_i);
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x00;
    DataToSend[_cnt++]=0x80;
    DataToSend[_cnt++]=0x7f;

    vofa_uart_send_data(DataToSend,_cnt);//���ڷ������ݺ���
    system_delay_us(500);//�ӳٺ�������ֹ����
}
//���ڷ������ݺ���
void vofa_uart_send_data(uint8*DataToSend, uint16 _cnt)
{
    uart_write_buffer(VOFA_UART, DataToSend, _cnt);
    //wireless_uart_send_buff(DataToSend, _cnt);
}
#define USART_SENDBUFF_MAX_BYTES    100U    //���ڷ��ͻ�������С ��λ���ֽڣ� ���ڷ�printf ���ͺ���
/*����Ϊ�� printf ���ڷ��ͺ�������vofa �޹�*/
int USARTPrintf(const char* format, ...)
{
    static char sendBuff[USART_SENDBUFF_MAX_BYTES] = { 0 };
    uint16 bytes = 0;
    va_list list;

    va_start(list, format);
    bytes = vsprintf(sendBuff, format, list);   //��������о��棬���Լ���
    va_end(list);

    vofa_uart_send_data(sendBuff,bytes);
    return bytes;
}


