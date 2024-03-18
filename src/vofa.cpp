
/*
 * vofa.c
 *
 *  Created on: 2023年3月6日
 *      Author: Admin
 */
#include "vofa.h"
#include <SerialPort.h>
#define tx_num 32
unsigned char count = 0;
unsigned char DataToSend[100];
using namespace LibSerial;
// 以下与浮点数转化为无符号字符型变量相关
#define BYTE0(dwTemp) (*(char *)(&dwTemp))
#define BYTE1(dwTemp) (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp) (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp) (*((char *)(&dwTemp) + 3))

void serial_init(void)
{

    try
    {
        // Open the Serial Ports at the desired hardware devices.
        serial_port.Open("/dev/ttyUSB0");
    }
    catch (const OpenFailed &)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl;
        return EXIT_FAILURE;
    }

    using LibSerial::BaudRate;
    serial_port.SetBaudRate(BaudRate::BAUD_115200);

    // Set the number of data bits.
    serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

    // Turn off hardware flow control.
    serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

    // Disable parity.
    serial_port.SetParity(Parity::PARITY_NONE);

    // Set the number of stop bits.
    serial_port.SetStopBits(StopBits::STOP_BITS_1);
}
void send_adjust_to_vofa(float *anjust)
{
    SendDataToVofa(anjust[0], anjust[1], anjust[2], anjust[3], anjust[4], anjust[5], anjust[6], anjust[7], anjust[8]);
}
// 发送数据的函数，一次发送1个数据,一般可用于中断，可以节省时间
// 数据引擎选择 JustFloat
void Send_one_DataToVofa(float _a)
{
    unsigned char _cnt = 0;
    // 以下为把浮点数转化为四个无符号字符数据的过程
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    DataToSend[_cnt++] = BYTE2(_a);
    DataToSend[_cnt++] = BYTE3(_a);
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x80;
    DataToSend[_cnt++] = 0x7f;

    vofa_uart_send_data(DataToSend, _cnt); // 串口发送数据函数
    sleep(5)                 // 延迟函数，防止死机
}

// 发送数据的函数，一次发送1个数据,一般可用于中断，可以节省时间
// 数据引擎选择 JustFloat
void Send_two_DataToVofa(float _a, float _b)
{
    unsigned char _cnt = 0;
    // 以下为把浮点数转化为四个无符号字符数据的过程
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    DataToSend[_cnt++] = BYTE2(_a);
    DataToSend[_cnt++] = BYTE3(_a);
    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);
    DataToSend[_cnt++] = BYTE2(_b);
    DataToSend[_cnt++] = BYTE3(_b);
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x80;
    DataToSend[_cnt++] = 0x7f;

    vofa_uart_send_data(DataToSend, _cnt); // 串口发送数据函数
    sleep(5)                 // 延迟函数，防止死机
}
// 发送数据的函数，一次发送4个数据
// 数据引擎选择 JustFloat
void Send_four_DataToVofa(float _a, float _b, float _c, float _d)
{
    unsigned char _cnt = 0;
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    DataToSend[_cnt++] = BYTE2(_a);
    DataToSend[_cnt++] = BYTE3(_a);
    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);
    DataToSend[_cnt++] = BYTE2(_b);
    DataToSend[_cnt++] = BYTE3(_b);
    DataToSend[_cnt++] = BYTE0(_c);
    DataToSend[_cnt++] = BYTE1(_c);
    DataToSend[_cnt++] = BYTE2(_c);
    DataToSend[_cnt++] = BYTE3(_c);
    DataToSend[_cnt++] = BYTE0(_d);
    DataToSend[_cnt++] = BYTE1(_d);
    DataToSend[_cnt++] = BYTE2(_d);
    DataToSend[_cnt++] = BYTE3(_d);
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x80;
    DataToSend[_cnt++] = 0x7f;

    vofa_uart_send_data(DataToSend, _cnt); // 串口发送数据函数
    sleep(5)                 // 延迟函数，防止死机
}
void Send_five_DataToVofa(float _a, float _b, float _c, float _d, float _e)
{
    unsigned char _cnt = 0;
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    DataToSend[_cnt++] = BYTE2(_a);
    DataToSend[_cnt++] = BYTE3(_a);
    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);
    DataToSend[_cnt++] = BYTE2(_b);
    DataToSend[_cnt++] = BYTE3(_b);
    DataToSend[_cnt++] = BYTE0(_c);
    DataToSend[_cnt++] = BYTE1(_c);
    DataToSend[_cnt++] = BYTE2(_c);
    DataToSend[_cnt++] = BYTE3(_c);
    DataToSend[_cnt++] = BYTE0(_d);
    DataToSend[_cnt++] = BYTE1(_d);
    DataToSend[_cnt++] = BYTE2(_d);
    DataToSend[_cnt++] = BYTE3(_e);
    DataToSend[_cnt++] = BYTE1(_e);
    DataToSend[_cnt++] = BYTE2(_e);
    DataToSend[_cnt++] = BYTE3(_e);
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x80;
    DataToSend[_cnt++] = 0x7f;

    vofa_uart_send_data(DataToSend, _cnt); // 串口发送数据函数
    // system_delay_us(500);//延迟函数，防止死机
}
// 发送数据的函数，一次发送9个数据
// 数据引擎选择 JustFloat
void SendDataToVofa(float _a, float _b, float _c, float _d, float _e, float _f, float _g, float _h, float _i)
{
    unsigned char _cnt = 0;
    // 以下为把浮点数转化为四个无符号字符数据的过程
    DataToSend[_cnt++] = BYTE0(_a);
    DataToSend[_cnt++] = BYTE1(_a);
    DataToSend[_cnt++] = BYTE2(_a);
    DataToSend[_cnt++] = BYTE3(_a);
    DataToSend[_cnt++] = BYTE0(_b);
    DataToSend[_cnt++] = BYTE1(_b);
    DataToSend[_cnt++] = BYTE2(_b);
    DataToSend[_cnt++] = BYTE3(_b);
    DataToSend[_cnt++] = BYTE0(_c);
    DataToSend[_cnt++] = BYTE1(_c);
    DataToSend[_cnt++] = BYTE2(_c);
    DataToSend[_cnt++] = BYTE3(_c);
    DataToSend[_cnt++] = BYTE0(_d);
    DataToSend[_cnt++] = BYTE1(_d);
    DataToSend[_cnt++] = BYTE2(_d);
    DataToSend[_cnt++] = BYTE3(_d);
    DataToSend[_cnt++] = BYTE0(_e);
    DataToSend[_cnt++] = BYTE1(_e);
    DataToSend[_cnt++] = BYTE2(_e);
    DataToSend[_cnt++] = BYTE3(_e);
    DataToSend[_cnt++] = BYTE0(_f);
    DataToSend[_cnt++] = BYTE1(_f);
    DataToSend[_cnt++] = BYTE2(_f);
    DataToSend[_cnt++] = BYTE3(_f);
    DataToSend[_cnt++] = BYTE0(_g);
    DataToSend[_cnt++] = BYTE1(_g);
    DataToSend[_cnt++] = BYTE2(_g);
    DataToSend[_cnt++] = BYTE3(_g);
    DataToSend[_cnt++] = BYTE0(_h);
    DataToSend[_cnt++] = BYTE1(_h);
    DataToSend[_cnt++] = BYTE2(_h);
    DataToSend[_cnt++] = BYTE3(_h);
    DataToSend[_cnt++] = BYTE0(_i);
    DataToSend[_cnt++] = BYTE1(_i);
    DataToSend[_cnt++] = BYTE2(_i);
    DataToSend[_cnt++] = BYTE3(_i);
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x00;
    DataToSend[_cnt++] = 0x80;
    DataToSend[_cnt++] = 0x7f;

    vofa_uart_send_data(DataToSend, _cnt); // 串口发送数据函数
    sleep(5)                 // 延迟函数，防止死机
}
// 串口发送数据函数
void vofa_uart_send_data(uint8 *DataToSend, uint16 _cnt)
{
    uart_write_buffer(UART_0, DataToSend, _cnt);
    // wireless_uart_send_buff(DataToSend, _cnt);
}
#define USART_SENDBUFF_MAX_BYTES 100U // 串口发送缓冲区大小 单位：字节， 用于仿printf 发送函数
/*以下为仿 printf 串口发送函数，与vofa 无关*/

// 函数简介       串口发送数组
// 参数说明       uart_n          串口模块号 参照 zf_driver_uart.h 内 uart_index_enum 枚举体定义
// 参数说明       *buff           要发送的数组地址
// 参数说明       len             发送长度
// 返回参数       void
// 使用示例       uart_write_buffer(UART_1, &a[0], 5);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void uart_write_buffer(uart_index_enum uart_n, const uint8 *buff, uint32 len)
{
    while (len)
    {

        serial_port.WriteByte(*buff);
        len--;
        buff++;
    }
}
