#include <SerialPort.h>
#include <iostream>
using namespace std;
using namespace LibSerial;
extern SerialPort serial_port;
bool serial_init(void)
{
    try
    {
        // Open the Serial Ports at the desired hardware devices.
        serial_port.Open("/dev/ttyUSB0");
    }
    catch (const OpenFailed &)
    {
        std::cerr << "The serial ports did not open correctly." << std::endl;
        return 0;
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
    return 0;
}

char serial_read(void)
{
    // Specify a timeout value (in milliseconds).
    size_t timeout_milliseconds = 5000000;
    char read_byte_1 = '0';
    using LibSerial::ReadTimeout;
    try
    {
        // Read a byte from the serial port using SerialPort Read() methods.
        serial_port.ReadByte(read_byte_1, timeout_milliseconds);
    }
    catch (const ReadTimeout &)
    {
        std::cerr << "The Read() call has timed out." << std::endl;
        return 0;
    }
    return read_byte_1;
}
