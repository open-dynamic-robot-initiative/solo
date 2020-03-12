// From: https://github.com/crayzeewulf/libserial/blob/master/examples/serial_stream_read.cpp
//
// Installation notes:
// - install from source following github readme
// - include /usr/loca/lib:
//      export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
/**
 *  @example serial_stream_read.cpp
 */

#include <cstdlib>
#include <iostream>
#include <unistd.h>
#include <sstream>      // std::stringstream, std::stringbuf

#include "blmc_drivers/serial_reader.hpp"
#include "blmc_robots/common_programs_header.hpp"

using namespace blmc_robots;

/**
 * @brief This example demonstrates configuring a serial stream and
 *        reading serial stream data.
 */
int main(int argc, char** argv)
{
    if (argc < 2) {
        rt_printf("Usage: demo_arduino_slider serial_port. (serial_port like /dev/ttyACM0)\n");
        return -1;
    }

    std::vector<int> values;
    blmc_drivers::SerialReader serial_reader(argv[1], 4);

    enable_ctrl_c();
    while (!CTRL_C_DETECTED)
    {
        serial_reader.fill_vector(values);
        for (int i = 0; i < 4; i++) {
            std::cout << values[i] << " ";
        }
        std::cout << std::endl;

        real_time_tools::Timer::sleep_sec(0.001);
    }

    // using namespace LibSerial;

    // // Instantiate a SerialStream object.
    // SerialStream serial_stream;

    // try
    // {
    //     // Open the Serial Port at the desired hardware port.
    //     serial_stream.Open(SERIAL_PORT_1) ;
    // }
    // catch (const OpenFailed&)
    // {
    //     std::cerr << "The serial port did not open correctly." << std::endl ;
    //     return EXIT_FAILURE ;
    // }

    // // Set the baud rate of the serial port.
    // serial_stream.SetBaudRate(BaudRate::BAUD_115200) ;

    // // Set the number of data bits.
    // serial_stream.SetCharacterSize(CharacterSize::CHAR_SIZE_8) ;

    // // Turn off hardware flow control.
    // serial_stream.SetFlowControl(FlowControl::FLOW_CONTROL_NONE) ;

    // // Disable parity.
    // serial_stream.SetParity(Parity::PARITY_NONE) ;

    // // Set the number of stop bits.
    // serial_stream.SetStopBits(StopBits::STOP_BITS_1) ;

    // // Wait for data to be available at the serial port.
    // while(serial_stream.rdbuf()->in_avail() == 0)
    // {
    //     usleep(1000) ;
    // }

    // int i = 0;
    // int a0, a1, a2, a3;
    // const int buffer_size = 64;
    // char buffer[buffer_size];

    // std::stringstream ss;

    // // using stringbuf directly:
    // std::stringbuf *pbuf = ss.rdbuf();
    // while (true) {
    //     // Keep reading data from serial port and print it to the screen.
    //     while(serial_stream.IsDataAvailable())
    //     {
    //         // Variable to store data coming from the serial port.
    //         char data_byte ;

    //         // Read a single byte of data from the serial port.
    //         serial_stream.get(data_byte) ;

    //         if (data_byte == '\n') {
    //             buffer[i] = '\0';
    //             pbuf->pubsetbuf(buffer, buffer_size);

    //             char c;
    //             int value;
    //             while(ss >> value >> c) {
    //                 std::cout << value << " ";
    //             }
    //             std::cout << i;
    //             // std::cout << buffer;
    //             std::cout << std::endl;

    //             // buffer[i] = '\0';
    //             // // std::cout << buffer << std::endl;
    //             // if (sscanf(buffer, "%d %d %d %d", &a0, &a1, &a2, &a3) == 4) {
    //             //     std::cout << float(a0)/1024. << " " << float(a1)/1024. << std::endl;
    //             // } else {
    //             //     std::cout << "Failed to read all four numbers." << std::endl;
    //             // }

    //             i = 0;
    //         } else {
    //             buffer[i] = data_byte;
    //             i++;
    //         }
    //     }
    //     usleep(1000) ;
    // }

    // serial_stream.Close()


    // // Successful program completion.
    // std::cout << "Done." << std::endl ;
    // return EXIT_SUCCESS ;
}
