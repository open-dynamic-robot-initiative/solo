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
}
