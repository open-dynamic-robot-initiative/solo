/**
 * \file
 * \brief Simple test tool to check if the encoder index is detected.
 *
 * Runs a motor at a constant torque and prints a line whenever the encoder
 * index is detected.
 *
 * \copyright Copyright (c) 2020 Max Planck Gesellschaft.
 */
#include <array>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#include <real_time_tools/thread.hpp>
#include <real_time_tools/spinner.hpp>

#include <blmc_robots/blmc_joint_module.hpp>


struct Params
{
    std::string can_port;
    int motor_idx;
    double torque;
};


void* foo(void *params)
{
    const std::string can_port = ((Params*)params)->can_port;
    const int motor_idx = ((Params*)params)->motor_idx;
    const double torque = ((Params*)params)->torque;

    std::cout << "CAN port: " << can_port << std::endl;
    std::cout << "Motor Index: " << motor_idx << std::endl;
    std::cout << "Torque: " << torque << std::endl;

    std::cout << "==============================" << std::endl;
    std::cout << "Initialising..." << std::endl;

    // setup can bus
    auto can_bus = std::make_shared<blmc_drivers::CanBus>(can_port);

    // set up motor board
    auto motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus, 1000, 10);
    motor_board->wait_until_ready();

    std::shared_ptr<blmc_drivers::MotorInterface> motor =
        std::make_shared<blmc_drivers::Motor>(motor_board, motor_idx);

    constexpr double torque_constant_NmpA = 0.02;
    constexpr double gear_ratio = 9.0;
    constexpr double max_current_A = 2.0;
    constexpr double one_motor_rotation_distance = 2.0 * M_PI / gear_ratio;

    blmc_robots::BlmcJointModule joint_module(
        motor, torque_constant_NmpA, gear_ratio, 0, false, max_current_A);


    std::cout << "Start moving with constant torque" << std::endl;
    real_time_tools::Spinner spinner;
    spinner.set_period(0.001);
    double last_index_position = joint_module.get_measured_index_angle();
    unsigned int counter = 0;
    while (true)
    {
        joint_module.set_torque(torque);
        joint_module.send_torque();

        double index_position = joint_module.get_measured_index_angle();

        if (!std::isnan(index_position) && index_position != last_index_position)
        {
            counter++;
            double diff = index_position - last_index_position;
            std::cout << "Found Encoder Index " << counter
                << ".\tPosition: " << index_position
                << ".\tDiff to last: " << diff
                << std::endl;

            if (std::abs(std::abs(diff) - one_motor_rotation_distance) > 0.01)
            {
                std::cout << std::endl
                    << "!!!! DISTANCE TO LAST INDEX DOES NOT MATCH ONE REVOLUTION !!!!"
                    << std::endl
                    << std::endl;
                exit(2);
            }

            last_index_position = index_position;
        }
        spinner.spin();
    }

    return nullptr;
}


int main(int argc, char* argv[])
{
    if (argc != 4)
    {
        std::cout << "Invalid number of arguments." << std::endl;
        std::cout << "Usage: " << argv[0] << " <can port> <motor index> <torque>"
                  << std::endl;
        return 1;
    }

    Params params;
    params.can_port = argv[1];
    params.motor_idx = std::stoi(argv[2]);
    params.torque = std::stod(argv[3]);

    if (params.motor_idx != 0 && params.motor_idx != 1)
    {
        std::cout << "Invalid motor index.  Only '0' and '1' are allowed."
                  << std::endl;
        return 1;
    }

    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(&foo, &params);

    while (true);

    return 0;
}
