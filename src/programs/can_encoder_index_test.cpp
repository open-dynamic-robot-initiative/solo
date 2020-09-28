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
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>

#include <blmc_robots/blmc_joint_module.hpp>

class EncoderIndexTester
{
public:
    static constexpr double torque_constant_NmpA = 0.02;
    static constexpr double gear_ratio = 9.0;
    static constexpr double max_current_A = 2.0;
    static constexpr double one_motor_rotation_distance = 2.0 * M_PI / gear_ratio;

    EncoderIndexTester(const std::string &can_port,
                       int motor_index,
                       double torque,
                       unsigned int number_of_revolutions)
        : torque_(torque)
    {
        std::cout << "CAN port: " << can_port << std::endl;
        std::cout << "Motor Index: " << motor_index << std::endl;
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
            std::make_shared<blmc_drivers::Motor>(motor_board, motor_index);

        joint_module_ = std::make_unique<blmc_robots::BlmcJointModule>(
            motor, torque_constant_NmpA, gear_ratio, 0, false, max_current_A);
    }

    void run()
    {
        std::cout << "Start moving with constant torque" << std::endl;
        real_time_tools::Spinner spinner;
        spinner.set_period(0.001);
        double last_index_position = joint_module_->get_measured_index_angle();
        unsigned int counter = 0;

        // Do not print, rotate 100 times, count index ticks
        // check after each rotation if tick was found

        while (true)
        {
            joint_module_->set_torque(torque_);
            joint_module_->send_torque();

            double index_position = joint_module_->get_measured_index_angle();

            if (!std::isnan(index_position) &&
                index_position != last_index_position)
            {
                counter++;
                double diff = index_position - last_index_position;
                std::cout << "Found Encoder Index " << counter
                          << ".\tPosition: " << index_position
                          << ".\tDiff to last: " << diff << std::endl;

                if (std::abs(std::abs(diff) - one_motor_rotation_distance) >
                    0.01)
                {
                    std::cout << std::endl
                              << "!!!! DISTANCE TO LAST INDEX DOES NOT MATCH "
                                 "ONE REVOLUTION !!!!"
                              << std::endl
                              << std::endl;
                    exit(2);
                }

                last_index_position = index_position;
            }
            spinner.spin();
        }
    }

private:
    double torque_;
    std::unique_ptr<blmc_robots::BlmcJointModule> joint_module_;
};


int main(int argc, char *argv[])
{
    if (argc != 5)
    {
        std::cout << "Invalid number of arguments." << std::endl;
        std::cout
            << "Usage: " << argv[0]
            << " <can port> <motor index> <torque> <number of revolutions>"
            << std::endl;
        return 1;
    }

    std::string can_port = argv[1];
    int motor_index = std::stoi(argv[2]);
    double torque = std::stod(argv[3]);
    int num_revolutions = std::stoi(argv[4]);

    if (motor_index != 0 && motor_index != 1)
    {
        std::cout << "Invalid motor index.  Only '0' and '1' are allowed."
                  << std::endl;
        return 1;
    }

    EncoderIndexTester tester(can_port, motor_index, torque, num_revolutions);

    // run in real-time thread
    real_time_tools::RealTimeThread thread;
    thread.create_realtime_thread(
        [](void *instance_pointer) {
            ((EncoderIndexTester *)(instance_pointer))->run();
            return (void *)nullptr;
        },
        &tester);
    thread.join();

    return 0;
}
