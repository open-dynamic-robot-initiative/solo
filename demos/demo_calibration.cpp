/**
 * \file demo_test_bench_8_motors.cpp
 * \brief The use of the wrapper implementing a small pid controller.
 * \author Maximilien Naveau
 * \date 2018
 *
 * This file uses the Quadruped class in a small demo.
 */


#include "blmc_robots/blmc_joint_module.hpp"
#include "blmc_robots/common_programs_header.hpp"


using namespace blmc_robots;


struct Robot
{
    std::shared_ptr<blmc_drivers::CanBus> can_bus;
    std::shared_ptr<blmc_drivers::CanBusMotorBoard> can_bus_motor_board;

    std::shared_ptr<blmc_drivers::AnalogSensor> slider_a;
    std::shared_ptr<blmc_drivers::AnalogSensor> slider_b;
    std::shared_ptr<blmc_drivers::MotorInterface> motor;

    std::shared_ptr<BlmcJointModule> joint_module;

    double motor_constant;
    double gear_ratio;
    double zero_angle;
    double angle_zero_to_index;
    double calibrated_index_angle;
    bool mechanical_calibration;
    bool reverse_polarity;
};

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_void_ptr)
{
    Robot& robot = *(static_cast<Robot*>(robot_void_ptr));
    robot.angle_zero_to_index = 0.134252;
    robot.calibrated_index_angle = 0.0;
    robot.mechanical_calibration = false;  // true;
    robot.joint_module->calibrate(robot.angle_zero_to_index,
                                  robot.calibrated_index_angle,
                                  robot.mechanical_calibration);

    // real_time_tools::Spinner spinner;
    // spinner.set_period(0.1);
    // while(!StopControl)
    // {
    //   rt_printf("current measurement = %f, current index = %f\n",
    //   robot.joint_module->get_measured_angle(),
    //   robot.joint_module->get_measured_index_angle()); spinner.spin();
    // }
}  // end control_loop

int main(int, char**)
{
    enable_ctrl_c()

    real_time_tools::RealTimeThread thread;

    Robot robot;
    robot.can_bus = std::make_shared<blmc_drivers::CanBus>("can0");
    robot.can_bus_motor_board =
        std::make_shared<blmc_drivers::CanBusMotorBoard>(robot.can_bus);
    robot.slider_a = std::make_shared<blmc_drivers::AnalogSensor>(
        robot.can_bus_motor_board, 0);
    robot.slider_b = std::make_shared<blmc_drivers::AnalogSensor>(
        robot.can_bus_motor_board, 1);
    robot.motor =
        std::make_shared<blmc_drivers::Motor>(robot.can_bus_motor_board, 0);
    robot.motor_constant = 0.025;
    robot.gear_ratio = 9.0;
    robot.zero_angle = 0.0;
    robot.reverse_polarity = true;
    robot.joint_module =
        std::make_shared<BlmcJointModule>(robot.motor,
                                          robot.motor_constant,
                                          robot.gear_ratio,
                                          robot.zero_angle,
                                          robot.reverse_polarity);

    robot.can_bus_motor_board->wait_until_ready();

    char str[256];

    rt_printf("Press enter to launch the calibration \n");
    std::cin.get(str, 256);  // get c-string

    rt_printf("controller is set up \n");

    thread.create_realtime_thread(&control_loop, &robot);

    rt_printf("control loop started \n");

    thread.join();

    return 0;
}
