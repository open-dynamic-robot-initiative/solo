#include "test_bench_8_motors.hh"

using namespace blmc_motors
{

void TestBench8Motors::initialize()
{
  osi::initialize_realtime_printing();
  can_bus0_ = std::make_shared<blmc_drivers::CanBus>("can0");
  can_bus1_ = std::make_shared<blmc_drivers::CanBus>("can1");
  can_bus2_ = std::make_shared<blmc_drivers::CanBus>("can2");
  can_bus3_ = std::make_shared<blmc_drivers::CanBus>("can3");
  board0_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus0);
  board1_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus1);
  board2_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus2);
  board3_ = std::make_shared<blmc_drivers::CanBusMotorBoard>(can_bus3);
}

/**
 * @brief send_target_torques
 */
void TestBench8Motors::send_target_torques(
    Eigen::Matrix<double, 8, 1> target_currents)
{

}

} // namespace blmc_motors
