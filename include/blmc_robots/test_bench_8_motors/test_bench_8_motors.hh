#ifndef TEST_BENCH_8_MOTORS_HH
#define TEST_BENCH_8_MOTORS_HH

#include<blmc_robots/common_header.hh>

using namespace blmc_motors
{

class TestBench8Motors
{
public:
  /**
   * @brief TestBench8Motors
   */
  TestBench8Motors()
  {
    max_current_ = 1.0 ;
  }

  /**
   * @brief initialize the robot by setting alining the motors and calibrate the
   * sensors to 0
   */
  void initialize();

  /**
   * @brief send_target_torques
   */
  void send_target_torques(Eigen::Matrix<double, 8, 1> target_currents);

  /**
   * @brief get_motor_positions
   */
  Eigen::Ref<Eigen::Matrix<double, 8, 1> > get_motor_positions()
  {
    return motor_positions_;
  }

  /**
   * @brief get_motor_velocities
   */
  Eigen::Ref<Eigen::Matrix<double, 8, 1> > void get_motor_velocities()
  {
    return motor_velocities_;
  }

  /**
   * @brief get_max_current
   * @return the maximum current to apply
   */
  double get_max_current()
  {
    return max_current_;
  }

private:

  Eigen::Matrix<double, 8, 1> motor_positions_;
  Eigen::Matrix<double, 8, 1> motor_velocities_;
  Eigen::Matrix<double, 8, 1> target_currents;


  std::shared_ptr<blmc_drivers::CanBus> can_bus0_;
  std::shared_ptr<blmc_drivers::CanBus> can_bus1_;
  std::shared_ptr<blmc_drivers::CanBus> can_bus2_;
  std::shared_ptr<blmc_drivers::CanBus> can_bus3_;

  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board0_;
  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board1_;
  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board2_;
  std::shared_ptr<blmc_drivers::CanBusMotorBoard> board3_;

  double max_current_ ;

  board0_
  board1_
  board2_
  board3_

};

} // namespace blmc_motors

#endif // TEST_BENCH_8_MOTORS_HH
