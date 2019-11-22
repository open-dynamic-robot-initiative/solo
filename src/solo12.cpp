#include <cmath>
#include "real_time_tools/spinner.hpp"
#include "blmc_robots/solo12.hpp"

namespace blmc_robots{

const double Solo12::max_joint_torque_security_margin_ = 0.99;

Solo12::Solo12()
{
  /**
   * Hardware properties
   */
  motor_inertias_.setZero();
  motor_torque_constants_.setZero();
  joint_gear_ratios_.setZero();
  motor_max_current_.setZero();
  max_joint_torques_.setZero();
  joint_zero_positions_.setZero();
  reverse_polarities_.fill(false);

  /**
   * Hardware status
   */
  for(unsigned i=0 ; i<motor_enabled_.size(); ++i)
  {
    motor_enabled_[i] = false;
    motor_ready_[i] = false;
  }
  for(unsigned i=0 ; i<motor_board_enabled_.size(); ++i)
  {
    motor_board_enabled_[0] = false;
    motor_board_errors_[0] = 0;
  }

  /**
   * Joint data
   */
  joint_positions_.setZero();
  joint_velocities_.setZero();
  joint_torques_.setZero();
  joint_target_torques_.setZero();
  joint_encoder_index_.setZero();

  /**
   * Additional data
   */
  slider_positions_.setZero();
  contact_sensors_states_.setZero();

  /**
   * Setup some known data
   */

  // for now this value is very small but it is currently for debug mode
  motor_max_current_.fill(4.0); // TODO: set as paramters?
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
}

void Solo12::initialize(const std::string &if_name, const int n_active_motors)
{
  // Initialize the communication with the main board.
  main_board_ptr = std::make_shared<MasterBoardInterface>(if_name);
  main_board_ptr->Init();

  n_active_motors_ = n_active_motors;

  /**
   * Mapping between the DOF and driver boards + motor ports
   *
   * FL_HAA - driver 0, motor port 0, motor index 0
   * FL_HFE - driver 1, motor port 1, motor index 3
   * FL_KFE - driver 1, motor port 0, motor index 2
   * FR_HAA - driver 0, motor port 1, motor index 1
   * FR_HFE - driver 2, motor port 1, motor index 5
   * FR_KFE - driver 2, motor port 0, motor index 4
   * HL_HAA - driver 3, motor port 0, motor index 6
   * HL_HFE - driver 4, motor port 1, motor index 9
   * HL_KFE - driver 4, motor port 0, motor index 8
   * HR_HAA - driver 3, motor port 1, motor index 7
   * HR_HFE - driver 5, motor port 1, motor index 11
   * HR_KFE - driver 5, motor port 0, motor index 10
   */
  joint_to_motor_index_ = {0, 3, 2, 1, 5, 4, 6, 9, 8, 7, 11, 10};

  // Create the motors object.
  for(unsigned i = 0; i < motors_.size(); ++i)
  {
    motors_[i] = std::make_shared<blmc_drivers::SPIMotor> (
      main_board_ptr, joint_to_motor_index_[i]);
  }

  // Create the joint module objects
  joints_.set_motor_array(motors_, motor_torque_constants_, joint_gear_ratios_,
                          joint_zero_positions_, motor_max_current_);

  // Set the maximum joint torque available
  max_joint_torques_ = max_joint_torque_security_margin_ *
                       joints_.get_max_torques().array();

  // fix the polarity to be the same as the urdf model.
  reverse_polarities_[0] = true;  // FL_HAA
  reverse_polarities_[1] = true;  // FL_HFE
  reverse_polarities_[2] = true;  // FL_KFE
  reverse_polarities_[3] = false; // FR_HAA
  reverse_polarities_[4] = false; // FR_HFE
  reverse_polarities_[5] = false; // FR_KFE
  reverse_polarities_[6] = true;  // HL_HAA
  reverse_polarities_[7] = true;  // HL_HFE
  reverse_polarities_[8] = true;  // HL_KFE
  reverse_polarities_[9] = false;  // HL_HAA
  reverse_polarities_[10] = false; // HR_HFE
  reverse_polarities_[11] = false; // HR_KFE
  joints_.set_joint_polarities(reverse_polarities_);

  // The the control gains in order to perform the calibration
  blmc_robots::Vector12d kp, kd;
  kp.fill(3.0);
  kd.fill(0.05);
  joints_.set_position_control_gains(kp, kd);


  // Enable all the motors and boards.
  for (unsigned i = 0; i < n_active_motors / 2; i++)
  {
    main_board_ptr->motor_drivers[i].motor1->SetCurrentReference(0);
    main_board_ptr->motor_drivers[i].motor2->SetCurrentReference(0);
    main_board_ptr->motor_drivers[i].motor1->Enable();
    main_board_ptr->motor_drivers[i].motor2->Enable();
    main_board_ptr->motor_drivers[i].EnablePositionRolloverError();
    main_board_ptr->motor_drivers[i].SetTimeout(5);
    main_board_ptr->motor_drivers[i].Enable();
    rt_printf("Enabled motor_driver %d\n", i);
  }

  // wait until all board are ready and connected.
  real_time_tools::Spinner spinner;
  spinner.set_frequency(100);

  int c = 0;
  bool is_ready = false;
  while (!is_ready)
  {
    is_ready = true;
    main_board_ptr->ParseSensorData();
    main_board_ptr->SendCommand(); // To keep boards alive.
    for(unsigned i = 0; i < n_active_motors_; ++i)
    {
      if (!main_board_ptr->motors[i].IsReady())
      {
        is_ready = false;
        if (c % 1000 == 0) {
            rt_printf("motor %d not ready.\n", i);
        }
        break;
      }
    }
    c += 1;
    spinner.spin();
  }
  rt_printf("All motors and boards are ready.");
}

double Solo12::get_adc_by_index_(unsigned int adc_index)
{
  return main_board_ptr->motor_drivers[adc_index / 2].adc[adc_index % 2];
}

void Solo12::acquire_sensors()
{
  main_board_ptr->ParseSensorData();

  /**
    * Joint data
    */
  // acquire the joint position
  joint_positions_ = joints_.get_measured_angles();
  // acquire the joint velocities
  joint_velocities_ = joints_.get_measured_velocities();
  // acquire the joint torques
  joint_torques_ = joints_.get_measured_torques();
  // acquire the target joint torques
  joint_target_torques_ = joints_.get_sent_torques();

  // The index angle is not transmitted.
  //joint_encoder_index_ = joints_.get_measured_index_angles();

  /**
    * Additional data
    */
  // acquire the slider positions
  for (unsigned i=0 ; i<slider_positions_.size() ; ++i)
  {
    // acquire the slider
    slider_positions_(i) = get_adc_by_index_(i);

    // acquire the current contact states
    // TODO: Implement me.
    // contact_sensors_states_(i) =
    //     contact_sensors_[i]->get_measurement()->newest_element();
  }

  /**
   * The different status.
   */
  for(unsigned i = 0; i < n_active_motors_; ++i)
  {
      motor_enabled_[i] = main_board_ptr->motors[i].IsEnabled();
      motor_ready_[i] = main_board_ptr->motors[i].IsReady();
  }

  for(unsigned i = 0; i < n_active_motors_ / 2; ++i)
  {
      motor_board_enabled_[i] = main_board_ptr->motor_drivers[i].is_enabled;
      motor_board_errors_[i] = main_board_ptr->motor_drivers[i].error_code;
  }
}

void Solo12::send_target_joint_torque(
    const Eigen::Ref<Vector12d> target_joint_torque)
{
  Vector12d ctrl_torque = target_joint_torque;
  ctrl_torque = ctrl_torque.array().min(max_joint_torques_);
  ctrl_torque = ctrl_torque.array().max(- max_joint_torques_);
  joints_.set_torques(ctrl_torque);
  main_board_ptr->SendCommand();
}

bool Solo12::calibrate(const Vector12d& home_offset_rad)
{
    throw std::invalid_argument("Calibration is not supported yet.");
}

} // namespace blmc_robots
