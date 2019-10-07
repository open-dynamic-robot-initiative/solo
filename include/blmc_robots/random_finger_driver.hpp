#pragma once

#include <math.h>
#include <Eigen/Eigen>
#include <blmc_robots/common_header.hpp>
#include <tuple>

#include <blmc_robots/blmc_joint_module.hpp>
#include "real_time_tools/spinner.hpp"
#include "real_time_tools/timer.hpp"

#include <robot_interfaces/finger.hpp>

namespace blmc_robots
{
class RandomFingerDriver : public robot_interfaces::RobotDriver<
                               robot_interfaces::finger::Action,
                               robot_interfaces::finger::Observation>
{
public:
    typedef robot_interfaces::finger::Action Action;
    typedef robot_interfaces::finger::Observation Observation;
    typedef robot_interfaces::finger::Vector Vector;
    typedef robot_interfaces::finger::Status Status;

    int data_generating_index_ = 0;

    // adjusted values
    RandomFingerDriver() : RobotDriver(0.03, 0.05)
    {
    }

    Observation get_latest_observation() override
    {
        // generating observations by a rule to make it easier to check they are
        // being logged correctly as the timeindex increases.

        Observation observation;
        observation.angle[0] = data_generating_index_;
        observation.angle[1] = 2 * data_generating_index_;
        observation.angle[2] = 3 * data_generating_index_;

        observation.velocity[0] = data_generating_index_ + 1;
        observation.velocity[1] = 2 * data_generating_index_ + 1;
        observation.velocity[2] = 3 * data_generating_index_ + 1;

        observation.torque[0] = data_generating_index_ + 2;
        observation.torque[1] = 2 * data_generating_index_ + 2;
        observation.torque[2] = 3 * data_generating_index_ + 2;

        data_generating_index_++;

        return observation;
    }

    Action apply_action(const Action &desired_action) override
    {
        double start_time_sec = real_time_tools::Timer::get_current_time_sec();

        real_time_tools::Timer::sleep_until_sec(start_time_sec + 0.001);

        return desired_action;
    }

    void shutdown() override
    {
        return;
    }
};

robot_interfaces::finger::BackendPtr create_random_finger_backend(
    robot_interfaces::finger::DataPtr robot_data)
{
    std::shared_ptr<
        robot_interfaces::RobotDriver<robot_interfaces::finger::Action,
                                      robot_interfaces::finger::Observation>>
        robot = std::make_shared<RandomFingerDriver>();

    auto backend =
        std::make_shared<robot_interfaces::finger::Backend>(robot, robot_data);
    backend->set_max_action_repetitions(-1);

    return backend;
}

}  // namespace blmc_robots
