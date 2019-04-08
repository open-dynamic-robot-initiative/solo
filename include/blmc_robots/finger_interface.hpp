#pragma once

#include <Eigen/Eigen>


class FingerInterface
{
public:
    typedef Eigen::Vector3d Vector;

    virtual void set_torques(const Vector& desired_torques) = 0;
    virtual void send_torques() = 0;

    virtual Vector get_sent_torques() const = 0;
    virtual Vector get_measured_torques() const = 0;
    virtual Vector get_angles() const = 0;
    virtual Vector get_angular_velocities() const = 0;

    virtual void pause_motors() = 0;
    virtual void wait_since_last_send(const double& time_s) = 0;
};
