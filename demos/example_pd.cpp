#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <chrono>

#include <master_board_sdk/defines.h>
#include <master_board_sdk/master_board_interface.h>
#include <iostream>
#include <real_time_tools/spinner.hpp>
#include <real_time_tools/thread.hpp>
#include <real_time_tools/timer.hpp>

#define N_SLAVES_CONTROLED 6

class SineWaver
{
public:
    SineWaver(const std::string &interface)
        : robot_if_(interface), enabled_(false)
    {
    }

    void run()
    {
        enabled_ = true;

        robot_if_.Init();

        real_time_tools::RealTimeThread rt_thread;
        rt_thread.create_realtime_thread(
            [](void *self)
            {
                static_cast<SineWaver *>(self)->sine_motion();
                //static_cast<SineWaver *>(self)->zero_torque();
                return (void *)nullptr;
            },
            this);

        double start_time = real_time_tools::Timer::get_current_time_sec();

        while (enabled_ && !robot_if_.IsTimeout())
        {
            real_time_tools::Timer::sleep_sec(1);
            double now = real_time_tools::Timer::get_current_time_sec();
            double passed_time_min = (now - start_time) / 60.0;

            printf("\33[H\33[2J");  // clear screen
            robot_if_.PrintPowerBoard();
            robot_if_.PrintIMU();
            robot_if_.PrintADC();
            robot_if_.PrintMotors();
            robot_if_.PrintMotorDrivers();
            robot_if_.PrintStats();
            printf("Duration: %.1f min\n", passed_time_min);
            fflush(stdout);
        }
        if (!error_msg_.empty())
        {
            std::cerr << "ERROR: " << error_msg_ << std::endl;
        }
        if (!robot_if_.IsTimeout())
        {
            std::cerr
                << "Masterboard timeout detected. Either the masterboard has "
                   "been shut down or there has been a connection issue with "
                   "the cable/wifi."
                << std::endl;
        }

        rt_thread.join();
    }

private:
    MasterBoardInterface robot_if_;
    std::atomic<bool> enabled_;
    std::string error_msg_;

    void zero_torque()
    {
        constexpr double dt = 0.001;
        constexpr double iq_sat = 4.0;

        real_time_tools::Spinner spinner;
        spinner.set_period(dt);

        double t = 0;

        // Initialisation, send the init commands
        for (int i = 0; i < N_SLAVES_CONTROLED; i++)
        {
            robot_if_.motor_drivers[i].motor1->SetCurrentReference(0);
            robot_if_.motor_drivers[i].motor2->SetCurrentReference(0);
            robot_if_.motor_drivers[i].motor1->Enable();
            robot_if_.motor_drivers[i].motor2->Enable();

            // Set the maximum current controlled by the card.
            robot_if_.motor_drivers[i].motor1->set_current_sat(iq_sat);
            robot_if_.motor_drivers[i].motor2->set_current_sat(iq_sat);

            robot_if_.motor_drivers[i].EnablePositionRolloverError();
            robot_if_.motor_drivers[i].SetTimeout(5);
            robot_if_.motor_drivers[i].Enable();
        }

        std::chrono::time_point<std::chrono::system_clock> last =
            std::chrono::system_clock::now();
        while (enabled_ && !robot_if_.IsTimeout() &&
               !robot_if_.IsAckMsgReceived())
        {
            robot_if_.SendInit();
            spinner.spin();
        }

        if (robot_if_.IsTimeout())
        {
            printf("Timeout while waiting for ack.\n");
        }

        while (enabled_ && !robot_if_.IsTimeout())
        {
            // last+dt would be better
            last = std::chrono::system_clock::now();
            t += dt;
            // This will read the last incoming packet and update all sensor
            // fields.
            robot_if_.ParseSensorData();

            // This will send the command packet
            robot_if_.SendCommand();

            spinner.spin();
        }
    }

    void sine_motion()
    {
        constexpr double dt = 0.001;
        constexpr double kp = 5.;
        constexpr double kd = 0.1;
        constexpr double iq_sat = 4.0;
        constexpr double freq = 0.5;
        constexpr double amplitude = M_PI;

        real_time_tools::Spinner spinner;
        spinner.set_period(dt);

        double t = 0;
        double init_pos[N_SLAVES * 2] = {0};
        int state = 0;

        // Initialisation, send the init commands
        for (int i = 0; i < N_SLAVES_CONTROLED; i++)
        {
            robot_if_.motor_drivers[i].motor1->SetCurrentReference(0);
            robot_if_.motor_drivers[i].motor2->SetCurrentReference(0);
            robot_if_.motor_drivers[i].motor1->Enable();
            robot_if_.motor_drivers[i].motor2->Enable();

            // Set the gains for the PD controller running on the cards.
            robot_if_.motor_drivers[i].motor1->set_kp(kp);
            robot_if_.motor_drivers[i].motor2->set_kp(kp);
            robot_if_.motor_drivers[i].motor1->set_kd(kd);
            robot_if_.motor_drivers[i].motor2->set_kd(kd);

            // Set the maximum current controlled by the card.
            robot_if_.motor_drivers[i].motor1->set_current_sat(iq_sat);
            robot_if_.motor_drivers[i].motor2->set_current_sat(iq_sat);

            robot_if_.motor_drivers[i].EnablePositionRolloverError();
            robot_if_.motor_drivers[i].SetTimeout(5);
            robot_if_.motor_drivers[i].Enable();
        }

        std::chrono::time_point<std::chrono::system_clock> last =
            std::chrono::system_clock::now();
        while (enabled_ && !robot_if_.IsTimeout() &&
               !robot_if_.IsAckMsgReceived())
        {
            robot_if_.SendInit();
            spinner.spin();
        }

        if (robot_if_.IsTimeout())
        {
            printf("Timeout while waiting for ack.\n");
        }

        while (enabled_ && !robot_if_.IsTimeout())
        {
            // last+dt would be better
            last = std::chrono::system_clock::now();
            t += dt;
            // This will read the last incoming packet and update all sensor
            // fields.
            robot_if_.ParseSensorData();
            switch (state)
            {
                // check the end of calibration (are the all controlled motor
                // enabled and ready?)
                case 0:
                    state = 1;
                    for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                    {
                        if (!robot_if_.motor_drivers[i / 2].is_connected)
                        {
                            // ignoring the motors of a disconnected slave
                            continue;
                        }

                        if (!(robot_if_.motors[i].IsEnabled() &&
                              robot_if_.motors[i].IsReady()))
                        {
                            state = 0;
                        }
                        // initial position
                        init_pos[i] = robot_if_.motors[i].GetPosition();

                        // Use the current state as target for the PD
                        // controller.
                        robot_if_.motors[i].SetCurrentReference(0.);
                        robot_if_.motors[i].SetPositionReference(init_pos[i]);
                        robot_if_.motors[i].SetVelocityReference(0.);

                        t = 0;  // to start sin at 0
                    }
                    break;
                case 1:
                    // closed loop, position
                    for (int i = 0; i < N_SLAVES_CONTROLED * 2; i++)
                    {
                        if (i % 2 == 0)
                        {
                            if (!robot_if_.motor_drivers[i / 2].is_connected)
                            {
                                // ignoring the motors of a disconnected slave
                                continue;
                            }

                            // making sure that the transaction with the
                            // corresponding µdriver board succeeded
                            if (robot_if_.motor_drivers[i / 2].error_code ==
                                0xf)
                            {
                                // printf("Transaction with SPI%d failed\n", i /
                                // 2); user should decide what to do in that
                                // case, here we ignore that motor
                                continue;
                            }

                            if (robot_if_.motor_drivers[i / 2].error_code)
                            {
                                error_msg_ = "Motor driver has error";
                                enabled_ = false;
                                return;
                            }
                        }

                        if (robot_if_.motors[i].IsEnabled())
                        {
                            double ref = init_pos[i] +
                                         amplitude * sin(2 * M_PI * freq * t);
                            double v_ref = 2. * M_PI * freq * amplitude *
                                           cos(2 * M_PI * freq * t);

                            robot_if_.motors[i].SetCurrentReference(0.);
                            robot_if_.motors[i].SetPositionReference(ref);
                            robot_if_.motors[i].SetVelocityReference(v_ref);
                        }
                        else
                        {
                            error_msg_ = "Motor got disabled";
                            enabled_ = false;
                            return;
                        }
                    }
                    break;
            }
            // This will send the command packet
            robot_if_.SendCommand();

            spinner.spin();
        }
    }
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Invalid number of arguments" << std::endl;
        std::cerr << "Usage:  " << argv[0] << " <network_interface>"
                  << std::endl;
        return 1;
    }
    const std::string interface = argv[1];

    SineWaver sine_waver(interface);
    sine_waver.run();

    // run() runs forever, so if it terminates, this is always an error
    return 1;
}
