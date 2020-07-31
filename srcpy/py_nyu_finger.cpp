/*
 * Copyright [2017] Max Planck Society. All rights reserved.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include <blmc_robots/nyu_finger.hpp>

namespace py = pybind11;
using namespace blmc_robots;

PYBIND11_MODULE(py_nyu_finger, m)
{
    py::class_<NYUFinger>(m, "NYUFinger")
        .def(py::init<>())
        .def("initialize", &NYUFinger::initialize,  py::arg("interface_name"))
        .def("acquire_sensors", &NYUFinger::acquire_sensors)
        .def("send_target_joint_torque",
             &NYUFinger::send_target_joint_torque,
             py::arg("target_joint_torque"))
        .def("set_max_joint_torques",
             &NYUFinger::set_max_joint_torques,
             py::arg("max_joint_torques"))
        .def("get_motor_board_errors", &NYUFinger::get_motor_board_errors)
        .def("get_motor_board_enabled", &NYUFinger::get_motor_board_enabled)
        .def("get_motor_enabled", &NYUFinger::get_motor_enabled)
        .def("get_motor_ready", &NYUFinger::get_motor_ready)
        // .def("get_slider_positions", &NYUFinger::get_slider_positions)
        .def("get_joint_positions", &NYUFinger::get_joint_positions)
        .def("get_joint_velocities", &NYUFinger::get_joint_velocities);
}
