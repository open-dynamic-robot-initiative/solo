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

#include <solo/solo8.hpp>

namespace py = pybind11;
using namespace solo;

PYBIND11_MODULE(py_solo8, m)
{
    // binding of stl containers
    // py::bind_vector<std::vector<Eigen::Vector3d>>(m, "ArrayVector3d");
    // py::bind_vector<std::vector<KinematicsState>>(m, "KinStateVector");
    // py::bind_vector<std::vector<Eigen::MatrixXd>>(m, "JacobianVector");

    py::class_<Solo8>(m, "Solo8")
        .def(py::init<>())
        .def("initialize", &Solo8::initialize)
        .def("acquire_sensors", &Solo8::acquire_sensors)
        .def("send_target_joint_torque",
             &Solo8::send_target_joint_torque,
             py::arg("target_joint_torque"))
        .def("set_max_joint_torques",
             &Solo8::set_max_joint_torques,
             py::arg("max_joint_torques"))
        .def("get_motor_board_errors", &Solo8::get_motor_board_errors)
        .def("get_motor_board_enabled", &Solo8::get_motor_board_enabled)
        .def("get_motor_enabled", &Solo8::get_motor_enabled)
        .def("get_motor_ready", &Solo8::get_motor_ready)
        .def("get_slider_positions", &Solo8::get_slider_positions)
        .def("get_joint_positions", &Solo8::get_joint_positions)
        .def("get_joint_velocities", &Solo8::get_joint_velocities);
}
