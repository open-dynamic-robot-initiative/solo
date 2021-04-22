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

#include <blmc_robots/solo12.hpp>

namespace py = pybind11;
using namespace blmc_robots;

PYBIND11_MODULE(py_solo12, m)
{
    // binding of stl containers
    // py::bind_vector<std::vector<Eigen::Vector3d>>(m, "ArrayVector3d");
    // py::bind_vector<std::vector<KinematicsState>>(m, "KinStateVector");
    // py::bind_vector<std::vector<Eigen::MatrixXd>>(m, "JacobianVector");

    py::class_<Solo12>(m, "Solo12")
        .def(py::init<>())
        .def("initialize",
             &Solo12::initialize,
             py::arg("interface_name"),
             py::arg("serial_port"))
        .def("acquire_sensors", &Solo12::acquire_sensors)
        .def("send_target_joint_torque",
             &Solo12::send_target_joint_torque,
             py::arg("target_joint_torque"))
        .def(
            "set_max_current", &Solo12::set_max_current, py::arg("max_current"))
        .def("get_motor_board_errors", &Solo12::get_motor_board_errors)
        .def("get_motor_board_enabled", &Solo12::get_motor_board_enabled)
        .def("get_motor_enabled", &Solo12::get_motor_enabled)
        .def("get_motor_ready", &Solo12::get_motor_ready)
        .def("get_slider_positions", &Solo12::get_slider_positions)
        .def("get_joint_positions", &Solo12::get_joint_positions)
        .def("get_joint_velocities", &Solo12::get_joint_velocities);
}
