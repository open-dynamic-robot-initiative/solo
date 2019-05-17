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
#include <pybind11/stl_bind.h>
#include <pybind11/pybind11.h>

#include <blmc_robots/single_leg.hpp>

namespace py = pybind11;
using namespace blmc_robots;

PYBIND11_MODULE(py_blmc_single_leg, m) {
  // binding of stl containers
  // py::bind_vector<std::vector<Eigen::Vector3d>>(m, "ArrayVector3d");
  // py::bind_vector<std::vector<KinematicsState>>(m, "KinStateVector");
  // py::bind_vector<std::vector<Eigen::MatrixXd>>(m, "JacobianVector");

  py::class_<SingleLeg>(m, "SingleLeg")
    .def(py::init<>())
    .def("initialize", &SingleLeg::initialize)
    .def("acquire_sensors", &SingleLeg::acquire_sensors)
    .def("send_target_joint_torque", &SingleLeg::send_target_joint_torque, py::arg("target_joint_torque"))
    .def("get_joint_positions", &SingleLeg::get_joint_positions)
    .def("get_joint_velocities", &SingleLeg::get_joint_velocities)
    .def("get_motor_positions", &SingleLeg::get_motor_positions)
    .def("get_motor_velocities", &SingleLeg::get_motor_velocities)
    .def("zero_joint_positions", &SingleLeg::zero_joint_positions)
    .def("get_slider_positions", &SingleLeg::get_slider_positions)
    .def("set_max_current", &SingleLeg::set_max_current)
    .def("disable_can_recv_timeout", &SingleLeg::disable_can_recv_timeout)
  ;
}
