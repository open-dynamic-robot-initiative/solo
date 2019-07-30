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

#include <blmc_robots/real_finger.hpp>

namespace py = pybind11;
using namespace blmc_robots;

PYBIND11_MODULE(py_real_finger, m)
{

  // m.def("test", &test);

  py::class_<RealFinger::Observation>(m, "Observation")
      .def_readwrite("angle", &RealFinger::Observation::angle)
      .def_readwrite("velocity", &RealFinger::Observation::velocity)
      .def_readwrite("torque", &RealFinger::Observation::torque);

  py::class_<RealFinger, std::shared_ptr<RealFinger>>(m, "RealFinger")
      .def_static("create", RealFinger::create)
      .def(py::init<const std::string &, const std::string &>())
      .def("get_observation", &RealFinger::get_observation)
      .def("get_desired_action", &RealFinger::get_desired_action)
      .def("get_safe_action", &RealFinger::get_safe_action)
      .def("get_time_stamp_ms", &RealFinger::get_time_stamp_ms)
      .def("append_desired_action", &RealFinger::append_desired_action)
      .def("wait_until_time_index", &RealFinger::wait_until_time_index)
      .def("current_time_index", &RealFinger::current_time_index)
      .def("pause", &RealFinger::pause);
}
