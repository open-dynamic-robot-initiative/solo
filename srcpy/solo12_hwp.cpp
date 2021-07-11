/**
 * @file
 * @license BSD 3-clause
 * @copyright Copyright (c) 2021, New York University and Max Planck
 * Gesellschaft
 *
 * @brief Python bindings for the nyu finger hardware process.
 */

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>

#include "solo/dynamic_graph_manager/dgm_solo12.hpp"

namespace solo
{

namespace py = pybind11;

PYBIND11_MODULE(solo12_hwp_cpp, m)
{
    m.doc() = R"pbdoc(
        Solo12 HardWareProcess bindigns
        ---------------------------------
        .. currentmodule:: mim_control
        .. autosummary::
           :toctree: _generate
           add
           subtract
    )pbdoc";

    py::class_<DGMSolo12>(m, "Solo12HWP")
        .def(py::init<>())
        .def("initialize", &DGMSolo12::initialize)
        .def("calibrate", &DGMSolo12::calibrate_joint_position)
        .def("calibrate_from_yaml", &DGMSolo12::calibrate_joint_position_from_yaml)
        .def("run", &DGMSolo12::run)
    ;
}

}