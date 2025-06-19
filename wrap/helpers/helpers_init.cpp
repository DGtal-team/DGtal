/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 **/

/**
 * @file helpers_init.cpp
 * @author Bastien Doignies <bastien.doignies@liris.cnrs.fr>
 *
 * @date 2025/05/11
 *
 * Initializer code for helpers python binding submodule 
 *
 * This file is part of the DGtal library.
 */

#include "dgtal_pybind11_common.h"
namespace py = pybind11;

#include "DGtal/helpers/Parameters.h"

void bind_shortcuts(py::module& m);

void init_dgtal_helpers(py::module& m) {
    using namespace DGtal;

    auto m_helpers = m.def_submodule("helpers", "Submodule for DGtal helpers");
    m_helpers.attr("SAMPLES_PATH") = py::str(std::string(DGTAL_PATH) + "/examples/samples/");

    // Bind parameter class
    py::class_<Parameters>(m_helpers, "Parameters")
        .def(py::init<>())
        .def("set", [](Parameters& params, const std::string& name, int value) {
            return params(name, ParameterValue(value));
        }, py::return_value_policy::reference)
        .def("set", [](Parameters& params, const std::string& name, float value) {
            return params(name, ParameterValue(value));
        }, py::return_value_policy::reference)
       .def("set", [](Parameters& params, const std::string& name, float value) {
            return params(name, ParameterValue(value));
        }, py::return_value_policy::reference)
       .def("set", [](Parameters& params, const std::string& name, std::string value) {
            return params(name, ParameterValue(value));
        }, py::return_value_policy::reference)
       .def("__str__", [](const Parameters& params) {
            std::stringstream ss;
            params.selfDisplay(ss);
            return ss.str();
        });

    bind_shortcuts(m_helpers);
}
