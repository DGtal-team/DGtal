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
 *
 **/

#include "dgtal_pybind11_common.h"

#include "DigitalTopology_types_py.h"
#include "DigitalTopology_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_DigitalTopology(py::module & m) {

    py::enum_<DGtal::DigitalTopologyProperties>(m, "DigitalTopologyProperties",
            R"(Possible properties of digital topologies.)")
        .value("UNKNOWN_DT", DGtal::UNKNOWN_DT)
        .value("NOT_JORDAN_DT", DGtal::NOT_JORDAN_DT)
        .value("JORDAN_DT", DGtal::JORDAN_DT)
        .export_values();

    // --- 2D ---
    auto py_class_DT4_8 = declare_DigitalTopology<Python::DT4_8>(m, "DT4_8");
    auto py_class_DT8_4 = declare_DigitalTopology<Python::DT8_4>(m, "DT8_4");
    // --- 3D ---
    auto py_class_DT6_18 = declare_DigitalTopology<Python::DT6_18>(m, "DT6_18");
    auto py_class_DT18_6 = declare_DigitalTopology<Python::DT18_6>(m, "DT18_6");
    auto py_class_DT6_26 = declare_DigitalTopology<Python::DT6_26>(m, "DT6_26");
    auto py_class_DT26_6 = declare_DigitalTopology<Python::DT26_6>(m, "DT26_6");
}
