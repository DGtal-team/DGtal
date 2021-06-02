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

#include "MetricAdjacency_types_py.h"
#include "MetricAdjacency_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_MetricAdjacency(py::module & m) {
    // --- 2D ---
    using Adj4 = Python::Adj4;
    using Adj8 = Python::Adj8;
    auto py_class_Adj4 = declare_MetricAdjacency<Adj4>(m, "Adj4");
    auto py_class_Adj8 = declare_MetricAdjacency<Adj8>(m, "Adj8");
    // --- 3D ---
    using Adj6 = Python::Adj6;
    using Adj18 = Python::Adj18;
    using Adj26 = Python::Adj26;
    auto py_class_Adj6 = declare_MetricAdjacency<Adj6>(m, "Adj6");
    auto py_class_Adj18 = declare_MetricAdjacency<Adj18>(m, "Adj18");
    auto py_class_Adj26 = declare_MetricAdjacency<Adj26>(m, "Adj26");
}
