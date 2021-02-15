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

#include "CubicalComplex_types_py.h"
#include "CubicalComplex_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_CubicalComplex(py::module & m) {
    // CubicalCellData
    const std::string docs_CubicalCellData =
R"(Any cell is stored within a cubical complex with an associated
data.
Its basic usage is to store flags associated to the cells, but it may store other
values.

Predefined flags are:
CubicalComplex.REMOVED,
CubicalComplex.COLLAPSIBLE,
CubicalComplex.FIXED,
CubicalComplex::USER1.
Other bits can be used to associate an integer to the cell.
The corresponding mask is CubicalComplex.VALUE.

Such data is notably used in collapse operation
)";
    auto py_class_CubicalCellData = py::class_<CubicalCellData>(m, "CubicalCellData",
            docs_CubicalCellData.c_str());
    py_class_CubicalCellData.def(py::init());
    py_class_CubicalCellData.def(py::init<DGtal::uint32_t>());
    py_class_CubicalCellData.def_readwrite("data", &CubicalCellData::data);
    py_class_CubicalCellData.def("__repr__", [](const CubicalCellData & self) {
        std::stringstream os;
        os << "data: " << self.data;
        return os.str();
    });


    // CellMap
    declare_CellMap<Python::CellMap2D>(m, "CellMap2D");
    declare_CellMap<Python::CellMap3D>(m, "CellMap3D");

    // CubicalComplex
    auto py_class_CubicalComplex2D = declare_CubicalComplex<Python::CubicalComplex2D>(m, "CubicalComplex2D");
    declare_CubicalComplex2DMethods<Python::CubicalComplex2D>(py_class_CubicalComplex2D);
    auto py_class_CubicalComplex3D = declare_CubicalComplex<Python::CubicalComplex3D>(m, "CubicalComplex3D");
    declare_CubicalComplex3DMethods<Python::CubicalComplex3D>(py_class_CubicalComplex3D);
}
