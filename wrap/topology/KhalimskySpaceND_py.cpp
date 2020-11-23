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

#include "KhalimskySpaceND_types_py.h"
#include "KhalimskySpaceND_declare_py.h"

namespace py = pybind11;
using namespace DGtal;


void init_KhalimskySpaceND(py::module & m) {
    using Cell2D = DGtal::Python::Cell2D;
    using SCell2D = DGtal::Python::SCell2D;
    using KSpace2D = DGtal::Python::KSpace2D;
    using Cell3D = DGtal::Python::Cell3D;
    using SCell3D = DGtal::Python::SCell3D;
    using KSpace3D = DGtal::Python::KSpace3D;

    auto py_class_Cell2D = declare_KhalimskyCell<Cell2D>(m, "Cell2D");
    auto py_class_Cell3D = declare_KhalimskyCell<Cell3D>(m, "Cell3D");
}
