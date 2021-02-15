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

#include "KhalimskyPreSpaceND_types_py.h"
#include "KhalimskyPreSpaceND_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_KhalimskyPreSpaceND(py::module & m) {
    using PreCell2D = DGtal::Python::PreCell2D;
    using SPreCell2D = DGtal::Python::SPreCell2D;
    using KPreSpace2D = DGtal::Python::KPreSpace2D;
    using PreCell3D = DGtal::Python::PreCell3D;
    using SPreCell3D = DGtal::Python::SPreCell3D;
    using KPreSpace3D = DGtal::Python::KPreSpace3D;

    auto py_class_PreCell2D = declare_KhalimskyPreCell<PreCell2D>(m, "PreCell2D");
    auto py_class_SPreCell2D = declare_SignedKhalimskyPreCell<SPreCell2D>(m, "SPreCell2D");
    auto py_class_KPreSpace2D = declare_KhalimskyPreSpaceND<KPreSpace2D>(m, "KPreSpace2D");

    auto py_class_PreCell3D = declare_KhalimskyPreCell<PreCell3D>(m, "PreCell3D");
    auto py_class_SPreCell3D = declare_SignedKhalimskyPreCell<SPreCell3D>(m, "SPreCell3D");
    auto py_class_KPreSpace3D = declare_KhalimskyPreSpaceND<KPreSpace3D>(m, "KPreSpace3D");
}
