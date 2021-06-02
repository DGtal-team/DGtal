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

#include "Object_types_py.h"
#include "Object_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_Object(py::module & m) {
    // --- 2D ---
    auto py_class_Object4_8 = declare_Object<Python::Object4_8>(m, "Object4_8");
    auto py_class_Object8_4 = declare_Object<Python::Object8_4>(m, "Object8_4");
    // --- 2D ---
    auto py_class_Object6_18 = declare_Object<Python::Object6_18>(m, "Object6_18");
    auto py_class_Object18_6 = declare_Object<Python::Object18_6>(m, "Object18_6");
    auto py_class_Object6_26 = declare_Object<Python::Object6_26>(m, "Object6_26");
    auto py_class_Object26_6 = declare_Object<Python::Object26_6>(m, "Object26_6");
}
