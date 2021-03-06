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

#include "DigitalSetBySTLVector_types_py.h"
#include "DigitalSetBySTLVector_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_DigitalSetBySTLVector(py::module & m) {
    auto py_class_DigitalSetZ2i = declare_DigitalSetBySTLVector<Python::DigitalSetZ2i>(m, "DigitalSetZ2i");
    auto py_class_DigitalSetZ3i = declare_DigitalSetBySTLVector<Python::DigitalSetZ3i>(m, "DigitalSetZ3i");
}
