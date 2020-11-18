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
#include "dgtal_python_types.h"

#include "HyperRectDomain_declare_py.h"
#include "DGtal/kernel/SpaceND.h"

namespace py = pybind11;
using namespace DGtal;

void init_HyperRectDomain(py::module & m) {
    using Z2i  = SpaceND<2, DGtal::PythonInteger>;
    using DomainZ2i = HyperRectDomain <Z2i>;
    auto py_class_DomainZ2i = declare_HyperRectDomain<DomainZ2i>(m, "DomainZ2i");

    using Z3i  = SpaceND<3, DGtal::PythonInteger>;
    using DomainZ3i = HyperRectDomain <Z3i>;
    auto py_class_DomainZ3i = declare_HyperRectDomain<DomainZ3i>(m, "DomainZ3i");
}
