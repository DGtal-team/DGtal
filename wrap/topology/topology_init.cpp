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

namespace py = pybind11;

void init_KhalimskyPreSpaceND(py::module &);
void init_KhalimskySpaceND(py::module &);
void init_MetricAdjacency(py::module &);
void init_DigitalTopology(py::module &);

void init_dgtal_topology(py::module & mparent) {
    auto m = mparent.def_submodule("topology");
    init_KhalimskyPreSpaceND(m);
    init_KhalimskySpaceND(m);
    init_MetricAdjacency(m);
    init_DigitalTopology(m);
}
