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

#include "DGtal/base/Common.h"

namespace py = pybind11;
using namespace DGtal;

void init_common(py::module &m) {
    py::enum_<Orientation>(m, "Orientation")
        .value("INSIDE", INSIDE)
        .value("ON", ON)
        .value("OUTSIDE", OUTSIDE)
        .export_values();
};
