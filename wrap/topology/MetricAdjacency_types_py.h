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

#ifndef DGTAL_METRICADJACENCY_TYPES_PY_H
#define DGTAL_METRICADJACENCY_TYPES_PY_H

#include "kernel/HyperRectDomain_types_py.h" // For Z2i, Z3i (SpaceND)
#include "DGtal/topology/MetricAdjacency.h"

namespace DGtal {
    namespace Python {
        using Adj4 = DGtal::MetricAdjacency< DGtal::Python::Z2i, 1>;
        using Adj8 = DGtal::MetricAdjacency< DGtal::Python::Z2i, 2>;

        using Adj6 = DGtal::MetricAdjacency< DGtal::Python::Z3i, 1>;
        using Adj18 = DGtal::MetricAdjacency< DGtal::Python::Z3i, 2>;
        using Adj26 = DGtal::MetricAdjacency< DGtal::Python::Z3i, 3>;
    } // namespace Python
} // namespace DGtal
#endif
