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

#ifndef DGTAL_DIGITALTOPOLOGY_TYPES_PY_H
#define DGTAL_DIGITALTOPOLOGY_TYPES_PY_H

#include "MetricAdjacency_types_py.h" // For Adj4, etc.
#include "DGtal/topology/DigitalTopology.h"

namespace DGtal {
    namespace Python {
        // --- 2D ---
        using DT4_8 = DGtal::DigitalTopology<Python::Adj4, Python::Adj8>;
        using DT8_4 = DGtal::DigitalTopology<Python::Adj8, Python::Adj4>;
        // --- 3D ---
        using DT6_18 = DGtal::DigitalTopology<Python::Adj6, Python::Adj18>;
        using DT18_6 = DGtal::DigitalTopology<Python::Adj18, Python::Adj6>;
        using DT6_26 = DGtal::DigitalTopology<Python::Adj6, Python::Adj26>;
        using DT26_6 = DGtal::DigitalTopology<Python::Adj26, Python::Adj6>;
    } // namespace Python
} // namespace DGtal
#endif
