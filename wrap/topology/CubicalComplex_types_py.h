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

#ifndef DGTAL_CUBICALCOMPLEX_TYPES_PY_H
#define DGTAL_CUBICALCOMPLEX_TYPES_PY_H

#include "topology/KhalimskySpaceND_types_py.h" // For KSpace2D
#include "DGtal/topology/CubicalComplex.h"

#include <unordered_map>
#include "DGtal/topology/KhalimskyCellHashFunctions.h"

namespace DGtal {
    namespace Python {
        using CellMap2D = std::unordered_map<typename KSpace2D::Cell, DGtal::CubicalCellData>;
        using CellMap3D = std::unordered_map<typename KSpace3D::Cell, DGtal::CubicalCellData>;
        using CubicalComplex2D = DGtal::CubicalComplex<KSpace2D, CellMap2D>;
        using CubicalComplex3D = DGtal::CubicalComplex<KSpace3D, CellMap3D>;
    } // namespace Python
} // namespace DGtal
#endif
