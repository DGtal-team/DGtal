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

#ifndef DGTAL_KHALIMSKYSPACEND_TYPES_PY_CPP
#define DGTAL_KHALIMSKYSPACEND_TYPES_PY_CPP

#include "base/Common_types_py.h" // For DGtal::Python::Integer
#include "DGtal/topology/KhalimskySpaceND.h"

namespace DGtal {
    namespace Python {
        using KSpace2D = DGtal::KhalimskySpaceND<2, Python::Integer>;
        using Cell2D = KSpace2D::Cell;
        using SCell2D = KSpace2D::SCell;

        using KSpace3D = DGtal::KhalimskySpaceND<3, Python::Integer>;
        using Cell3D = KSpace3D::Cell;
        using SCell3D = KSpace3D::SCell;
    } // namespace Python
} // namespace DGtal
#endif
