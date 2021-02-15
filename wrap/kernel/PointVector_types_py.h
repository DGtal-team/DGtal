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

#ifndef DGTAL_POINTVECTOR_TYPES_PY_H
#define DGTAL_POINTVECTOR_TYPES_PY_H

#include "DGtal/kernel/PointVector.h"
#include "base/Common_types_py.h"

namespace DGtal {
    namespace Python {
        using Point2D = PointVector<2, DGtal::Python::Integer>;
        using Point3D = PointVector<3, DGtal::Python::Integer>;
        using RealPoint2D = PointVector<2, DGtal::Python::Real>;
        using RealPoint3D = PointVector<3, DGtal::Python::Real>;
    } // namespace Python
} // namespace DGtal
#endif
