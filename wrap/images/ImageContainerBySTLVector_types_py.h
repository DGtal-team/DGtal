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

#ifndef DGTAL_IMAGECONTAINERBYSTLVECTOR_TYPES_PY_H
#define DGTAL_IMAGECONTAINERBYSTLVECTOR_TYPES_PY_H

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "kernel/HyperRectDomain_types_py.h"
#include "io/Color_types_py.h"

/*
 * List of TValues for ImageContainer:
 * Integer (i.e int), Real (i.e double), float
 * RealPoint2D, RealPoint3D, Point2D, Point3D
 * DGtal::Color
 */
namespace DGtal {
    namespace Python {
        // --- 2D ---
        using ImageContainerByVector2DInteger =
            DGtal::ImageContainerBySTLVector<DomainZ2i, Integer>;
        using ImageContainerByVector2DReal =
            DGtal::ImageContainerBySTLVector<DomainZ2i, Real>;
        using ImageContainerByVector2DFloat =
            DGtal::ImageContainerBySTLVector<DomainZ2i, float>;
        using ImageContainerByVector2DColor =
            DGtal::ImageContainerBySTLVector<DomainZ2i, Color>;

        // --- 3D ---
        using ImageContainerByVector3DInteger =
            DGtal::ImageContainerBySTLVector<DomainZ3i, Integer>;
        using ImageContainerByVector3DReal =
            DGtal::ImageContainerBySTLVector<DomainZ3i, Real>;
        using ImageContainerByVector3DFloat =
            DGtal::ImageContainerBySTLVector<DomainZ3i, float>;
        using ImageContainerByVector3DColor =
            DGtal::ImageContainerBySTLVector<DomainZ3i, Color>;
    } // namespace Python
} // namespace DGtal
#endif
