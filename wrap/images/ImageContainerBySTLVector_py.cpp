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

#include "ImageContainerBySTLVector_types_py.h"
#include "ImageContainerBySTLVector_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_ImageContainerBySTLVector(py::module & m) {
    // --- 2D ---
    auto py_class_ImageContainerByVector2DInteger =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DInteger>(m, "ImageContainerByVector2DInteger");
    auto py_class_ImageContainerByVector2DReal =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DReal>(m, "ImageContainerByVector2DReal");
    auto py_class_ImageContainerByVector2DFloat =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DFloat>(m, "ImageContainerByVector2DFloat");

    auto py_class_ImageContainerByVector2DColor =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DColor>(m, "ImageContainerByVector2DColor");

    auto py_class_ImageContainerByVector2DPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DPoint2D>(m, "ImageContainerByVector2DPoint2D");
    auto py_class_ImageContainerByVector2DRealPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DRealPoint2D>(m, "ImageContainerByVector2DRealPoint2D");
    auto py_class_ImageContainerByVector2DPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DPoint3D>(m, "ImageContainerByVector2DPoint3D");
    auto py_class_ImageContainerByVector2DRealPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DRealPoint3D>(m, "ImageContainerByVector2DRealPoint3D");

    // --- 3D ---
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DInteger>(m, "ImageContainerByVector3DInteger");
    auto py_class_ImageContainerByVector3DReal =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DReal>(m, "ImageContainerByVector3DReal");
    auto py_class_ImageContainerByVector3DFloat =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DFloat>(m, "ImageContainerByVector3DFloat");

    auto py_class_ImageContainerByVector3DColor =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DColor>(m, "ImageContainerByVector3DColor");

    auto py_class_ImageContainerByVector3DPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DPoint2D>(m, "ImageContainerByVector3DPoint2D");
    auto py_class_ImageContainerByVector3DRealPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DRealPoint2D>(m, "ImageContainerByVector3DRealPoint2D");
    auto py_class_ImageContainerByVector3DPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DPoint3D>(m, "ImageContainerByVector3DPoint3D");
    auto py_class_ImageContainerByVector3DRealPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DRealPoint3D>(m, "ImageContainerByVector3DRealPoint3D");
}
