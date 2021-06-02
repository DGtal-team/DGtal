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

#include "images/SetFromImage_declare_py.h"
#include "kernel/DigitalSetBySTLVector_types_py.h" // For DigitalSetZ

namespace py = pybind11;
using namespace DGtal;

void init_ImageContainerBySTLVector(py::module & m) {
    // ------------------------------ 2D --------------------------------------
    // --------------- Single types --------------
    // Integer
    const std::string typestr_2DInteger = "ImageContainerByVector2DInteger";
    auto py_class_ImageContainerByVector2DInteger =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DInteger>(m, typestr_2DInteger );
    def_buffer_bridge(py_class_ImageContainerByVector2DInteger);
    add_set_from_image<
        Python::ImageContainerByVector2DInteger,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DInteger);

    // Real
    const std::string typestr_2DReal = "ImageContainerByVector2DReal";
    auto py_class_ImageContainerByVector2DReal =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DReal>(m, typestr_2DReal );
    def_buffer_bridge(py_class_ImageContainerByVector2DReal);
    add_set_from_image<
        Python::ImageContainerByVector2DReal,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DReal);

    // Float
    const std::string typestr_2DFloat = "ImageContainerByVector2DFloat";
    auto py_class_ImageContainerByVector2DFloat =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DFloat>(m, typestr_2DFloat );
    def_buffer_bridge(py_class_ImageContainerByVector2DFloat);
    add_set_from_image<
        Python::ImageContainerByVector2DFloat,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DFloat);

    // Short
    const std::string typestr_2DShort = "ImageContainerByVector2DShort";
    auto py_class_ImageContainerByVector2DShort =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DShort>(m, typestr_2DShort );
    def_buffer_bridge(py_class_ImageContainerByVector2DShort);
    add_set_from_image<
        Python::ImageContainerByVector2DShort,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DShort);

    // Unsigned char (Binary images)
    const std::string typestr_2DUnsignedChar = "ImageContainerByVector2DUnsignedChar";
    auto py_class_ImageContainerByVector2DUnsignedChar =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DUnsignedChar>(m, typestr_2DUnsignedChar );
    def_buffer_bridge(py_class_ImageContainerByVector2DUnsignedChar);
    add_set_from_image<
        Python::ImageContainerByVector2DUnsignedChar,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DUnsignedChar);

    // --------------- Points -------------
    // Point2D
    const std::string typestr_2DPoint2D = "ImageContainerByVector2DPoint2D";
    auto py_class_ImageContainerByVector2DPoint2D =
        declare_ImageContainerBySTLVector<
            Python::ImageContainerByVector2DPoint2D>(m, typestr_2DPoint2D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector2DPoint2D,
        typename Python::ImageContainerByVector2DPoint2D::Value::Component>
        (py_class_ImageContainerByVector2DPoint2D);
    add_set_from_image<
        Python::ImageContainerByVector2DPoint2D,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DPoint2D);

    // RealPoint2D
    const std::string typestr_2DRealPoint2D = "ImageContainerByVector2DRealPoint2D";
    auto py_class_ImageContainerByVector2DRealPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DRealPoint2D>(m, typestr_2DRealPoint2D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector2DRealPoint2D,
        typename Python::ImageContainerByVector2DRealPoint2D::Value::Component>
        (py_class_ImageContainerByVector2DRealPoint2D);
    add_set_from_image<
        Python::ImageContainerByVector2DRealPoint2D,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DRealPoint2D);

    // Point3D
    const std::string typestr_2DPoint3D = "ImageContainerByVector2DPoint3D";
    auto py_class_ImageContainerByVector2DPoint3D =
        declare_ImageContainerBySTLVector<
            Python::ImageContainerByVector2DPoint3D>(m, typestr_2DPoint3D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector2DPoint3D,
        typename Python::ImageContainerByVector2DPoint3D::Value::Component>
        (py_class_ImageContainerByVector2DPoint3D);
    add_set_from_image<
        Python::ImageContainerByVector2DPoint3D,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DPoint3D);

    // RealPoint3D
    const std::string typestr_2DRealPoint3D = "ImageContainerByVector2DRealPoint3D";
    auto py_class_ImageContainerByVector2DRealPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DRealPoint3D>(m, typestr_2DRealPoint3D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector2DRealPoint3D,
        typename Python::ImageContainerByVector2DRealPoint3D::Value::Component>
        (py_class_ImageContainerByVector2DRealPoint3D);
    add_set_from_image<
        Python::ImageContainerByVector2DRealPoint3D,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DRealPoint3D);

    //------ Other ------
    // Color
    const std::string typestr_2DColor = "ImageContainerByVector2DColor";
    auto py_class_ImageContainerByVector2DColor =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector2DColor>(m, typestr_2DColor );
    def_buffer_bridge_for_Color<
        Python::ImageContainerByVector2DColor>
        (py_class_ImageContainerByVector2DColor);
    add_set_from_image<
        Python::ImageContainerByVector2DColor,
        Python::DigitalSetZ2i>(py_class_ImageContainerByVector2DColor);
    // ------------------------------ 3D --------------------------------------
    // --------------- Single types --------------
    // Integer
    const std::string typestr_3DInteger = "ImageContainerByVector3DInteger";
    auto py_class_ImageContainerByVector3DInteger =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DInteger>(m, typestr_3DInteger );
    def_buffer_bridge(py_class_ImageContainerByVector3DInteger);
    add_set_from_image<
        Python::ImageContainerByVector3DInteger,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DInteger);

    // Real
    const std::string typestr_3DReal = "ImageContainerByVector3DReal";
    auto py_class_ImageContainerByVector3DReal =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DReal>(m, typestr_3DReal );
    def_buffer_bridge(py_class_ImageContainerByVector3DReal);
    add_set_from_image<
        Python::ImageContainerByVector3DReal,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DReal);

    // Float
    const std::string typestr_3DFloat = "ImageContainerByVector3DFloat";
    auto py_class_ImageContainerByVector3DFloat =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DFloat>(m, typestr_3DFloat );
    def_buffer_bridge(py_class_ImageContainerByVector3DFloat);
    add_set_from_image<
        Python::ImageContainerByVector3DFloat,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DFloat);

    // Short
    const std::string typestr_3DShort = "ImageContainerByVector3DShort";
    auto py_class_ImageContainerByVector3DShort =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DShort>(m, typestr_3DShort );
    def_buffer_bridge(py_class_ImageContainerByVector3DShort);
    add_set_from_image<
        Python::ImageContainerByVector3DShort,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DShort);

    // Unsigned char (Binary images)
    const std::string typestr_3DUnsignedChar = "ImageContainerByVector3DUnsignedChar";
    auto py_class_ImageContainerByVector3DUnsignedChar =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DUnsignedChar>(m, typestr_3DUnsignedChar );
    def_buffer_bridge(py_class_ImageContainerByVector3DUnsignedChar);
    add_set_from_image<
        Python::ImageContainerByVector3DUnsignedChar,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DUnsignedChar);

    // --------------- Points -------------
    // Point2D
    const std::string typestr_3DPoint2D = "ImageContainerByVector3DPoint2D";
    auto py_class_ImageContainerByVector3DPoint2D =
        declare_ImageContainerBySTLVector<
            Python::ImageContainerByVector3DPoint2D>(m, typestr_3DPoint2D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector3DPoint2D,
        typename Python::ImageContainerByVector3DPoint2D::Value::Component>
        (py_class_ImageContainerByVector3DPoint2D);
    add_set_from_image<
        Python::ImageContainerByVector3DPoint2D,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DPoint2D);

    // RealPoint2D
    const std::string typestr_3DRealPoint2D = "ImageContainerByVector3DRealPoint2D";
    auto py_class_ImageContainerByVector3DRealPoint2D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DRealPoint2D>(m, typestr_3DRealPoint2D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector3DRealPoint2D,
        typename Python::ImageContainerByVector3DRealPoint2D::Value::Component>
        (py_class_ImageContainerByVector3DRealPoint2D);
    add_set_from_image<
        Python::ImageContainerByVector3DRealPoint2D,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DRealPoint2D);

    // Point3D
    const std::string typestr_3DPoint3D = "ImageContainerByVector3DPoint3D";
    auto py_class_ImageContainerByVector3DPoint3D =
        declare_ImageContainerBySTLVector<
            Python::ImageContainerByVector3DPoint3D>(m, typestr_3DPoint3D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector3DPoint3D,
        typename Python::ImageContainerByVector3DPoint3D::Value::Component>
        (py_class_ImageContainerByVector3DPoint3D);
    add_set_from_image<
        Python::ImageContainerByVector3DPoint3D,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DPoint3D);

    // RealPoint3D
    const std::string typestr_3DRealPoint3D = "ImageContainerByVector3DRealPoint3D";
    auto py_class_ImageContainerByVector3DRealPoint3D =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DRealPoint3D>(m, typestr_3DRealPoint3D );
    def_buffer_bridge_for_PointVector<
        Python::ImageContainerByVector3DRealPoint3D,
        typename Python::ImageContainerByVector3DRealPoint3D::Value::Component>
        (py_class_ImageContainerByVector3DRealPoint3D);
    add_set_from_image<
        Python::ImageContainerByVector3DRealPoint3D,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DRealPoint3D);

    //------ Other ------
    // Color
    const std::string typestr_3DColor = "ImageContainerByVector3DColor";
    auto py_class_ImageContainerByVector3DColor =
        declare_ImageContainerBySTLVector<Python::ImageContainerByVector3DColor>(m, typestr_3DColor );
    def_buffer_bridge_for_Color<
        Python::ImageContainerByVector3DColor>
        (py_class_ImageContainerByVector3DColor);
    add_set_from_image<
        Python::ImageContainerByVector3DColor,
        Python::DigitalSetZ3i>(py_class_ImageContainerByVector3DColor);

}
