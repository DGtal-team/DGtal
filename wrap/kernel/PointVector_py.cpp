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
#include "dgtal_python_types.h"

#include "PointVector_declare_py.h"

namespace py = pybind11;
using namespace DGtal;

void init_PointVector(py::module & m) {
    using Point2D = PointVector<2, DGtal::PythonInteger>;
    using Point3D = PointVector<3, DGtal::PythonInteger>;
    using RealPoint2D = PointVector<2, DGtal::PythonReal>;
    using RealPoint3D = PointVector<3, DGtal::PythonReal>;

    auto py_class_Point2D = declare_PointVector<Point2D>(m,"Point2D", DGtal::PythonInteger_str);
    declare_PointVector_all_mixings<RealPoint2D>(py_class_Point2D);

    auto py_class_Point3D = declare_PointVector<Point3D>(m,"Point3D", DGtal::PythonInteger_str);
    declare_PointVector_all_mixings<RealPoint3D>(py_class_Point3D);

    auto py_class_RealPoint2D = declare_PointVector<RealPoint2D>(m,"RealPoint2D", DGtal::PythonReal_str);
    declare_PointVector_all_mixings<Point2D>(py_class_RealPoint2D);

    auto py_class_RealPoint3D = declare_PointVector<RealPoint3D>(m,"RealPoint3D", DGtal::PythonReal_str);
    declare_PointVector_all_mixings<Point3D>(py_class_RealPoint3D);

}

