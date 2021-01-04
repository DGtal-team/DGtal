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

#ifndef DGTAL_HYPERRECTDOMAIN_DECLARE_PY_H
#define DGTAL_HYPERRECTDOMAIN_DECLARE_PY_H

#include "dgtal_pybind11_common.h"
#include <pybind11/pytypes.h>

#include "base/Common_types_py.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

template<typename THyperRectDomain>
pybind11::class_<THyperRectDomain> declare_HyperRectDomain(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = THyperRectDomain;
    using TTSpace = typename THyperRectDomain::Space;
    using TTPoint = typename THyperRectDomain::Point;
    using TTRealPoint = typename TTSpace::RealPoint;
    const std::string docs =
R"(Parallelepidec region of a digital space, model of a 'CDomain'.

The following code snippet demonstrates how to use \p HyperRectDomain

    import dgtal
    Domain = dgtal.kernel.DomainZ2i
    Point = Domain.TPoint
    lower_bound = Point.zero
    upper_bound = Point.diagonal(4)
    dom = Domain(lower_bound, upper_bound)
)";
    auto py_class = py::class_<TT>(m, typestr.c_str(), docs.c_str());

    py_class.def("dtype", [](const TT &self) {
            return DGtal::Python::Integer_str;
            });
    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());
    py_class.def(py::init<const TTPoint &, const TTPoint&>());
    py_class.def(py::init<const TTRealPoint &, const TTRealPoint&>());
    py_class.def(py::init<const TT &>());

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("__contains__", &TT::isInside);
    py_class.def("__iter__", [](const TT &self) {
        return py::make_iterator(self.begin(), self.end()); },
         py::keep_alive<0, 1>() /* Keep object alive while iterator exists */);

    // ----------------------- Class operators --------------------------------

    // ----------------------- Class functions --------------------------------
    py_class.def("is_empty", &TT::isEmpty, R"(True if the domain is empty)");
    py_class.def("size", &TT::size, "Return the number of points of the domain.");

    // ----------------------- Class data -------------------------------------
    py_class.def_readwrite("lower_bound", &TT::myLowerBound);
    py_class.def_readwrite("upper_bound", &TT::myUpperBound);

    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });
    py_class.def_property_readonly_static("TRealPoint",
            [](py::object /* self */) {
            return py::type::of<TTRealPoint>();
            });
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the domain.)");

    // ----------------------- Print / Display --------------------------------
    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_LowerBound = py::cast(self.myLowerBound);
        os << py_LowerBound.attr("__repr__")();
        os << ", ";
        auto py_UpperBound = py::cast(self.myUpperBound);
        os << py_UpperBound.attr("__repr__")();
        os << ")";
        return os.str();
        });

    return py_class;
}
#endif
