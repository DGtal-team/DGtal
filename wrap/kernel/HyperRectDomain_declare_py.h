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

#if defined (_MSC_VER) and !defined(ssize_t)
    // ssize_t is not standard, only posix which is not supported by MSVC
    #define ssize_t ptrdiff_t
#endif

#include "dgtal_nanobind_common.h"
// pytypes not needed in nanobind

#include "base/Common_types_py.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"

template<typename THyperRectDomain>
nanobind::class_<THyperRectDomain> declare_HyperRectDomain(nanobind::module_ &m,
    const std::string &typestr) {
    namespace nb = nanobind;
    using namespace nanobind::literals;
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
    auto nb_class = nb::class_<TT>(m, typestr.c_str(), docs.c_str());

    nb_class.def("dtype", [](const TT &) {
            return DGtal::Python::Integer_str;
            });
    // ----------------------- Constructors -----------------------------------
    nb_class.def(nb::init(), "Default constructor.");

    nb_class.def(nb::init<const TTPoint &, const TTPoint&>(),
R"(Constructor from two points lower_bound, upper_bound defining the space diagonal.)",
        nb::arg("lower_bound"), nb::arg("upper_bound"));

    nb_class.def(nb::init<const TTRealPoint &, const TTRealPoint&>(),
R"(Constructor from two points lower_bound, upper_bound with
real coordinates defining the space diagonal.

The domain actualy defined is the smallest domain with integer bounds that
contains the two given points.
)",
        nb::arg("lower_bound"), nb::arg("upper_bound"));

    nb_class.def(nb::init<const TT &>());

    // ----------------------- Python operators -------------------------------
    nb_class.def("__len__", &TT::size);
    nb_class.def("__contains__", &TT::isInside);
    nb_class.def("__iter__", [](const TT &self) {
        return nb::make_iterator(self.begin(), self.end()); },
         nb::keep_alive<0, 1>() /* Keep object alive while iterator exists */);

    // ----------------------- Class operators --------------------------------

    // ----------------------- Class functions --------------------------------
    nb_class.def("is_empty", &TT::isEmpty, R"(True if the domain is empty)");
    nb_class.def("size", &TT::size, "Return the number of points of the domain.");

    // ----------------------- Class data -------------------------------------
    nb_class.def_readwrite("lower_bound", &TT::myLowerBound);
    nb_class.def_readwrite("upper_bound", &TT::myUpperBound);

    nb_class.def_property_readonly_static("TPoint",
            [](nb::object /* self */) {
            return nb::type::of<TTPoint>();
            });
    nb_class.def_property_readonly_static("TRealPoint",
            [](nb::object /* self */) {
            return nb::type::of<TTRealPoint>();
            });
    nb_class.def_property_readonly_static("dimension",
            [](nb::object /* self */) { return TT::dimension; },
            R"(The dimension of the domain.)");

    // ----------------------- Print / Display --------------------------------
    nb_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    nb_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_LowerBound = nb::cast(self.myLowerBound);
        os << py_LowerBound.attr("__repr__")();
        os << ", ";
        auto py_UpperBound = nb::cast(self.myUpperBound);
        os << py_UpperBound.attr("__repr__")();
        os << ")";
        return os.str();
        });

    return nb_class;
}
#endif
