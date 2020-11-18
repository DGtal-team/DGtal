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

#ifndef DGTAL_DIGITALSETBYSTLVECTOR_DECLARE_PY_H
#define DGTAL_DIGITALSETBYSTLVECTOR_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"

template<typename TDigitalSetBySTLVector>
pybind11::class_<TDigitalSetBySTLVector> declare_DigitalSetBySTLVector(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TDigitalSetBySTLVector;
    using TTDomain = typename TDigitalSetBySTLVector::Domain;
    using TTPoint = typename TDigitalSetBySTLVector::Point;
    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init<const TTDomain &>());
    py_class.def(py::init<const TT &>());

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.begin(), self.end()); },
         py::keep_alive<0, 1>() /* Keep object alive while iterator exists */);

    // ----------------------- Class operators --------------------------------

    // Arithmetic
    py_class.def(py::self += py::self);

    // ----------------------- Class functions --------------------------------
    py_class.def("insert", [](TT & self, const TTPoint & p) {
            self.insert(p);
            }, R"(
Adds point [p] to this set.
The point should belong to the associated domain.)");

    py_class.def("erase", [](TT & self, const TTPoint & p) {
            self.erase(p);
            }, R"(
Removes point [p] to the set.
Returns the number of removed elements (0 or 1).)");

    py_class.def("clear", &TT::clear, R"(
Clears the set.
Post: The set is empty.)");

    py_class.def("size", &TT::size, R"(
Returns the number of elements in the set.)");

    py_class.def("empty", &TT::empty, R"(
Returns true if the set is empty.)");

    py_class.def("bounding_box", [](TT & self) {
            auto lower = TTPoint();
            auto upper = TTPoint();
            self.computeBoundingBox(lower, upper);
            return std::array<TTPoint, 2>({lower, upper});
            }, R"(
Computes the bounding_box of this set.
Returns [lower_bound, upper_bound].)");

    py_class.def("complement", [](TT & self) {
            auto self_copy = self;
            self_copy.assignFromComplement(self);
            return self_copy;
            }, R"(
Returns a new set which is the complement in the domain of this set.)");

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly("domain", &TT::domain);
    py_class.def_property_readonly_static("TDomain",
            [](py::object /* self */) {
            return py::type::of<TTDomain>();
            });
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });

    // ----------------------- Print / Display --------------------------------
    py_class.def("__str__", [typestr](const TT & self) {
        std::stringstream os;
        os << "type: " << typestr << "\n";
        os << "size: " << self.size() << "\n";
        os << "domain: ";
        auto py_domain = py::cast(self.domain());
        os << py_domain.attr("__repr__")();
        os << "\n";
        os << "points:\n";
        os << "[ ";
        for ( auto it = self.begin(); it != self.end(); ++it) {
            auto point = *it;
            os << "{ ";
                for ( size_t i = 0; i < TTPoint::dimension; ++i) {
                os << point[i];
                if(i < TTPoint::dimension - 1) {
                    os << ", ";
                    }
                }
            os << " }";
            os << ", ";
        }
        os << " ]";
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_domain = py::cast(self.domain());
        os << py_domain.attr("__repr__")();
        os << ", ";
        os << ")";
        return os.str();
    });

    return py_class;
}
#endif
