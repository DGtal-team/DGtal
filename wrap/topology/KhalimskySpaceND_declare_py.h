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

#ifndef DGTAL_KHALIMSKYSPACEND_DECLARE_PY_H
#define DGTAL_KHALIMSKYSPACEND_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/KhalimskySpaceND.h"
#include "KhalimskySpaceND_types_py.h"

template<typename TKhalimskyCell>
pybind11::class_<TKhalimskyCell> declare_KhalimskyCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyCell;
    using TTPreCell = typename TKhalimskyCell::PreCell;
    using TTInteger = typename TKhalimskyCell::Integer;
    using TTPoint = typename TKhalimskyCell::Point;
    auto py_class = py::class_<TT>(m, typestr.c_str(), py::buffer_protocol());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init([](){ return TT();}));
    py_class.def(py::init<const TT &>());

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", [](const TT & self) {
            return self.preCell().coordinates.size();
            });
    py_class.def("__getitem__", [](const TT & self, const size_t index) {
        if (index >= self.preCell().coordinates.size()) throw py::index_error();
        return self.preCell().coordinates[index];
        });

    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.preCell().coordinates.begin(), self.preCell().coordinates.end()); },
        py::keep_alive<0, 1>() /* keep object alive while iterator exists */);

    // ------------------ Pickling: only __getstate__ -------------------------
    py_class.def("__getstate__",
            [](const TT & self) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(self.preCell().coordinates);
            });

    // Comparisons
    py_class.def(py::self == py::self);
    py_class.def(py::self != py::self);
    py_class.def(py::self < py::self);

    // ----------------------- Class functions --------------------------------
    py_class.def_property_readonly("pre_cell", [](const TT& self) {
            return self.preCell();
            }, R"(Underlying constant preCell (read-only) )");
    py_class.def("preCell", &TT::preCell, R"(Underlying constant preCell)");

    // ----------------------- Class data -------------------------------------
    //
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TTPoint::dimension; },
            R"(The dimension of the KhalimskyCell.)");
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });

    // ----------------------- Print / Display --------------------------------

    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        auto py_coordinates = py::cast(self.preCell().coordinates);
        os << "preCell.coordinates: " << py_coordinates.attr("__str__")();
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_coordinates = py::cast(self.preCell().coordinates);
        os << py_coordinates.attr("__repr__")();
        os << ")";
        return os.str();
    });
    return py_class;
}
#endif
