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

#ifndef DGTAL_KHALIMSKYPRESPACEND_DECLARE_PY_H
#define DGTAL_KHALIMSKYPRESPACEND_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/KhalimskyPreSpaceND.h"
#include "KhalimskyPreSpaceND_types_py.h"

template<typename TKhalimskyPreCell>
pybind11::class_<TKhalimskyPreCell> declare_KhalimskyPreCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyPreCell;
    using TTInteger = typename TKhalimskyPreCell::Integer;
    using TTPoint = typename TKhalimskyPreCell::Point;
    auto py_class = py::class_<TT>(m, typestr.c_str(), py::buffer_protocol());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init([](){ return TT();}));
    py_class.def(py::init<const TTPoint &>());
    py_class.def(py::init<const TT &>());

    // ----------------------- Bridges ----------------------------------------
    // Python buffers (requires py::buffer_protocol in py_class instantiation)
    // The only data is the coordinates (PointVector)
    py_class.def_buffer([](TT &self) -> py::buffer_info {
        return py::buffer_info(
            self.coordinates.data(),                    /* Pointer to buffer */
            static_cast<ssize_t>(sizeof(TTInteger)),    /* Size of one scalar */
            py::format_descriptor<TTInteger>::format(), /* Python struct-style format descriptor */
            1,                                          /* Number of dimensions */
            { TTPoint::dimension },                     /* Shape, buffer dimensions */
            { static_cast<ssize_t>(sizeof(TTInteger)) } /* Strides (in bytes) for each index */
            );
        });

    py_class.def(py::init([](py::buffer buf) {
        /* Note(phcerdan): Adapted from numpy/stl_bind.h vector_buffer */
        /* Request a buffer descriptor from Python */
        auto info = buf.request();

        /* Sanity checks */
        if (info.ndim != 1 || info.strides[0] % static_cast<ssize_t>(sizeof(TTInteger)))
            throw py::type_error("Only valid 1D buffers can be copied to a PointVector");
        if (!py::detail::compare_buffer_info<TTInteger>::compare(info) || (ssize_t) sizeof(TTInteger) != info.itemsize)
            throw py::type_error("Format mismatch (Python: " + info.format + " C++: " + py::format_descriptor<TTInteger>::format() + ")");

        if(info.shape[0] != TTPoint::dimension)
            throw py::type_error("Shape missmatch (Python: " + std::to_string(info.shape[0]) + " C++: " + std::to_string(TTPoint::dimension) + ")");

        TTInteger *p = static_cast<TTInteger*>(info.ptr);
        return TT(TTPoint(p));
        }));
    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", [](const TT & self) {
            return self.coordinates.size();
            });
    py_class.def("__getitem__", [](const TT & self, const size_t index) {
        if (index >= self.coordinates.size()) throw py::index_error();
        return self.coordinates[index];
        });
    py_class.def("__setitem__", [](TT & self, const size_t index,
                const TTInteger value) {
        if (index >= self.coordinates.size()) throw py::index_error();
        self.coordinates[index] = value;
        });

    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.coordinates.begin(), self.coordinates.end()); },
        py::keep_alive<0, 1>() /* keep object alive while iterator exists */);

    // ----------------------- Pickling ---------------------------------------
    py_class.def(py::pickle(
            [](const TT & self) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(self.coordinates);
            },
            [](py::tuple t) { //__setstate__
            if(t.size() != 1) {
                throw std::runtime_error("Invalid state!");
            }
            TT cell(t[0].cast<TTPoint>());
            return cell;
            }
            ));

    // Comparisons
    py_class.def(py::self == py::self);
    py_class.def(py::self != py::self);
    py_class.def(py::self < py::self);

    // ----------------------- Class functions --------------------------------
    py_class.def("preCell", &TT::preCell, R"(Underlying constant preCell (itself))");

    // ----------------------- Class data -------------------------------------
    //
    py_class.def_readwrite("coordinates", &TT::coordinates, R"(Khalimsky coordinates)");
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TTPoint::dimension; },
            R"(The dimension of the KhalimskyPreCell.)");
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });

    // ----------------------- Print / Display --------------------------------

    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        auto py_coordinates = py::cast(self.coordinates);
        os << "coordinates: " << py_coordinates.attr("__str__")();
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_coordinates = py::cast(self.coordinates);
        os << py_coordinates.attr("__repr__")();
        os << ")";
        return os.str();
    });
    return py_class;
}

template<typename TKhalimskyPreSpaceND>
pybind11::class_<TKhalimskyPreSpaceND> declare_KhalimskyPreSpaceND(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyPreSpaceND;
    using TTCell = typename TT::Cell;
    using TTPreCell = typename TT::PreCell;
    using TTSCell = typename TT::SCell;
    using TTSPreCell = typename TT::SPreCell;
    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("__getitem__", [](const TT &self, const size_t index) {
        if (index >= self.size()) throw py::index_error();
        return self[index];
        });
    py_class.def("__setitem__", [](TT &self, const size_t index,
            const typename TT::Component value) {
        if (index >= self.size()) throw py::index_error();
        self[index] = value;
        });
    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.begin(), self.end()); },
         py::keep_alive<0, 1>() /* Keep object alive while iterator exists */);

    // ----------------------- Class operators --------------------------------

    // Comparisons
    py_class.def(py::self == py::self);
    py_class.def(py::self != py::self);
    py_class.def(py::self < py::self);

    // ----------------------- Class functions --------------------------------

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly_static("TPreCell",
            [](py::object /* self */) {
            return py::type::of<TTPreCell>();
            });
    py_class.def_property_readonly_static("TSPreCell",
            [](py::object /* self */) {
            return py::type::of<TTSPreCell>();
            });
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the KhalimskyPreSpace.)");
    py_class.def_property_readonly_static("POS",
            [](py::object /* self */) { return TT::POS; },
            R"(Positive Sign.)");
    py_class.def_property_readonly_static("NEG",
            [](py::object /* self */) { return TT::NEG; },
            R"(Negative Sign.)");

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
        auto py_X = py::cast(self.myX);
        os << py_X.attr("__repr__")();
        os << ", ";
        os << ")";
        return os.str();
    });

    return py_class;
}
#endif
