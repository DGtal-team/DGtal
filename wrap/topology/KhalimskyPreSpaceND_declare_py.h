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

/**
 * Shared implementation between KhalimskyPreCell and SignedKhalimskyPreCell.
 * They differ in contructors and extra sign member in SignedKhalimskyPreCell.
 *
 * @return the respective py:class_ of the type.
 */
template<typename TKhalimskyPreCell>
pybind11::class_<TKhalimskyPreCell> declare_KhalimskyPreCell_common(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyPreCell;
    using TTInteger = typename TT::Integer;
    using TTPoint = typename TT::Point;
    auto py_class = py::class_<TT>(m, typestr.c_str(), py::buffer_protocol());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init([](){ return TT();}));
    py_class.def(py::init<const TT &>());

    // ----------------------- Bridges ----------------------------------------
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

    return py_class;
}

template<typename TKhalimskyPreCell>
pybind11::class_<TKhalimskyPreCell> declare_KhalimskyPreCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyPreCell;
    using TTPoint = typename TT::Point;
    using TTInteger = typename TT::Integer;
    auto py_class = declare_KhalimskyPreCell_common<TT>(m, typestr);
    py_class.def(py::init<const TTPoint &>());

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

template<typename TSignedKhalimskyPreCell>
pybind11::class_<TSignedKhalimskyPreCell> declare_SignedKhalimskyPreCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TSignedKhalimskyPreCell;
    using TTPoint = typename TSignedKhalimskyPreCell::Point;
    auto py_class = declare_KhalimskyPreCell_common<TT>(m, typestr);
    py_class.def(py::init<const TTPoint &, bool>());
    py_class.def_readwrite("positive", &TT::positive, R"(Cell sign.)");

    // ----------------------- Pickling ---------------------------------------
    py_class.def(py::pickle(
            [](const TT & self) { // __getstate__
            /* Return a tuple that fully encodes the state of the object */
            return py::make_tuple(self.coordinates, self.positive);
            },
            [](py::tuple t) { //__setstate__
            if(t.size() != 2) {
                throw std::runtime_error("Invalid state!");
            }
            TT cell(t[0].cast<TTPoint>(), t[1].cast<bool>());
            return cell;
            }
            ));

    // ----------------------- Print / Display --------------------------------
    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        os << "positive: " << self.positive << "\n";
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
        os << ", " << self.positive;
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
    using TTInteger = typename TT::Integer;
    using TTPoint = typename TT::Point;
    using TTDimension = typename TTPoint::Dimension;
    using TTCell = typename TT::Cell;
    using TTSign = typename TT::Sign;
    using TTSCell = typename TT::SCell;

    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    // Not needed, all member functions are static, and there is no data.
    py_class.def(py::init());

    // ----------------------- Class functions --------------------------------
    // ----------------------- Pre-cell creation services --------------------------
    py_class.def_static("uCell", py::detail::overload_cast_impl<const TTPoint&>()(&TT::uCell),
R"(From the Khalimsky coordinates of a cell, builds the corresponding unsigned pre-cell.

Parameters
----------
kpoint: Point
    Khalimsky coordinates of a cell.

Return
------
    Unsigned precell.
)", py::arg("kpoint"));

    py_class.def_static("uCell", py::detail::overload_cast_impl<TTPoint, const TTCell&>()(&TT::uCell),
R"(From the digital coordinates of a point in Zn and a cell type, builds the corresponding unsigned pre-cell.

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
cell: Cell
    Another cell defining the topology.

Return
------
    The pre-cell having the topology of [cell] and the given digital coordinates [p].
)", py::arg("point"), py::arg("cell"));

    py_class.def_static("sCell", py::detail::overload_cast_impl<const TTPoint&, TTSign>()(&TT::sCell),
R"(From the Khalimsky coordinates of a cell and a sign, builds the corresponding signed pre-cell.

Parameters
----------
kpoint: Point
    Khalimsky coordinates of a cell.
sign: Bool
    The sign of the cell (either POS (True) or NEG (False)), defaults to True.

Return
------
    Signed precell.
)", py::arg("kpoint"), py::arg("sign")=true);

    py_class.def_static("sCell", py::detail::overload_cast_impl<TTPoint, const TTSCell&>()(&TT::sCell),
R"(From the digital coordinates of a point in Zn and a signed cell type, builds the corresponding signed pre-cell.

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
cell: Cell
    Another cell defining the topology and sign.

Return
------
    The pre-cell having the topology and sign of [cell] and the given digital coordinates [p].
)", py::arg("point"), py::arg("cell"));

    py_class.def_static("uSpel", &TT::uSpel,
R"(From the digital coordinates of a point in Zn, builds the corresponding pre-spel (pre-cell of maximal dimension).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).

Return
------
    The pre-spel having the given digital coordinates [p].
)", py::arg("point"));

    py_class.def_static("sSpel", &TT::sSpel,
R"(From the digital coordinates of a point in Zn and a cell sign, builds the corresponding signed pre-spel (pre-cell of maximal dimension).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
sign: Bool
    The sign of the cell (either POS (True) or NEG (False), defaults to True.

Return
------
    The signed pre-spel having the given digital coordinates [p] and [sign].
)", py::arg("point"), py::arg("sign")=true);

    py_class.def_static("uPointel", &TT::uPointel,
R"(From the digital coordinates of a point in Zn, builds the corresponding pre-pointel (pre-cell of dimension 0).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).

Return
------
    The pre-pointel having the given digital coordinates [p].
)", py::arg("point"));

    py_class.def_static("sPointel", &TT::sPointel,
R"(From the digital coordinates of a point in Zn and a cell sign, builds the corresponding signed pre-pointel (pre-cell dimension 0).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
sign: Bool
    The sign of the cell (either POS (True) or NEG (False), defaults to True.

Return
------
    The signed pre-pointel having the given digital coordinates [p] and [sign].
)", py::arg("point"), py::arg("sign")=true);

    // ----------------------- Read accessors to pre-cells ------------------------

    py_class.def_static("uKCoord", &TT::uKCoord,
R"(Return its Khalimsky coordinate along [dim].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    Any valid dimension.

Return
------
    The Khalimsky coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uCoord", &TT::uCoord,
R"(Return its digital coordinate along [dim].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    Any valid dimension.

Return
------
    The digital coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uKCoords", &TT::uKCoords,
R"(Return its Khalimsky coordinates.

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    The Khalimsky coordinates of input [cell].
)", py::arg("cell"));

    py_class.def_static("uCoords", &TT::uCoords,
R"(Return its digital coordinates.

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    The digital coordinates of input [cell].
)", py::arg("cell"));

    py_class.def_static("sKCoord", &TT::sKCoord,
R"(Return its Khalimsky coordinate along [dim].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    Any valid dimension.

Return
------
    The Khalimsky coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sCoord", &TT::sCoord,
R"(Return its digital coordinate along [dim].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    Any valid dimension.

Return
------
    The digital coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sKCoords", &TT::sKCoords,
R"(Return its Khalimsky coordinates.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The Khalimsky coordinates of input [cell].
)", py::arg("cell"));

    py_class.def_static("sCoords", &TT::sCoords,
R"(Return its digital coordinates.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The digital coordinates of input [cell].
)", py::arg("cell"));

    py_class.def_static("sSign", &TT::sSign,
R"( Return its sign.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The sign of input [cell].
)", py::arg("cell"));

    // ----------------------- Write accessors to pre-cells ------------------------

    py_class.def_static("uSetKCoord", &TT::uSetKCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def_static("sSetKCoord", &TT::sSetKCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def_static("uSetCoord", &TT::uSetCoord,
R"(Sets the [dim]-th digital coordinate of [cell] to [i].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def_static("sSetCoord", &TT::sSetCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def_static("uSetKCoords", &TT::uSetKCoords,
R"(Sets the Khalimsky coordinates of [cell] to [kp]

Parameters
----------
cell: Cell
    Any unsigned pre-cell
kp: Point
    The new Khalimsky coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def_static("sSetKCoords", &TT::sSetKCoords,
R"(Sets the Khalimsky coordinates of [cell] to [kp]

Parameters
----------
cell: SCell
    Any signed pre-cell
kp: Point
    The new Khalimsky coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def_static("uSetCoords", &TT::uSetCoords,
R"(Sets the digital coordinates of [cell] to [kp]

Parameters
----------
cell: Cell
    Any unsigned pre-cell
kp: Point
    The new digital coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def_static("sSetCoords", &TT::sSetCoords,
R"(Sets the digital coordinates of [cell] to [kp]

Parameters
----------
cell: SCell
    Any signed pre-cell
kp: Point
    The new digital coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def_static("sSetSign", &TT::sSetSign,
R"(Sets the sign of the cell

Parameters
----------
cell: SCell
    Any signed pre-cell
s: Bool
    Any sign (positive or negative)
)", py::arg("cell"), py::arg("sign"));

    // -------------------- Conversion signed/unsigned ------------------------
    py_class.def_static("signs", &TT::signs,
R"(Creates a signed pre-cell from an unsigned one and a given sign.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
s: Bool
    Any sign (positive or negative)

Return
------
    The signed version of the input pre-cell with input sign.
)", py::arg("cell"), py::arg("sign"));

    py_class.def_static("unsigns", &TT::unsigns,
R"(Creates an unsigned pre-cell from an signed one.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The unsigned version of the input signed pre-cell.
)", py::arg("cell"));

    py_class.def_static("sOpp", &TT::sOpp,
R"(Creates an signed pre-cell with the inverse sign of input cell.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The pre-cell with opposite sign than the input cell.
)", py::arg("cell"));

    // ------------------------- Pre-cell topology services -----------------------
    py_class.def_static("uTopology", &TT::uTopology,
R"(Returns the topology word of the input cell.

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    The topology word of [cell]
)", py::arg("cell"));

    py_class.def_static("sTopology", &TT::sTopology,
R"(Returns the topology word of the input cell.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The topology word of [cell]
)", py::arg("cell"));

    py_class.def_static("uDim", &TT::uDim,
R"(Returns the dimension of the input cell.
If the cell lives in 3D, the resulting uDim(cell):
0 -> Pointel
1 -> Linel
2 -> Surfel
3 -> Voxel

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    The dim word of [cell]
)", py::arg("cell"));

    py_class.def_static("sDim", &TT::sDim,
R"(Returns the dimension of the input cell.
If the cell lives in 3D, the resulting sDim(cell):
0 -> Pointel
1 -> Linel
2 -> Surfel
3 -> Voxel

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    The dimension of [cell]
)", py::arg("cell"));

    py_class.def_static("uIsSurfel", &TT::uIsSurfel,
R"(Returns true if input cell is a surfel (spans all but one coordinate).

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    True if input cell is a surfel (spans all but one coordinate).
)", py::arg("cell"));


    py_class.def_static("sIsSurfel", &TT::sIsSurfel,
R"(Returns true if input cell is a surfel (spans all but one coordinate).

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    True if input cell is a surfel (spans all but one coordinate).
)", py::arg("cell"));

    py_class.def_static("uIsOpen", &TT::uIsOpen,
R"(Returns true if input cell is open along the direction [dim].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    Any direction

Return
------
    True if input cell is open along the direction [dim].
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIsOpen", &TT::sIsOpen,
R"(Returns true if input cell is open along the direction [dim].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    Any direction

Return
------
    True if input cell is open along the direction [dim].
)", py::arg("cell"), py::arg("dim"));

    // -------------- Iterator services for pre-cells (not wrapped) -----------
    // -------------------- Unsigned pre-cell geometry services --------------------

    py_class.def_static("uGetIncr", &TT::uGetIncr,
R"(Return the same element as [cell] except for the incremented coordinate [dim].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The changed coordinate

Return
------
    Cell with coordinate [dim] incremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uIsMax", &TT::uIsMax,
R"(Check if input cell is out of the space.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The tested coordinate

Return
------
    Always False for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uIsInside", py::detail::overload_cast_impl<const TTCell&, TTDimension>()(&TT::uIsInside),
R"(Check if the coordinate [] of input [cell] is inside the space.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The tested coordinate

Return
------
    Always True for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uIsInside", py::detail::overload_cast_impl<const TTCell&>()(&TT::uIsInside),
R"(Check if input cell is inside the space.

Parameters
----------
cell: Cell
    Any unsigned pre-cell

Return
------
    Always True for PreKhalimskySpace
)", py::arg("cell"));

    py_class.def_static("uGetDecr", &TT::uGetDecr,
R"(Return the same element as [cell] except for the decremented coordinate [dim].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The changed coordinate

Return
------
    Cell with coordinate [dim] decremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uIsMin", &TT::uIsMin,
R"(Check if input cell is out of the space.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The tested coordinate

Return
------
    Always False for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("uGetAdd", &TT::uGetAdd,
R"(Return the same element as [cell] except for the coordinate [dim] incremented by x.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The changed coordinate
x: Int
    The increment

Return
------
    Cell with coordinate [dim] incremented with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def_static("uGetSub", &TT::uGetSub,
R"(Return the same element as [cell] except for the coordinate [dim] decremeneted by x.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
dim: Int
    The changed coordinate
x: Int
    The decrement

Return
------
    Cell with coordinate [dim] decremeneted with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def_static("uTranslation", &TT::uTranslation,
R"(Add the vector [vec] to the input [cell].

Parameters
----------
cell: Cell
    Any unsigned pre-cell
vec: Point
    A vector

Return
------
    The unsigned cell resulting from the [cell] translated by [vec]
)", py::arg("cell"), py::arg("vec"));

    py_class.def_static("uProjection", &TT::uProjection,
R"(The projection of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

Pre:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProjection(p, bound, k))`.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
bound: Cell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

Return
------
    The projection of [cell] along the [dim]th direction toward [bound].

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def_static("uProject", &TT::uProject,
R"(Modifies input cell. Projects of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

Pre:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProject(p, bound, k))`.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
bound: Cell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def_static("uNext", &TT::uNext,
R"(Increment the [cell] to its next position (as classically done in a scanning)

Pre: `uTopology(p) == uTopology(lower) == uTopology(upper)`.

Parameters
----------
cell: Cell
    Any unsigned pre-cell
lower: Cell
    The lower bound.
upper: Cell
    The upper bound.

Return
------
    True if [cell] is still withing the bounds, false if the scanning is finished.

)", py::arg("cell"), py::arg("lower"), py::arg("upper"));

    // -------------------- Signed pre-cell geometry services --------------------

    py_class.def_static("sGetIncr", &TT::sGetIncr,
R"(Return the same element as [cell] except for the incremented coordinate [dim].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The changed coordinate

Return
------
    SCell with coordinate [dim] incremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIsMax", &TT::sIsMax,
R"(Check if input cell is out of the space.

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The tested coordinate

Return
------
    Always False for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIsInside", py::detail::overload_cast_impl<const TTSCell&,
            TTDimension>()(&TT::sIsInside),
R"(Check if the coordinate [] of input [cell] is inside the space.

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The tested coordinate

Return
------
    Always True for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIsInside", py::detail::overload_cast_impl<const TTSCell&>
            ()(&TT::sIsInside),
R"(Check if input cell is inside the space.

Parameters
----------
cell: SCell
    Any signed pre-cell

Return
------
    Always True for PreKhalimskySpace
)", py::arg("cell"));

    py_class.def_static("sGetDecr", &TT::sGetDecr,
R"(Return the same element as [cell] except for the decremented coordinate [dim].

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The changed coordinate

Return
------
    SCell with coordinate [dim] decremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIsMin", &TT::sIsMin,
R"(Check if input cell is out of the space.

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The tested coordinate

Return
------
    Always False for PreKhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sGetAdd", &TT::sGetAdd,
R"(Return the same element as [cell] except for the coordinate [dim] incremented by x.

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The changed coordinate
x: Int
    The increment

Return
------
    SCell with coordinate [dim] incremented with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def_static("sGetSub", &TT::sGetSub,
R"(Return the same element as [cell] except for the coordinate [dim] decremeneted by x.

Parameters
----------
cell: SCell
    Any signed pre-cell
dim: Int
    The changed coordinate
x: Int
    The decrement

Return
------
    SCell with coordinate [dim] decremeneted with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def_static("sTranslation", &TT::sTranslation,
R"(Add the vector [vec] to the input [cell].

Parameters
----------
cell: SCell
    Any signed pre-cell
vec: Point
    A vector

Return
------
    The signed cell resulting from the [cell] translated by [vec]
)", py::arg("cell"), py::arg("vec"));

    py_class.def_static("sProjection", &TT::sProjection,
R"(The projection of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

Pre:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProjection(p, bound, k))`.

Parameters
----------
cell: SCell
    Any signed pre-cell
bound: SCell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

Return
------
    The projection of [cell] along the [dim]th direction toward [bound].

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def_static("sProject", &TT::sProject,
R"(Modifies input cell. Projects of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

Pre:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProject(p, bound, k))`.

Parameters
----------
cell: SCell
    Any signed pre-cell
bound: SCell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def_static("sNext", &TT::sNext,
R"(Increment the [cell] to its next position (as classically done in a scanning)

Pre: `uTopology(p) == uTopology(lower) == uTopology(upper)`.

Parameters
----------
cell: SCell
    Any signed pre-cell
lower: SCell
    The lower bound.
upper: SCell
    The upper bound.

Return
------
    True if [cell] is still withing the bounds, false if the scanning is finished.

)", py::arg("cell"), py::arg("lower"), py::arg("upper"));


    // ----------------------- Neighborhood services --------------------------

    py_class.def_static("uNeighborhood", &TT::uNeighborhood,
R"(Computes the 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell].

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The pre-cells of 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def_static("sNeighborhood", &TT::sNeighborhood,
R"(Computes the 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell].

Parameters
----------
cell: SCell
    Input signed pre-cell

Return
------
    The pre-cells of 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def_static("uProperNeighborhood", &TT::uProperNeighborhood,
R"(Computes the proper 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell] and different from [cell].

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The pre-cells of the proper 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def_static("sProperNeighborhood", &TT::sProperNeighborhood,
R"(Computes the proper 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell] and different from [cell].

Parameters
----------
cell: SCell
    Input signed pre-cell

Return
------
    The pre-cells of the proper 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def_static("uAdjacent", &TT::uAdjacent,
R"(Returns the cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].

Note: It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.

Parameters
----------
cell: Cell
    Input unsigned pre-cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def_static("sAdjacent", &TT::sAdjacent,
R"(Returns the cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].

Note: It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.

Parameters
----------
cell: SCell
    Input signed pre-cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    // ----------------------- Incidence services --------------------------

    py_class.def_static("uIncident", &TT::uIncident,
R"(Returns the cell which is incident to the input unsigned [cell] along axis [dim]
in the given orientation defined by [up].

Note: It may be a lower incident pre-cell if [cell] is open along axis [dim],
else an upper incident pre-cell.

Parameters
----------
cell: Cell
    Input unsigned pre-cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is incident to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def_static("sIncident", &TT::sIncident,
R"(Returns the cell which is incident to the input signed [cell] along axis [dim]
in the given orientation defined by [up].

Note: It may be a lower incident pre-cell if [cell] is open along axis [dim],
else an upper incident pre-cell.

Parameters
----------
cell: SCell
    Input signed pre-cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is incident to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def_static("uLowerIncident", &TT::uLowerIncident,
R"(Returns the cells directly low incident to [cell].

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The cells directly low incident to [cell].
)", py::arg("cell"));

    py_class.def_static("sLowerIncident", &TT::sLowerIncident,
R"(Returns the cells directly low incident to [cell].

Parameters
----------
cell: SCell
    Input signed pre-cell

Return
------
    The cells directly low incident to [cell].
)", py::arg("cell"));

    py_class.def_static("uUpperIncident", &TT::uUpperIncident,
R"(Returns the cells directly up incident to [cell].

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The cells directly up incident to [cell].
)", py::arg("cell"));

    py_class.def_static("sUpperIncident", &TT::sUpperIncident,
R"(Returns the cells directly up incident to [cell].

Parameters
----------
cell: SCell
    Input signed pre-cell

Return
------
    The cells directly up incident to [cell].
)", py::arg("cell"));

    py_class.def_static("uFaces", &TT::uFaces,
R"(Returns the proper faces of [cell] (chain of lower incidence).

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The proper faces of [cell] (chain of lower incidence).
)", py::arg("cell"));

    py_class.def_static("uCoFaces", &TT::uCoFaces,
R"(Returns the proper cofaces of [cell] (chain of lower incidence).

Parameters
----------
cell: Cell
    Input unsigned pre-cell

Return
------
    The proper cofaces of [cell] (chain of lower incidence).
)", py::arg("cell"));

    py_class.def_static("sDirect", &TT::sDirect,
R"(Returns True if the direct orientation of [cell] along [dim] is in the
positive coordinate direction.

The direct orientation in a direction allows to go from positive incident
pre-cells to positive incident pre-cells.
This means that:
```
KPreSpace::sSign( KPreSpace::sIncident( p, k, KPreSpace::sDirect( p, k ) ) ) == KPreSpace::POS
```
is always true.

Parameters
----------
cell: SCell
    Input signed pre-cell
dim: Integer
    Any coordinate

Return
------
    The direct orientation of [cell] along [dim] (True is upward, False is backward).
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sDirectIncident", &TT::sDirectIncident,
R"(Returns the direct incident pre-cell of [cell] along [dim] (the incident pre-cell along [dim]).

Parameters
----------
cell: SCell
    Input signed pre-cell
dim: Integer
    Any coordinate

Return
------
    The direct incident pre-cell of [cell] along [dim] (the incident pre-cell along [dim]).
)", py::arg("cell"), py::arg("dim"));

    py_class.def_static("sIndirectIncident", &TT::sIndirectIncident,
R"(Returns the indirect incident pre-cell of [cell] along [dim] (the incident
pre-cell along [dim] whose sign is negative).

Parameters
----------
cell: SCell
    Input signed pre-cell
dim: Integer
    Any coordinate

Return
------
    The indirect incident pre-cell of [cell] along [dim] (the incident pre-cell
    along [dim] whose sign is negative).
)", py::arg("cell"), py::arg("dim"));

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly_static("TCell",
            [](py::object /* self */) {
            return py::type::of<TTCell>();
            });
    py_class.def_property_readonly_static("TSCell",
            [](py::object /* self */) {
            return py::type::of<TTSCell>();
            });
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the KhalimskyPreSpace.)");
    py_class.def_property_readonly_static("DIM",
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
        os << typestr << ": ";
        self.selfDisplay(os);
        return os.str();
    });

    return py_class;
}
#endif
