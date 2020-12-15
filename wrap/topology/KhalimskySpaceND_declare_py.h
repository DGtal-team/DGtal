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
pybind11::class_<TKhalimskyCell> declare_KhalimskyCell_common(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyCell;
    using TTInteger = typename TT::Integer;
    using TTPoint = typename TT::Point;
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

    return py_class;
}

template<typename TKhalimskyCell>
pybind11::class_<TKhalimskyCell> declare_KhalimskyCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyCell;
    using TTPoint = typename TT::Point;
    using TTInteger = typename TT::Integer;
    using TTPreCell = typename TT::PreCell;
    auto py_class = declare_KhalimskyCell_common<TT>(m, typestr);

    py_class.def_property_readonly_static("TPreCell",
            [](py::object /* self */) {
            return py::type::of<TTPreCell>();
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

template<typename TKhalimskyCell>
pybind11::class_<TKhalimskyCell> declare_SignedKhalimskyCell(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskyCell;
    using TTPoint = typename TT::Point;
    using TTSPreCell = typename TT::SPreCell;
    using TTInteger = typename TT::Integer;
    auto py_class = declare_KhalimskyCell_common<TT>(m, typestr);

    py_class.def_property_readonly_static("TSPreCell",
            [](py::object /* self */) {
            return py::type::of<TTSPreCell>();
            });
    // ----------------------- Print / Display --------------------------------

    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        auto py_coordinates = py::cast(self.preCell().coordinates);
        os << "preCell.coordinates: " << py_coordinates.attr("__str__")();
        os << "\n";
        os << "preCell.positive: " << self.preCell().positive;
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << "(";
        auto py_coordinates = py::cast(self.preCell().coordinates);
        os << py_coordinates.attr("__repr__")();
        os << ", sign=" << self.preCell().positive << ")";
        return os.str();
    });
    return py_class;
}

template<typename TKhalimskySpaceND>
pybind11::class_<TKhalimskySpaceND> declare_KhalimskySpaceND(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TKhalimskySpaceND;
    using TTInteger = typename TT::Integer;
    using TTPoint = typename TT::Point;
    using TTDimension = typename TTPoint::Dimension;
    using TTCell = typename TT::Cell;
    using TTPreCell = typename TT::PreCell;
    using TTSign = typename TT::Sign;
    using TTSCell = typename TT::SCell;
    using TTSPreCell = typename TT::SPreCell;

    auto py_class = py::class_<TT>(m, typestr.c_str());

    // Wrap Closure enum
    py::enum_<typename TT::Closure>(py_class, "Closure")
        .value("CLOSED", TT::CLOSED)
        .value("OPEN", TT::OPEN)
        .value("PERIODIC", TT::PERIODIC);

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());
    py_class.def("init", py::detail::overload_cast_impl<const TTPoint&,
            const TTPoint&, bool>()(&TT::init),
R"(Specifies the upper and lower bounds for the maximal cells in this space.

Parameters
----------
lower: Point
    The lowest point in this space (digital coordinates)
upper: Point
    The upper point in this space (digital coordinates)
isClosed: Bool
    True if this space is closed and non-periodic in every dimension, False if open.

Return
------
    True if the initialization was valid (ie, such bounds are representable with these integers).
)", py::arg("lower"), py::arg("upper"), py::arg("is_closed"));

    py_class.def("init", py::detail::overload_cast_impl<const TTPoint&,
            const TTPoint&, typename TT::Closure>()(&TT::init),
R"(Specifies the upper and lower bounds for the maximal cells in this space.

Parameters
----------
lower: Point
    The lowest point in this space (digital coordinates)
upper: Point
    The upper point in this space (digital coordinates)
closure: Closure
    CLOSED, OPEN or PERIODIC

Return
------
    True if the initialization was valid (ie, such bounds are representable with these integers).
)", py::arg("lower"), py::arg("upper"), py::arg("closure"));

    // ----------------------- Class functions --------------------------------
    // ----------------------- cell creation services --------------------------
    py_class.def("uCell", py::detail::overload_cast_impl<const TTPreCell&>()(&TT::uCell, py::const_),
R"(From an unsigned cell, returns an unsigned cell lying into this Khalismky space.

Along a non-periodic dimension, if the given Khalimsky coordinate lies
outside the space, it replaces it by the nearest valid coordinate.

Along a periodic dimension, the Khalimsky coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
cell: PreCell
    A pre-cell.

Return
------
    The same cell with appropriate Khalimsky coordinates along periodic dimensions.
)", py::arg("cell"));

    py_class.def("uCell", py::detail::overload_cast_impl<const TTPoint&>()(&TT::uCell, py::const_),
R"(From the Khalimsky coordinates of a cell, builds the corresponding unsigned cell.

Along a non-periodic dimension, if the given Khalimsky coordinate lies
outside the space, it is replaced by the nearest valid coordinate.

Along a periodic dimension, the Khalimsky coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
kpoint: Point
    Khalimsky coordinates of a cell.

Return
------
    Unsigned cell.
)", py::arg("kpoint"));

    py_class.def("uCell", py::detail::overload_cast_impl<TTPoint, const TTPreCell&>()(&TT::uCell, py::const_),
R"(From the digital coordinates of a point in Zn and a cell type, builds the corresponding unsigned cell.

Along a non-periodic dimension, if the given digital coordinate lies
outside the space, it is replaced by the nearest valid coordinate.

Along a periodic dimension, the digital coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
cell: PreCell
    Another cell defining the topology.

Return
------
    The cell having the topology of [cell] and the given digital coordinates [p].
)", py::arg("point"), py::arg("cell"));

    py_class.def("sCell", py::detail::overload_cast_impl<const TTSPreCell&>()(&TT::sCell, py::const_),
R"(From a signed cell, returns a signed cell lying into this Khalismky space.

Along a non-periodic dimension, if the given Khalimsky coordinate lies
outside the space, it replaces it by the nearest valid coordinate.

Along a periodic dimension, the Khalimsky coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
cell: SPreCell
    Signed pre-cell

Return
------
    The same signed cell with appropriate Khalimsky coordinates along periodic dimensions.
)", py::arg("cell"));

    py_class.def("sCell", py::detail::overload_cast_impl<const TTPoint&, TTSign>()(&TT::sCell, py::const_),
R"(From the Khalimsky coordinates of a cell and a sign, builds the corresponding signed cell.

Along a non-periodic dimension, if the given Khalimsky coordinate lies
outside the space, it is replaced by the nearest valid coordinate.

Along a periodic dimension, the Khalimsky coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
kpoint: Point
    Khalimsky coordinates of a cell.
sign: Bool
    The sign of the cell (either POS (True) or NEG (False)), defaults to True.

Return
------
    Signed cell.
)", py::arg("kpoint"), py::arg("sign")=true);

    py_class.def("sCell", py::detail::overload_cast_impl<TTPoint, const TTSPreCell&>()(&TT::sCell, py::const_),
R"(From the digital coordinates of a point in Zn and a signed cell type, builds the corresponding signed cell.

Along a non-periodic dimension, if the given Khalimsky coordinate lies
outside the space, it is replaced by the nearest valid coordinate.

Along a periodic dimension, the Khalimsky coordinate is corrected
(by periodicity) to lie between the coordinates of lowerCell() and upperCell().

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
cell: SPreCell
    Another cell defining the topology and sign.

Return
------
    The cell having the topology and sign of [cell] and the given digital coordinates [p].
)", py::arg("point"), py::arg("cell"));

    py_class.def("uSpel", &TT::uSpel,
R"(From the digital coordinates of a point in Zn, builds the corresponding -spel (-cell of maximal dimension).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).

Return
------
    The -spel having the given digital coordinates [p].
)", py::arg("point"));

    py_class.def("sSpel", &TT::sSpel,
R"(From the digital coordinates of a point in Zn and a cell sign, builds the corresponding signed -spel (-cell of maximal dimension).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
sign: Bool
    The sign of the cell (either POS (True) or NEG (False), defaults to True.

Return
------
    The signed -spel having the given digital coordinates [p] and [sign].
)", py::arg("point"), py::arg("sign")=true);

    py_class.def("uPointel", &TT::uPointel,
R"(From the digital coordinates of a point in Zn, builds the corresponding -pointel (-cell of dimension 0).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).

Return
------
    The -pointel having the given digital coordinates [p].
)", py::arg("point"));

    py_class.def("sPointel", &TT::sPointel,
R"(From the digital coordinates of a point in Zn and a cell sign, builds the corresponding signed -pointel (-cell dimension 0).

Parameters
----------
p: Point
    An integer point (digital coordinates of cell).
sign: Bool
    The sign of the cell (either POS (True) or NEG (False), defaults to True.

Return
------
    The signed -pointel having the given digital coordinates [p] and [sign].
)", py::arg("point"), py::arg("sign")=true);

    // ----------------------- Read accessors to cells ------------------------

    py_class.def("uKCoord", &TT::uKCoord,
R"(Return its Khalimsky coordinate along [dim].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    Any valid dimension.

Return
------
    The Khalimsky coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def("uCoord", &TT::uCoord,
R"(Return its digital coordinate along [dim].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    Any valid dimension.

Return
------
    The digital coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def("uKCoords", &TT::uKCoords,
R"(Return its Khalimsky coordinates.

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    The Khalimsky coordinates of input [cell].
)", py::arg("cell"));

    py_class.def("uCoords", &TT::uCoords,
R"(Return its digital coordinates.

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    The digital coordinates of input [cell].
)", py::arg("cell"));

    py_class.def("sKCoord", &TT::sKCoord,
R"(Return its Khalimsky coordinate along [dim].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    Any valid dimension.

Return
------
    The Khalimsky coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sCoord", &TT::sCoord,
R"(Return its digital coordinate along [dim].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    Any valid dimension.

Return
------
    The digital coordinates along the dimension [dim] of input [cell].
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sKCoords", &TT::sKCoords,
R"(Return its Khalimsky coordinates.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The Khalimsky coordinates of input [cell].
)", py::arg("cell"));

    py_class.def("sCoords", &TT::sCoords,
R"(Return its digital coordinates.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The digital coordinates of input [cell].
)", py::arg("cell"));

    py_class.def("sSign", &TT::sSign,
R"( Return its sign.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The sign of input [cell].
)", py::arg("cell"));
    // ----------------------- Closure helpers ------------------------
    py_class.def("isSpaceClosed", py::detail::overload_cast_impl<>()(&TT::isSpaceClosed, py::const_),
R"(Return true if the space is closed or periodic for all dimensions.
)");
    py_class.def("isSpaceClosed", py::detail::overload_cast_impl<DGtal::Dimension>()(&TT::isSpaceClosed, py::const_),
R"(Return true if the space is closed or periodic along the specified dimension.

Parameters
----------
dim: Dimension
    Any valid dimension
)", py::arg("dim"));

    py_class.def("isSpacePeriodic", py::detail::overload_cast_impl<>()(&TT::isSpacePeriodic, py::const_),
R"(Return true if the space is periodic for all dimensions.
)");
    py_class.def("isSpacePeriodic", py::detail::overload_cast_impl<DGtal::Dimension>()(&TT::isSpacePeriodic, py::const_),
R"(Return true if the space is periodic along the specified dimension.

Parameters
----------
dim: Dimension
    Any valid dimension
)", py::arg("dim"));

    py_class.def("isAnyDimensionPeriodic", &TT::isAnyDimensionPeriodic,
R"( Return true if any dimension is periodic

Return
------
    True if at least one dimension is periodic.
)");

    py_class.def("getClosure", &TT::getClosure,
R"(Returns the closure type along the specified dimension.

Parameters
----------
dim: Dimension
    Any valid dimension
)", py::arg("dim"));

    // ----------------------- Write accessors to cells ------------------------

    py_class.def("uSetKCoord", &TT::uSetKCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def("sSetKCoord", &TT::sSetKCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def("uSetCoord", &TT::uSetCoord,
R"(Sets the [dim]-th digital coordinate of [cell] to [i].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def("sSetCoord", &TT::sSetCoord,
R"(Sets the [dim]-th Khalimsky coordinate of [cell] to [i].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    Any valid dimension.
i: Int
    An integer coordinate.
)", py::arg("cell"), py::arg("dim"), py::arg("value"));

    py_class.def("uSetKCoords", &TT::uSetKCoords,
R"(Sets the Khalimsky coordinates of [cell] to [kp]

Parameters
----------
cell: Cell
    Any unsigned cell
kp: Point
    The new Khalimsky coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def("sSetKCoords", &TT::sSetKCoords,
R"(Sets the Khalimsky coordinates of [cell] to [kp]

Parameters
----------
cell: SCell
    Any signed cell
kp: Point
    The new Khalimsky coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def("uSetCoords", &TT::uSetCoords,
R"(Sets the digital coordinates of [cell] to [kp]

Parameters
----------
cell: Cell
    Any unsigned cell
kp: Point
    The new digital coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def("sSetCoords", &TT::sSetCoords,
R"(Sets the digital coordinates of [cell] to [kp]

Parameters
----------
cell: SCell
    Any signed cell
kp: Point
    The new digital coordinates for [cell].
)", py::arg("cell"), py::arg("value"));

    py_class.def("sSetSign", &TT::sSetSign,
R"(Sets the sign of the cell

Parameters
----------
cell: SCell
    Any signed cell
s: Bool
    Any sign (positive or negative)
)", py::arg("cell"), py::arg("sign"));

    // -------------------- Conversion signed/unsigned ------------------------
    py_class.def("signs", &TT::signs,
R"(Creates a signed cell from an unsigned one and a given sign.

Parameters
----------
cell: Cell
    Any unsigned cell
s: Bool
    Any sign (positive or negative)

Return
------
    The signed version of the input cell with input sign.
)", py::arg("cell"), py::arg("sign"));

    py_class.def("unsigns", &TT::unsigns,
R"(Creates an unsigned cell from an signed one.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The unsigned version of the input signed cell.
)", py::arg("cell"));

    py_class.def("sOpp", &TT::sOpp,
R"(Creates an signed cell with the inverse sign of input cell.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The cell with opposite sign than the input cell.
)", py::arg("cell"));

    // ------------------------- cell topology services -----------------------
    py_class.def("uTopology", &TT::uTopology,
R"(Returns the topology word of the input cell.

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    The topology word of [cell]
)", py::arg("cell"));

    py_class.def("sTopology", &TT::sTopology,
R"(Returns the topology word of the input cell.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The topology word of [cell]
)", py::arg("cell"));

    py_class.def("uDim", &TT::uDim,
R"(Returns the dimension of the input cell.
If the cell lives in 3D, the resulting uDim(cell):
0 -> Pointel
1 -> Linel
2 -> Surfel
3 -> Voxel

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    The dim word of [cell]
)", py::arg("cell"));

    py_class.def("sDim", &TT::sDim,
R"(Returns the dimension of the input cell.
If the cell lives in 3D, the resulting sDim(cell):
0 -> Pointel
1 -> Linel
2 -> Surfel
3 -> Voxel

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    The dimension of [cell]
)", py::arg("cell"));

    py_class.def("uIsSurfel", &TT::uIsSurfel,
R"(Returns true if input cell is a surfel (spans all but one coordinate).

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    True if input cell is a surfel (spans all but one coordinate).
)", py::arg("cell"));


    py_class.def("sIsSurfel", &TT::sIsSurfel,
R"(Returns true if input cell is a surfel (spans all but one coordinate).

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    True if input cell is a surfel (spans all but one coordinate).
)", py::arg("cell"));

    py_class.def("uIsOpen", &TT::uIsOpen,
R"(Returns true if input cell is open along the direction [dim].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    Any direction

Return
------
    True if input cell is open along the direction [dim].
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sIsOpen", &TT::sIsOpen,
R"(Returns true if input cell is open along the direction [dim].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    Any direction

Return
------
    True if input cell is open along the direction [dim].
)", py::arg("cell"), py::arg("dim"));

    // -------------- Iterator services for cells (not wrapped) -----------
    // -------------------- Unsigned cell geometry services --------------------

    py_class.def("uGetIncr", &TT::uGetIncr,
R"(Return the same element as [cell] except for the incremented coordinate [dim].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The changed coordinate

Return
------
    Cell with coordinate [dim] incremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def("uIsMax", &TT::uIsMax,
R"(Check if input cell is out of the space.

Note: It returns always False for periodic dimension.

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The tested coordinate

Return
------
    True if [cell] cannot have its [dim]-coordinate augmented without leaving the space.
)", py::arg("cell"), py::arg("dim"));

    const std::string uIsInside_with_dim_docs =
R"(Check if the coordinate [dim] of input [cell] is inside the space.

Note: Always True for periodic dimension.

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The tested coordinate

Return
------
    True if [cell] has its coordinates within the allowed bounds.
)";

    py_class.def("uIsInside", py::detail::overload_cast_impl<const TTPreCell&, TTDimension>()(&TT::uIsInside, py::const_),
            uIsInside_with_dim_docs.c_str(),
            py::arg("cell"), py::arg("dim"));
    py_class.def("uIsInside",
            [](const TT & self, const TTCell & cell, TTDimension dim) {
             return self.uIsInside(cell, dim);
            },
        uIsInside_with_dim_docs.c_str(),
        py::arg("cell"), py::arg("dim"));

    const std::string uIsInside_docs =
R"(Check if input cell is inside the space.

Note: Only the non-periodic dimensions are checked.

Parameters
----------
cell: Cell
    Any unsigned cell

Return
------
    True if [cell] has its coordinates within the allowed bounds.
)";
    py_class.def("uIsInside", py::detail::overload_cast_impl<const TTPreCell&>()(&TT::uIsInside, py::const_),
            uIsInside_docs.c_str(),
            py::arg("cell"));
    py_class.def("uIsInside",
            [](const TT & self, const TTCell & cell) {
             return self.uIsInside(cell);
            },
        uIsInside_docs.c_str(),
        py::arg("cell"));

    py_class.def("cIsInside", py::detail::overload_cast_impl<const TTPoint&, TTDimension>()(&TT::cIsInside, py::const_),
R"(Check if the coordinate [dim] of input [cell] is inside the space.

Note: Only the non-periodic dimensions are checked.

Parameters
----------
point: Point
    Point with khalimsky coordinates
dim: Int
    The tested coordinate

Return
------
    True if coordinates are within the allowed bounds.
)", py::arg("point"), py::arg("dim"));

    py_class.def("cIsInside", py::detail::overload_cast_impl<const TTPoint&>()(&TT::cIsInside, py::const_),
R"(Check if input cell is inside the space.

Note: Only the non-periodic dimensions are checked.

Parameters
----------
point: Point
    Point with khalimsky coordinates

Return
------
    True if coordinates are within the allowed bounds.
)", py::arg("point"));

    py_class.def("uGetDecr", &TT::uGetDecr,
R"(Return the same element as [cell] except for the decremented coordinate [dim].

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The changed coordinate

Return
------
    Cell with coordinate [dim] decremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def("uIsMin", &TT::uIsMin,
R"(Check if input cell is out of the space.

Note: It returns always False for periodic dimension.

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The tested coordinate

Return
------
    True if [cell] cannot have its [dim]-coordinate decremented without leaving the space.
)", py::arg("cell"), py::arg("dim"));

    py_class.def("uGetAdd", &TT::uGetAdd,
R"(Return the same element as [cell] except for the coordinate [dim] incremented by x.

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The changed coordinate
x: Int
    The increment

Return
------
    Cell with coordinate [dim] incremented with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def("uGetSub", &TT::uGetSub,
R"(Return the same element as [cell] except for the coordinate [dim] decremeneted by x.

Parameters
----------
cell: Cell
    Any unsigned cell
dim: Int
    The changed coordinate
x: Int
    The decrement

Return
------
    Cell with coordinate [dim] decremeneted with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def("uTranslation", &TT::uTranslation,
R"(Add the vector [vec] to the input [cell].

Parameters
----------
cell: Cell
    Any unsigned cell
vec: Point
    A vector

Return
------
    The unsigned cell resulting from the [cell] translated by [vec]
)", py::arg("cell"), py::arg("vec"));

    py_class.def("uProjection", &TT::uProjection,
R"(The projection of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProjection(p, bound, k))`.

Parameters
----------
cell: Cell
    Any unsigned cell
bound: Cell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

Return
------
    The projection of [cell] along the [dim]th direction toward [bound].

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def("uProject", &TT::uProject,
R"(Modifies input cell. Projects of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProject(p, bound, k))`.

Parameters
----------
cell: Cell
    Any unsigned cell
bound: Cell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def("uNext", &TT::uNext,
R"(Increment the [cell] to its next position (as classically done in a scanning)

: `uTopology(p) == uTopology(lower) == uTopology(upper)`.

Parameters
----------
cell: Cell
    Any unsigned cell
lower: Cell
    The lower bound.
upper: Cell
    The upper bound.

Return
------
    True if [cell] is still withing the bounds, false if the scanning is finished.

)", py::arg("cell"), py::arg("lower"), py::arg("upper"));

    // -------------------- Signed cell geometry services --------------------

    py_class.def("sGetIncr", &TT::sGetIncr,
R"(Return the same element as [cell] except for the incremented coordinate [dim].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The changed coordinate

Return
------
    SCell with coordinate [dim] incremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sIsMax", &TT::sIsMax,
R"(Check if input cell is out of the space.

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The tested coordinate

Return
------
    Always False for KhalimskySpace
)", py::arg("cell"), py::arg("dim"));


    const std::string sIsInside_with_dim_docs =
R"(Check if the coordinate [dim] of input [cell] is inside the space.

Note: Always True for periodic dimension.

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The tested coordinate

Return
------
    True if [cell] has its coordinates within the allowed bounds.
)";

    py_class.def("sIsInside", py::detail::overload_cast_impl<const TTSPreCell&, TTDimension>()(&TT::sIsInside, py::const_),
            sIsInside_with_dim_docs.c_str(),
            py::arg("cell"), py::arg("dim"));
    py_class.def("sIsInside",
            [](const TT & self, const TTSCell & cell, TTDimension dim) {
             return self.sIsInside(cell, dim);
            },
        sIsInside_with_dim_docs.c_str(),
        py::arg("cell"), py::arg("dim"));

    const std::string sIsInside_docs =
R"(Check if input cell is inside the space.

Note: Only the non-periodic dimensions are checked.

Parameters
----------
cell: SCell
    Any signed cell

Return
------
    True if [cell] has its coordinates within the allowed bounds.
)";
    py_class.def("sIsInside", py::detail::overload_cast_impl<const TTSPreCell&>()(&TT::sIsInside, py::const_),
            sIsInside_docs.c_str(),
            py::arg("cell"));
    py_class.def("sIsInside",
            [](const TT & self, const TTSCell & cell) {
             return self.sIsInside(cell);
            },
        sIsInside_docs.c_str(),
        py::arg("cell"));

    py_class.def("sGetDecr", &TT::sGetDecr,
R"(Return the same element as [cell] except for the decremented coordinate [dim].

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The changed coordinate

Return
------
    SCell with coordinate [dim] decremeneted.
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sIsMin", &TT::sIsMin,
R"(Check if input cell is out of the space.

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The tested coordinate

Return
------
    Always False for KhalimskySpace
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sGetAdd", &TT::sGetAdd,
R"(Return the same element as [cell] except for the coordinate [dim] incremented by x.

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The changed coordinate
x: Int
    The increment

Return
------
    SCell with coordinate [dim] incremented with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def("sGetSub", &TT::sGetSub,
R"(Return the same element as [cell] except for the coordinate [dim] decremeneted by x.

Parameters
----------
cell: SCell
    Any signed cell
dim: Int
    The changed coordinate
x: Int
    The decrement

Return
------
    SCell with coordinate [dim] decremeneted with x
)", py::arg("cell"), py::arg("dim"), py::arg("x"));

    py_class.def("sTranslation", &TT::sTranslation,
R"(Add the vector [vec] to the input [cell].

Parameters
----------
cell: SCell
    Any signed cell
vec: Point
    A vector

Return
------
    The signed cell resulting from the [cell] translated by [vec]
)", py::arg("cell"), py::arg("vec"));

    py_class.def("sProjection", &TT::sProjection,
R"(The projection of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProjection(p, bound, k))`.

Parameters
----------
cell: SCell
    Any signed cell
bound: SCell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

Return
------
    The projection of [cell] along the [dim]th direction toward [bound].

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def("sProject", &TT::sProject,
R"(Modifies input cell. Projects of [cell] along the [dim]th direction toward [bound].

Otherwise said, p[ k ] == bound[ k ] afterwards.

:  `uIsOpen(p, k) == uIsOpen(bound, k)`
Post: `uTopology(p) == uTopology(uProject(p, bound, k))`.

Parameters
----------
cell: SCell
    Any signed cell
bound: SCell
    The cell acting as bound (same topology as [cell])
dim: Int
    The concerned coordinate

)", py::arg("cell"), py::arg("bound"), py::arg("dim"));

    py_class.def("sNext", &TT::sNext,
R"(Increment the [cell] to its next position (as classically done in a scanning)

: `uTopology(p) == uTopology(lower) == uTopology(upper)`.

Parameters
----------
cell: SCell
    Any signed cell
lower: SCell
    The lower bound.
upper: SCell
    The upper bound.

Return
------
    True if [cell] is still withing the bounds, false if the scanning is finished.

)", py::arg("cell"), py::arg("lower"), py::arg("upper"));


    // ----------------------- Neighborhood services --------------------------

    py_class.def("uNeighborhood", &TT::uNeighborhood,
R"(Computes the 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell].

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The cells of 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def("sNeighborhood", &TT::sNeighborhood,
R"(Computes the 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell].

Parameters
----------
cell: SCell
    Input signed cell

Return
------
    The cells of 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def("uProperNeighborhood", &TT::uProperNeighborhood,
R"(Computes the proper 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell] and different from [cell].

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The cells of the proper 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def("sProperNeighborhood", &TT::sProperNeighborhood,
R"(Computes the proper 1-neighborhood of the [cell].

1-neighborhood is the set of cells with same topology that are adjacent to [cell] and different from [cell].

Parameters
----------
cell: SCell
    Input signed cell

Return
------
    The cells of the proper 1-neighborhood of [cell]
)", py::arg("cell"));

    py_class.def("uAdjacent", &TT::uAdjacent,
R"(Returns the cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].

Note: It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.

Parameters
----------
cell: Cell
    Input unsigned cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def("sAdjacent", &TT::sAdjacent,
R"(Returns the cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].

Note: It is an alias to 'up ? uGetIncr( p, k ) : uGetDecr( p, k )'.

Parameters
----------
cell: SCell
    Input signed cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is adjacent to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    // ----------------------- Incidence services --------------------------

    py_class.def("uIncident", &TT::uIncident,
R"(Returns the cell which is incident to the input unsigned [cell] along axis [dim]
in the given orientation defined by [up].

Note: It may be a lower incident cell if [cell] is open along axis [dim],
else an upper incident cell.

Parameters
----------
cell: Cell
    Input unsigned cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is incident to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def("sIncident", &TT::sIncident,
R"(Returns the cell which is incident to the input signed [cell] along axis [dim]
in the given orientation defined by [up].

Note: It may be a lower incident cell if [cell] is open along axis [dim],
else an upper incident cell.

Parameters
----------
cell: SCell
    Input signed cell
dim: Integer
    The coordinate that is changed
up: Bool
    Orientation: Forward (True) or backward (False)

Return
------
    The cell which is incident to the input [cell] along axis [dim] in the given orientation defined by [up].
)", py::arg("cell"), py::arg("dim"), py::arg("up"));

    py_class.def("uLowerIncident", &TT::uLowerIncident,
R"(Returns the cells directly low incident to [cell].

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The cells directly low incident to [cell].
)", py::arg("cell"));

    py_class.def("sLowerIncident", &TT::sLowerIncident,
R"(Returns the cells directly low incident to [cell].

Parameters
----------
cell: SCell
    Input signed cell

Return
------
    The cells directly low incident to [cell].
)", py::arg("cell"));

    py_class.def("uUpperIncident", &TT::uUpperIncident,
R"(Returns the cells directly up incident to [cell].

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The cells directly up incident to [cell].
)", py::arg("cell"));

    py_class.def("sUpperIncident", &TT::sUpperIncident,
R"(Returns the cells directly up incident to [cell].

Parameters
----------
cell: SCell
    Input signed cell

Return
------
    The cells directly up incident to [cell].
)", py::arg("cell"));

    py_class.def("uFaces", &TT::uFaces,
R"(Returns the proper faces of [cell] (chain of lower incidence).

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The proper faces of [cell] (chain of lower incidence).
)", py::arg("cell"));

    py_class.def("uCoFaces", &TT::uCoFaces,
R"(Returns the proper cofaces of [cell] (chain of lower incidence).

Parameters
----------
cell: Cell
    Input unsigned cell

Return
------
    The proper cofaces of [cell] (chain of lower incidence).
)", py::arg("cell"));

    py_class.def("sDirect", &TT::sDirect,
R"(Returns True if the direct orientation of [cell] along [dim] is in the
positive coordinate direction.

The direct orientation in a direction allows to go from positive incident
-cells to positive incident cells.
This means that:
```
KSpace::sSign( KSpace::sIncident( p, k, KSpace::sDirect( p, k ) ) ) == KSpace::POS
```
is always true.

Parameters
----------
cell: SCell
    Input signed cell
dim: Integer
    Any coordinate

Return
------
    The direct orientation of [cell] along [dim] (True is upward, False is backward).
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sDirectIncident", &TT::sDirectIncident,
R"(Returns the direct incident cell of [cell] along [dim] (the incident cell along [dim]).

Parameters
----------
cell: SCell
    Input signed cell
dim: Integer
    Any coordinate

Return
------
    The direct incident cell of [cell] along [dim] (the incident cell along [dim]).
)", py::arg("cell"), py::arg("dim"));

    py_class.def("sIndirectIncident", &TT::sIndirectIncident,
R"(Returns the indirect incident cell of [cell] along [dim] (the incident
-cell along [dim] whose sign is negative).

Parameters
----------
cell: SCell
    Input signed cell
dim: Integer
    Any coordinate

Return
------
    The indirect incident cell of [cell] along [dim] (the incident cell
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
            R"(The dimension of the KhalimskySpace.)");
    py_class.def_property_readonly_static("DIM",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the KhalimskySpace.)");
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
