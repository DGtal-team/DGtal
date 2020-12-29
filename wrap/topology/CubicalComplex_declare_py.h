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

#ifndef DGTAL_CUBICALCOMPLEX_DECLARE_PY_H
#define DGTAL_CUBICALCOMPLEX_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/CubicalComplex.h"
#include "DGtal/topology/ParDirCollapse.h"
#include "CubicalComplex_types_py.h"

template<typename TCellMap>
pybind11::class_<TCellMap> declare_CellMap(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TCellMap;
    auto py_class = py::bind_map<TT>(m, typestr.c_str());
    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        for (auto it = self.cbegin(); it != self.cend() ; ++it) {
            os << it->first << ": " << it->second.data << std::endl;
        }
        return os.str();
    });
    return py_class;
}


template<typename TCubicalComplex>
pybind11::class_<TCubicalComplex> declare_CubicalComplex(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TCubicalComplex;
    using TTKSpace = typename TT::KSpace;
    using TTCell = typename TT::Cell;
    using TTPreCell = typename TT::PreCell;
    using TTData = typename TT::Data;
    const std::string docs =
R"(This class represents an arbitrary cubical complex living in some
Khalimsky space.
Cubical complexes are sets of cells of different dimensions related together
with incidence relations.
Two cells in a cubical complex are incident if and only if they are
incident in the surrounding Khalimsky space. In other words,
cubical complexes are defined here as subsets of Khalimsky spaces.

CubicalComplex is close from being an AssociativeContainer, but values are not
sorted (they are sorted per dimension), and are not modifiable.

The CellContainer is chosen to be a std::unordered_map for python wrappings.

Example of usage:

import dgtal
CComplex = dgtal.topology.CubicalComplex3D
KSpace = CComplex.TKSpace
space = KSpace()
lower = space.TPoint.diagonal(0)
upper = space.TPoint.diagonal(5)
space.init(lower=lower, upper=upper, is_closed=True)
ccomplex = CComplex(space)
cell1 = space.uCell(KSpace.TPoint.diagonal(1))
ccomplex.insertCell(cell1)
cell2 = space.uCell(KSpace.TPoint.diagonal(2))
ccomplex.insert(cell2)
cells = ccomplex.getCells(dim=0)
print("cells: ", cells)
cell1_found = ccomplex.find(cell1)

faces = ccomplex.faces(cell1)
directFaces = ccomplex.directFaces(cell1)
isCellInterior = ccomplex.isCellInterior(cell1)
)";
    auto py_class = py::class_<TT>(m, typestr.c_str(), docs.c_str());
    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init<const TTKSpace &>());
    py_class.def(py::init<const TT &>());

    py_class.def("__copy__", [](const TT &self) {
        return TT(self);
    }, "Equivalent to __deepcopy__.");
    py_class.def("__deepcopy__", [](const TT &self, py::dict) {
        return TT(self);
    }, py::arg("memo"));

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("size", &TT::size);
    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.begin(), self.end()); },
         py::keep_alive<0, 1>() /* Keep object alive while iterator exists */);
    // py_class.def("__getitem__", [](const TT & self, const TTCell &cell) {
    //         return self.operator[](cell);
    //     });

    py_class.def("iter", [](const TT & self, size_t dim) {
        if(dim > TT::dimension) throw py::index_error();
        return py::make_iterator(self.begin(dim), self.end(dim)); },
         py::keep_alive<0, 1>() /* Keep object alive while iterator exists */);
    // ----------------------- Class operators --------------------------------

    // Arithmetic
    py_class.def(py::self | py::self, "Union");
    py_class.def(py::self |= py::self, "Union in-place");
    py_class.def(py::self & py::self, "Intersection");
    py_class.def(py::self &= py::self, "Intersection in-place");
    py_class.def(py::self - py::self, "Difference");
    py_class.def(py::self -= py::self, "Difference in-place");
    py_class.def(py::self ^ py::self, "Symmetric difference");
    py_class.def(py::self ^= py::self, "Symmetric difference in-place");
    py_class.def(~py::self, "Close");
    // Open not implemented: in python operator * is not unary
    // py_class.def(*py::self);//, "Open");

    // Comparisons
    py_class.def(py::self == py::self);
    py_class.def(py::self != py::self);
    py_class.def(py::self <= py::self);
    py_class.def(py::self >= py::self);

    // ----------------------- Class functions --------------------------------
    py_class.def("clear", py::detail::overload_cast_impl<>()(&TT::clear),
            "Clears the cubical complex, which becomes empty.");
    py_class.def("clear", py::detail::overload_cast_impl<DGtal::Dimension>()(&TT::clear),
R"(Clears all cell of the input dimension.
Parameters
----------
dim: Dimension
    Input dimension
)", py::arg("dim"));

    py_class.def("nbCells", &TT::nbCells,
R"(The number of cells of the input dimension in this complex.
Parameters
----------
dim: Dimension
    Input dimension
)", py::arg("dim"));

    py_class.def("euler", &TT::euler,
R"(The Euler number of this complex which equals nbCells( 0 ) - nbCells( 1 ) + nbCells( 2 ) - ...

Note: For instance, all Platonician solids have euler number
equal to one, while their surface have euler number equal to
two.
)");

    py_class.def("space", &TT::space,
R"(Returns a reference to the Khalimsky space associated to this complex.)");

    py_class.def("getCells", py::detail::overload_cast_impl<const DGtal::Dimension>()(&TT::getCells),
R"(The cell container associated to the cells of dimension dim
Parameters
----------
dim: Dimension
    Input dimension
)", py::arg("dim"));

    py_class.def("count", &TT::count,
R"(The number of matches for \a cell, which is thus zero (not present) or one (present).
Parameters
----------
cell: Cell
    Any cell
)", py::arg("cell"));
    py_class.def("max_size", &TT::max_size,
R"(The maximal number of cells in this complex (i.e., the number of cells of the Khalimsky space).
)");
    py_class.def("empty", &TT::empty,
R"('True' if and only if the complex does not hold any cell.)");
    py_class.def("erase", py::detail::overload_cast_impl<const TTCell &>()(&TT::erase),
R"(Erases input cell from the complex (STL version, see eraseCell).
Parameters
----------
cell: Cell
    Any cell
)", py::arg("cell"));

    py_class.def("insert", [](TT & self, const TTCell & cell) {
            auto insert_pair = self.insert(cell);
            return insert_pair.second;
            },
R"(Insert element cell into the complex.
Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.

Return
------
    True when the cell was indeed a new element.
    False when the cell already existed in the complex.

)",py::arg("cell"));

    py_class.def("insertCell", [](TT & self,
                const TTCell & cell, const TTData & data ) {
            self.insertCell(cell, data);
            },
R"(Insert element cell into the complex and assign it to the value data.
Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
data: CubicalCellData
    Any uint32_t value
)",py::arg("cell"), py::arg("data") = TTData());

    py_class.def("eraseCell", [](TT & self,
                const TTCell & cell) {
            self.eraseCell(cell);
            },
R"(Erases cell from the complex.
Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
Return
------
    The number of cells effectively removed from the complex.
)",py::arg("cell"));

    py_class.def("find", [](const TT & self, const TTCell & cell) -> py::object {
            typename TT::ConstIterator it = self.find(cell);
            if (it != self.end() ) {
                return py::cast(*it, py::return_value_policy::reference);
            } else {
                return py::none();
            }
            },
R"(Find input cell in the complex, returns None if not found, or the cell itself if found.
Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
Return
------
None if not found, or the cell itself.
)", py::arg("cell"));

    py_class.def("belongs", [](TT & self, const TTCell & cell) {
            return self.belongs(cell);
            },
R"(Returns True if cell (Cell) belongs to the complex.
Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
Return
------
    True if and only if [cell] belongs to this complex.
)", py::arg("cell"));
    py_class.def("belongs", [](TT & self, const TTPreCell & cell) {
            return self.belongs(cell);
            },
R"(Returns True if cell (PreCell) belongs to the complex.
Parameters
----------
cell: PreCell
    Any pre-cell valid in the Khalimsky space associated to the complex.
Return
------
    True if and only if [cell] belongs to this complex.
)", py::arg("cell"));

    py_class.def("faces", [](TT & self, const TTCell & cell, bool hintClosed) {
            typename TT::Cells out;
            std::back_insert_iterator< typename TT::Cells > outIt( out );
            self.faces(outIt, cell, hintClosed);
            return out;
            },
R"(Return all the cells that are proper faces of input cell.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintClosed") = false);

    py_class.def("directFaces", [](TT & self, const TTCell & cell, bool hintClosed) {
            typename TT::Cells out;
            std::back_insert_iterator< typename TT::Cells > outIt( out );
            self.directFaces(outIt, cell, hintClosed);
            return out;
            },
R"(Return all the cells that are direct faces of input cell.
Direct faces are lower incident cells with a dimension just one below.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintClosed") = false);

    py_class.def("coFaces", [](TT & self, const TTCell & cell, bool hintOpen) {
            typename TT::Cells out;
            std::back_insert_iterator< typename TT::Cells > outIt( out );
            self.coFaces(outIt, cell, hintOpen);
            return out;
            },
R"(Return all the cells that are co-faces of input cell.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintOpen: Bool [False]
    When 'true', this hint tells that the complex is open, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintOpen") = false);

    py_class.def("directCoFaces", [](TT & self, const TTCell & cell, bool hintOpen) {
            typename TT::Cells out;
            std::back_insert_iterator< typename TT::Cells > outIt( out );
            self.directCoFaces(outIt, cell, hintOpen);
            return out;
            },
R"(Return all the cells that are direct co-faces of input cell.
Direct co-faces are upper incident cells with a dimension just one above.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintOpen: Bool [False]
    When 'true', this hint tells that the complex is open, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintOpen") = false);

    // ---------- local operations for extracting specific subcomplexes -------------
    py_class.def("cellBoundary", &TT::cellBoundary,
R"(Returns the boundary of input cell as a cell collection,
i.e. all the cells that are proper faces of the cell. Generally
faster than calling the function faces.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintClosed") = false);

    py_class.def("cellCoBoundary", &TT::cellCoBoundary,
R"(Returns the boundary of input cell as a cell collection,
i.e. all the cells that are proper co-faces of the cell. Generally
faster than calling the function co-faces.

Note: all returned cells belong to this complex, while it is
not compulsory for input cell to belong to it.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
hintOpen: Bool [False]
    When 'true', this hint tells that the complex is open, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("cell"), py::arg("hintOpen") = false);

    // ---------------------- local properties --------------------------------------
    py_class.def("isCellInterior", &TT::isCellInterior,
R"(Returns true if and only if input cell is interior to the complex.
Which means that it has the same co-faces in the Khalimsky space as in this complex.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
)", py::arg("cell"));

    py_class.def("isCellBoundary", &TT::isCellBoundary,
R"(Returns true if and only if input cell is not interior to the complex.
Which means that it has the more co-faces in the Khalimsky space than in this complex.

Parameters
----------
cell: Cell
    Any cell valid in the Khalimsky space associated to the complex.
)", py::arg("cell"));

    // ----------------------- Standard subcomplexes --------------------------------
    py_class.def("interior", &TT::interior,
R"(Computes and returns the (topological) interior to this complex.)");

    py_class.def("boundary", &TT::boundary,
R"(Computes and returns the (topological) boundary of this complex (say X),
hence it may not be a subcomplex of X, but it is a subcomplex
of Cl(X).

Parameters
----------
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
)", py::arg("hintClosed") = false);

    py_class.def("closure", &TT::closure,
R"(Returns the closure of the cells in the input complex (S) within this complex,
i.e. the smallest subcomplex that contains each cell in S.

Parameters
----------
ccomplex: CubicalComplex
    S any complex the cells of which belong to this complex.
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
Return
------
The closure of input complex (S) within this complex as a cubical complex.
)", py::arg("ccomplex"), py::arg("hintClosed") = false);

    py_class.def("star", &TT::star,
R"(Returns the star of the cells in the input complex (S) within this complex,
i.e. the set of all cells of thiscomplex that have any faces in S.

Parameters
----------
ccomplex: CubicalComplex
    S any complex the cells of which belong to this complex.
hintOpen: Bool [False]
    When 'true', this hint tells that the complex is open, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
Return
------
The star of input complex (S) within this complex as a cubical complex.
)", py::arg("ccomplex"), py::arg("hintOpen") = false);

    py_class.def("link", &TT::link,
R"(Returns the link of the cells in the input complex (S) within this complex,
i.e. the closed star of S minus the stars of all faces of S.

Parameters
----------
ccomplex: CubicalComplex
    S any complex the cells of which belong to this complex.
hintClosed: Bool [False]
    When 'true', this hint tells that the complex is closed, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
hintOpen: Bool [False]
    When 'true', this hint tells that the complex is open, so this speeds up this method.
    Otherwise, the complex may be arbitrary.
    False by default
Return
------
The link of input complex (S) within this complex as a cubical complex.
)", py::arg("ccomplex"), py::arg("hintClosed") = false, py::arg("hintOpen") = false);

    // ----------------------- global operations on complexes -----------------------
    py_class.def("close", py::detail::overload_cast_impl<>()(&TT::close), R"(Close the whole complex.)");
    py_class.def("close", py::detail::overload_cast_impl<DGtal::Dimension>()(&TT::close),
R"(Close all cells of dimension less or equal to input dim.
Parameters
----------
dim: Dimension
    Input dimension
)", py::arg("dim"));

    py_class.def("open", py::detail::overload_cast_impl<>()(&TT::open), R"(open the whole complex.)");
    py_class.def("open", py::detail::overload_cast_impl<DGtal::Dimension>()(&TT::open),
R"(open all cells of dimension less or equal to input dim.
Parameters
----------
dim: Dimension
    Input dimension
)", py::arg("dim"));

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly_static("TKSpace",
            [](py::object /* self */) {
            return py::type::of<TTKSpace>();
            });
    py_class.def_property_readonly_static("TData",
            [](py::object /* self */) {
            return py::type::of<TTData>();
            });
    py_class.def_property_readonly_static("TCell",
            [](py::object /* self */) {
            return py::type::of<TTCell>();
            });
    py_class.def_property_readonly_static("TPreCell",
            [](py::object /* self */) {
            return py::type::of<TTPreCell>();
            });
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::dimension; },
            R"(The dimension of the complex.)");
    py_class.def_property_readonly_static("REMOVED",
            [](py::object /* self */) { return TT::REMOVED; },
            R"(Predefined flag for CubicalCellData.)");
    py_class.def_property_readonly_static("COLLAPSIBLE",
            [](py::object /* self */) { return TT::COLLAPSIBLE; },
            R"(Predefined flag for CubicalCellData.)");
    py_class.def_property_readonly_static("FIXED",
            [](py::object /* self */) { return TT::FIXED; },
            R"(Predefined flag for CubicalCellData.)");
    py_class.def_property_readonly_static("USER1",
            [](py::object /* self */) { return TT::USER1; },
            R"(Predefined flag for CubicalCellData.)");
    py_class.def_property_readonly_static("VALUE",
            [](py::object /* self */) { return TT::VALUE; },
            R"(Predefined flag for CubicalCellData.)");

    // ----------------- Python only functions  -------------------------------
    // ParDirCollapse
    py_class.def("thinning", [](TT &self, const std::string &method, const size_t iterations, bool verbose) {
        DGtal::ParDirCollapse < TT > thinning ( self.space() );
        thinning.verbose = verbose;
        thinning.attach(&self);
        if(method == "pardir") {
          if(iterations == 0) {
            while( thinning.eval(1) ) {};
          } else {
            thinning.eval(iterations);
          }
        } else if( method == "pardir_surface") {
            thinning.collapseSurface();
        } else if( method == "pardir_isthmus") {
            thinning.collapseIsthmus();
        } else {
            throw py::value_error("Not valid method ("
                    + method + "). Valid methods are: pardir, surface or isthmus.");
        }
    },
R"(Impements thinning algorithms in cubical complexes.

Based on article: Chaussard, J. and Couprie, M., Surface Thinning in 3D Cubical Complexes,
Combinatorial Image Analysis, (2009)

Methods:
pardir (ParDirCollapse) : Directional collapse of free pairs of faces. [Default]

pardir_surface: (CollapseSurface): Extension of ParDirCollapse such that faces
of dimension one lower than the dimension of the complex are preserved.

pardir_isthmus (CollapseIsthmus): Extension of ParDirCollapse such that faces
of dimension one lower than the dimension of the complex are preserved
when they do not contain free faces of dimension two lower than the
dimension of the complex.

The complex is thinned in-place.
Make a copy to the complex before applying this method if this is not desired:
import copy
thinned = copy.deepcopy(ccomplex)
thinned.thinning()

Parameters
----------
method: String ["pardir"]
    Options: pardir, pardir_surface, pardir_isthmus

iterations: Int [0]
    Number of iterations for the pardir method.
    The default 0 means the algorithm continues until full collapse (no more cells to remove).
    Only applies to "pardir" method, the other methods will always fully collapse.
verbose: Bool [False]
    State is reported during the run.
)",
    py::arg("method") = "pardir",
    py::arg("iterations") = 0,
    py::arg("verbose") = false
    );

    // ----------------------- Print / Display --------------------------------
    py_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    py_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << ": ";
        self.selfDisplay(os);
        return os.str();
    });

    return py_class;
}
#endif
