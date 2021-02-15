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

#ifndef DGTAL_VOXELCOMPLEX_DECLARE_PY_H
#define DGTAL_VOXELCOMPLEX_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/VoxelComplex.h"
#include "DGtal/topology/VoxelComplexThinning.h"
#include "VoxelComplex_types_py.h"
#include "kernel/DigitalSetBySTLVector_types_py.h" // For DigitalSetZ3i,Z2i

template<typename TVoxelComplex>
pybind11::class_<TVoxelComplex> declare_VoxelComplex(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TVoxelComplex;
    using TTCubicalComplex = typename TT::Parent;
    using TTKSpace = typename TT::KSpace;
    using TTPoint = typename TTKSpace::Point;
    using TTCell = typename TT::Cell;
    using TTPreCell = typename TT::PreCell;
    using TTData = typename TT::Data;

    const std::string docs =
R"(Represents a voxel complex living in some Khalimsky space.
Voxel complexes are derived from cubical complexes, with specialized methods to deal with spels.

The aim is to implement critical kernels, ie, cliques of spels,
as shown by M.Couprie and G.Bertrand:

 Asymmetric parallel 3D thinning scheme and algorithms based on isthmuses.
 https://doi.org/10.1016/j.patrec.2015.03.014

Implemented using resources from CubicalComplex and
(Digital) Object for simplicity check in voxels.

CellContainer is a std::unordered_map.
)";
    auto py_class = py::class_<TT, TTCubicalComplex>(m, typestr.c_str(), docs.c_str());
    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init<const TTKSpace &>());
    py_class.def(py::init<const TT &>());

    py_class.def("__copy__", [](const TT &self) {
        return TT(self);
    }, "Equivalent to __deepcopy__.");
    py_class.def("__deepcopy__", [](const TT &self, py::dict) {
        return TT(self);
    }, py::arg("memo"));

    py_class.def("setSimplicityTable", [](TT & self, const std::string & tables_folder) {
            const std::string table_filename = "simplicity_table26_6";
            const std::string table_suffix = ".zlib";
            const std::string full_path =
                tables_folder + "/" + table_filename +  table_suffix;
            const auto compressed = true;
            auto table_smart_ptr = DGtal::functions::loadTable<TTKSpace::dimension>(full_path, compressed);
            self.setSimplicityTable(table_smart_ptr);
            return full_path;
        },
R"(Load a pre-computed simplicity_table to speed up future isSimple computations in this Object.

Only a string with tables_folder needs to be provided. The right filename: "simplicity_table26_6.zlib" will be selected automatically.

These LUT tables are shipped along the python dgtal package in the folder: dgtal.tables_folder.

Parameters
----------
tables_folder: String
    Parent folder to locate the tables. Usually: tables_folder=dgtal.tables_folder

Return
------
    A string with the full path of the loaded table file.
)", py::arg("tables_folder"));

    py_class.def("copySimplicityTable", &TT::copySimplicityTable,
            "Copy table from other complex.");

    py_class.def("isTableLoaded", &TT::isTableLoaded,
            "True if table would be used for isSimple computations.");

    py_class.def("voxelClose", &TT::voxelClose,
R"(Close input voxel
Parameters
----------
cell: Cell
    Input voxel
)", py::arg("cell"));

    py_class.def("cellsClose", &TT::cellsClose,
R"(Close all the input cells of input dimension
Parameters
----------
dim: Dimension
    Close cells of input dimension
cells: CellContainer
    List of cells
)",py::arg("dim"), py::arg("cells"));

    py_class.def("insertVoxelCell",
        py::detail::overload_cast_impl<const TTCell &, const bool &, const TTData &>()
        (&TT::insertVoxelCell),
R"(Insert cell (voxel) in the Khalimsky space.
Parameters
----------
cell: Cell
    Input voxel
close_it: Bool
    If True, apply voxelClose in the input cell
data: CubicalCellData
    CubicalCellData associated to the input cell
)",py::arg("cell"), py::arg("close_it") = true, py::arg("data") = TTData());

    py_class.def("insertVoxelPoint", &TT::insertVoxelPoint,
R"(Create a uSpel from the input Point and insert it using inserVoxelCell.
Parameters
----------
point: Point
    Input point of the KSpace
close_it: Bool
    If True, apply voxelClose in the input cell
data: CubicalCellData
    CubicalCellData associated to the input cell
)",py::arg("point"), py::arg("close_it"), py::arg("data"));


    py_class.def("dumpVoxels", [](const TT &self, DGtal::Python::DigitalSetZ3i & in_out_set) {
            self.dumpVoxels(in_out_set);
            return in_out_set;
        },
R"(Dump the voxels (kcell with dimension 3) into the input container.
Parameters
----------
digital_set: DigitalSetZ3i
    Set to dump the voxels into
Return
------
The input/output set filled with the voxels.
)",py::arg("digital_set"));

    // --------------------- Spels --------------------------------------------
    py_class.def("pointelsFromCell", [](const TT & self, const TTCell & input_cell) {
            std::set<TTCell> pointels_out;
            self.pointelsFromCell(pointels_out, input_cell);
            return pointels_out;
        },
R"(Get pointels that are Faces of input cell.
Note that if input cell is a spel, then it will be emplaced only if it
belongs to the complex.
See KSpace3D.uFaces
Parameters
----------
cell: Cell
    Input_cell input cell from which get the surrounding pointels.
Return
------
A list of pointels
)", py::arg("cell"));

    py_class.def("spelsFromCell", [](const TT & self, const TTCell & input_cell) {
            std::set<TTCell> spels_out;
            self.spelsFromCell(spels_out, input_cell);
            return spels_out;
        },
R"(Get spels that are coFaces of input cell.
Note that if input cell is a spel, then it will be emplaced only if it
belongs to the complex.
See KSpace3D.uCoFaces
Parameters
----------
cell: Cell
    Input_cell input cell from which get the surrounding spels.
Return
------
A list of spels
)", py::arg("cell"));

    py_class.def("neighborhoodVoxels", &TT::neighborhoodVoxels,
R"(Return the neighbor spels of input cell
Parameters
----------
cell: Cell
    Input cell that is the center of the neighborhood.
Return
------
A list of spels
)", py::arg("cell"));

    py_class.def("properNeighborhoodVoxels", &TT::properNeighborhoodVoxels,
R"(Return the set of voxels that are the properNeighborhood of the input cell
Parameters
----------
cell: Cell
    Input cell that is the center of the properNeighborhood.
Return
------
A list of spels
)", py::arg("cell"));

    py_class.def("Kneighborhood", &TT::Kneighborhood,
R"(Get a clique holding the K-neighborhood of the input cell.
The K-neighborhood is calculated first, getting the pointels
from input cell (See pointelsFromCell) and then, getting all
the spels from all those pointels (See spelsFromCell).

Parameters
----------
cell: Cell
    Input cell from which get the surrounding spels.
Return
------
Clique with the the cells forming the K-Neighborhood of the input cell.
)", py::arg("cell"));

    py_class.def("isSpel", &TT::isSpel,
R"(True if input cell is a cell with max dimension (3)
)", py::arg("cell"));

    py_class.def("surfelBetweenAdjacentSpels", &TT::surfelBetweenAdjacentSpels,
R"(Surfel between between two adjacent spels.

Parameters
----------
spel_A: Cell
    Input spel (cell of dimension 3)
spel_B: Cell
    Input spel (cell of dimension 3)
Return
------
The surfel (cell with dimension 2) between two input spels.
)", py::arg("spel_A"), py::arg("spel_B"));
    // ---------------------- Critical Cliques --------------------------------
    py_class.def("criticalCliques", [](const TT & self, bool verbose) {
        return self.criticalCliques(verbose);
     },
R"(Returns all critical cliques for this complex.

Parameters
----------
verbose: Bool
    Verbose output when running
Return
------
All critical cliques arranged by dimension:
[[clique_dim0, ...], [clique_dim1, ...], [clique_dim2, ...], [clique_dim_3,...]]
)", py::arg("verbose") = false);

    py_class.def("criticalCliquesForD",
            [](const TT & self, const DGtal::Dimension dim, bool verbose) {
        return self.criticalCliquesForD(dim, self, verbose);
     },
R"(Returns all critical cliques for this complex for the specified dimension.

Parameters
----------
dim: Int
    Dimension of the output critical cliques.
verbose: Bool
    Verbose output when running
Return
------
All critical cliques arranged for the specified dimension.
)", py::arg("dim"), py::arg("verbose") = false);

    // ---------------------- Masks -------------------------------------------
    py_class.def("K_2",
        py::detail::overload_cast_impl<const TTCell &, const TTCell &, bool>()
        (&TT::K_2, py::const_),
R"(Compute the criticality of the surfel between A,B voxels and returns the
associated 2-clique.

Parameters
----------
spel_A: Cell
    Input spel (cell of dimension 3)
spel_B: Cell
    Input spel (cell of dimension 3)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 2-clique]
)", py::arg("spel_A"), py::arg("spel_B"), py::arg("verbose") = false);

    py_class.def("K_2",
        py::detail::overload_cast_impl<const TTPoint &, const TTPoint &, bool>()
        (&TT::K_2, py::const_),
R"(Compute the criticality of the surfel between points A, B (uCoords) and returns the
associated 2-clique.

Parameters
----------
point_A: Point
    Input point (uCoords)
point_B: Point
    Input point (uCoords)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 2-clique]
)", py::arg("point_A"), py::arg("point_B"), py::arg("verbose") = false);

    py_class.def("K_2",
        py::detail::overload_cast_impl<const TTCell &, bool>()
        (&TT::K_2, py::const_),
R"(Compute the criticality of the input surfel (face) and returns the
associated 2-clique.

Parameters
----------
face: Cell
    Input surfel (2-face)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 2-clique]
)", py::arg("face"), py::arg("verbose") = false);

    py_class.def("K_1",
        py::detail::overload_cast_impl<const TTCell &, bool>()
        (&TT::K_1, py::const_),
R"(Compute the criticality of the input linel and the associated 1-clique.

Parameters
----------
linel: Cell
    Input linel between two pointels (1-dimension cell)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 1-clique]
)", py::arg("linel"), py::arg("verbose") = false);

    py_class.def("K_0",
        py::detail::overload_cast_impl<const TTCell &, bool>()
        (&TT::K_0, py::const_),
R"(Compute the criticality of the input pointel and the associated 1-clique.

Parameters
----------
pointel: Cell
    Input pointel (0-dimension cell)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 0-clique]
)", py::arg("pointel"), py::arg("verbose") = false);

    py_class.def("K_3",
        py::detail::overload_cast_impl<const TTCell &, bool>()
        (&TT::K_3, py::const_),
R"(Compute the criticality of the input spel and the associated 3-clique.
Note: It uses isSimple to check criticality.

Parameters
----------
spel: Cell
    Input spel (3-dimension cell)
verbose: Bool [False]
    Extra verbosity when computing it.
Return
------
pair of types [Bool, CubicalComplex] -> [is_critical, 0-clique]
)", py::arg("spel"), py::arg("verbose") = false);

    // --------------------- Simplicity checks --------------------------------
    py_class.def("isSimpleByThinning", &TT::isSimpleByThinning,
R"(Check if the input_spel from khalimsky space is simple using thinning.
First create a CubicalComplex from the neighbor voxels on the input spel
This does not include the input spel itself, close the new clique and apply
a collapse operation.
The input spel is simple in the complex, if after the collapse on the proper
neighborhood clique, there is only one pointel.
either a simplicity_table if loaded, or isSimpleByThinning.
Parameters
----------
spel: Cell
    Input spel (cell of dimension 3)
Return
------
True if the input spel is simple within the complex.
)",py::arg("spel"));

    py_class.def("isSimple", &TT::isSimple,
R"(Check if the input_spel from khalimsky space is simple using
either a simplicity_table if loaded, or isSimpleByThinning.
Parameters
----------
spel: Cell
    Input spel (cell of dimension 3)
Return
------
True if the input spel is simple within the complex.
)",py::arg("spel"));


    // TODO wrap DistanceTransformation
    using TDistanceTransform = DGtal::DistanceTransformation<
          DGtal::Z3i::Space,
          DGtal::Python::DigitalSetZ3i,
          DGtal::ExactPredicateLpSeparableMetric<DGtal::Z3i::Space, 3>>;
    py_class.def("thinningVoxelComplex", [](TT &self,
      const std::string & skel_type,
      const std::string & skel_select_type,
      const std::string & tables_folder,
      const int & persistence,
      // const TDistanceTransform * distance_transform,
      const bool profile,
      const bool verbose) {
        return DGtal::functions::thinningVoxelComplex<
            TT, TDistanceTransform>(
                self, skel_type, skel_select_type,
                tables_folder, persistence,
                // distance_transform,
                nullptr,
                profile, verbose);
    },
R"(Get a skeletonized or thinned image from a binary image set.

Parameters:
----------
image_set: str
    input set representing the ON points of a binary image.

skel_type: str
    Voxels to keep in the skeletonization process.

    [end, ulti, isthmus]
    - end: keep end voxels.
    - ulti: don't keep extra voxels, ultimate skeleton.
    - isthmus: keep voxels that are isthmuses.

select_type: str
    [first, random, dmax]
    - first: the first voxel in the set (no criteria)
    - random: random voxel in the set.
    - dmax: choose voxel with greatest distance map value.
    Strategy to choose voxels in the asymmetric process.

table_folder: str
    Location of the DGtal look-up-tables for simplicity and isthmusicity,
    for example simplicity_table26_6.zlib.
    These tables are distributed with the sgext package. Use the variable
    'sgext.tables_folder'.

persistence: int
    if >0, performs a persistence algorithm that prunes
    branches that are not persistant (less important).

TODO remove or fix
distance_transform: str
    file holding a distance map. Required for select_type dmax option.
    This option provides a centered skeleton.


profile: bool
    time the algorithm

verbose: bool
    extra information displayed during the algorithm.

Return
------
A new thinned voxel complex.
)", py::arg("skel_type"),
    py::arg("select_type"),
    py::arg("tables_folder"),
    py::arg("persistence") = 0,
    py::arg("profile") = false,
    py::arg("verbose") = false
    );

    py_class.def_property_readonly_static("TDigitalSet",
            [](py::object /* self */) {
            return py::type::of<DGtal::Python::DigitalSetZ3i>();
            });

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
