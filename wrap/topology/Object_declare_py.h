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

#ifndef DGTAL_OBJECT_DECLARE_PY_H
#define DGTAL_OBJECT_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/Object.h"
#include "DGtal/topology/NeighborhoodConfigurations.h" // for loadTable
#include "Object_types_py.h"

template<typename TObject>
pybind11::class_<TObject> declare_Object(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TObject;
    using TTPoint = typename TT::Point;
    // Vertex is alias to Point
    using TTVertex = typename TT::Vertex;
    using TTDomain = typename TT::Domain;
    using TTDigitalSet = typename TT::DigitalSet;
    using TTDigitalTopology = typename TT::DigitalTopology;
    using TTForegroundAdjacency = typename TT::ForegroundAdjacency;
    using TTBackgroundAdjacency = typename TT::BackgroundAdjacency;
    using Connectedness = DGtal::Connectedness;

    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());
    py_class.def(py::init<const TT &>());
    py_class.def(py::init([](const TTDigitalTopology & topo,
                    const TTDigitalSet & point_set, Connectedness cxn){
                return TT(topo, point_set, cxn);}),
R"(Constructor

Parameters
----------
topology: DigitalTopology
     The digital topology chosen for this set, a copy of which is stored in the object.
point_set: DigitalSet
    The set of points of the objects. It is copied in the object.
connectedness: Connectedness (enum)
    The connectedness (DISCONNECTED, CONNECTED, or UNKNOWN). Defaults to UNKNOWN.
)", py::arg("topology"), py::arg("point_set"),
    py::arg("connectedness") = Connectedness::UNKNOWN);

    py_class.def(py::init([](const TTDigitalTopology & topo,
                    const TTDomain & domain){
                return TT(topo, domain);}),
R"(Constructor of an empty object by providing a domain.

Parameters
----------
topology: DigitalTopology
     The digital topology chosen for this set, a copy of which is stored in the object.
domain: Domain
    Any domain related to the given topology.
)", py::arg("topology"), py::arg("domain"));

    // ----------------------- Python operators -------------------------------
    py_class.def("__len__", &TT::size);
    py_class.def("__iter__", [](const TT & self) {
        return py::make_iterator(self.begin(), self.end()); },
        py::keep_alive<0, 1>() /* keep object alive while iterator exists */);
    // ----------------------- Class operators --------------------------------

    // ----------------------- Class functions --------------------------------
    py_class.def("size", &TT::size,
R"(Returns the number of elements in the set.)");

    py_class.def("domain", &TT::domain,
R"(A reference to the embedding domain.)");

    // We only wrap the non-const version
    py_class.def("pointSet", py::detail::overload_cast_impl<>()(&TT::pointSet),
R"(A reference to the point set containing the points of the digital object.)");

    py_class.def("topology", &TT::topology,
R"(A reference to the topology of this object.)");

    py_class.def("adjacency", &TT::adjacency,
R"(A reference to the adjacency of this object.)");

    py_class.def("connectedness", &TT::connectedness,
R"(A reference to the connectedness of this object. CONNECTED, DISCONNECTED, or UNKNOWN.)");

    py_class.def("computeConnectedness", &TT::computeConnectedness,
R"(Computes the connectedness of this object if `connectedness() == UNKNOWN`.
After this, the connectedness would be CONNECTED or DISCONNECTED.)");


    py_class.def("border", &TT::border);

    py_class.def("degree", &TT::degree,
R"(
Parameters
----------
vertex: Point
    Any vertex (i.e. point) of this object
Return
------
    The number of neighbors of the input point (vertex), excluding itself.
)", py::arg("vertex"));

    py_class.def("bestCapacity", &TT::bestCapacity,
R"(Returns maximum number of neighbors for this adjacency.)");

    py_class.def("writeNeighbors", [](const TT & self, const TTVertex & v) {
        using OutType = std::vector<TTPoint>;
        OutType neighs;
        std::back_insert_iterator<OutType> bii(neighs);
        self.writeNeighbors(bii, v);
        return neighs;
      },
R"(Returns a list with the neighbors of input vertex.

Parameters
----------
vertex: Point
    Any vertex (i.e. point) in this space

Return
------
    List with the neighbors of input vertex.
)", py::arg("vertex"));

    py_class.def("neighborhood", &TT::neighborhood,
R"(Let A be this object with foreground adjacency k and N_k(p) the
k-neighborhood of p. Returns the set A intersected with N_k(p).

Note: if only the size of neighborhood is required, use neighborhoodSize().

Parameters
----------
point: Point
    Any point in the domain of the digital object (not necessarily in the object).
Return
------
    The kappa-neighborhood of [point] in this object.
)", py::arg("point"));

    py_class.def("neighborhoodSize", &TT::neighborhoodSize,
R"(The size of the neighborhood.

Parameters
----------
point: Point
    Any point in the domain of the digital object (not necessarily in the object).
Return
------
    The cardinal of the kappa-neighborhood of [point] in this object.
)", py::arg("point"));

    py_class.def("properNeighborhood", &TT::properNeighborhood,
R"(The neighborhood of [point] without point.

Parameters
----------
point: Point
    Any point in the domain of the digital object (not necessarily in the object).
Return
------
    The kappa-neighborhood of [point] in this object without [point].
)", py::arg("point"));

    py_class.def("properNeighborhoodSize", &TT::properNeighborhoodSize,
R"(The size of the properNeighborhood.

Parameters
----------
point: Point
    Any point in the domain of the digital object (not necessarily in the object).
Return
------
    The cardinal of the kappa-neighborhood of [point] in this object without [point].
)", py::arg("point"));

    py_class.def("isSimple", &TT::isSimple,
R"([Bertrand, 1994] A voxel v is simple for a set X if \#C6 [G6 (v,
X)] = \#C18[G18(v, X^c)] = 1, where \#Ck [Y] denotes the number
of k-connected components of a set Y.

We adapt this definition to (kappa,lambda) connectednesses. Be
careful, such a definition is valid only for Jordan couples in
dimension 2 and 3.

Parameters
----------
point: Point
    Any point in the object.
Return
------
    True if this point is simple.
)", py::arg("point"));


    py_class.def("setTable", [](TT & self, const std::string & tables_folder) {
            auto foregroundAdjNumber = self.topology().kappa().bestCapacity();
            auto backgroundAdjNumber = self.topology().lambda().bestCapacity();
            const std::string table_filename = "simplicity_table" + std::to_string(foregroundAdjNumber) + "_" + std::to_string(backgroundAdjNumber);
            const std::string table_suffix = ".zlib";
            const std::string full_path =
                tables_folder + "/" + table_filename +  table_suffix;
            const auto compressed = true;
            auto table_smart_ptr = DGtal::functions::loadTable<TTPoint::dimension>(full_path, compressed);
            self.setTable(table_smart_ptr);
            return full_path;
        },
R"(Load a pre-computed simplicity_table to speed up future isSimple computations in this Object.

Only a string with tables_folder needs to be provided. The right filename: "simplicity_tableX_Y.zlib" will be selected automatically, where X is the foreground adjacency capactity of this Object (4, 8, 6, 18 or 26), and Y is the complementatry background adjacency.

These LUT tables are shipped along the python dgtal package in the folder: dgtal.tables_folder.

Parameters
----------
tables_folder: String
    Parent folder to locate the tables. Usually: tables_folder=dgtal.tables_folder

Return
------
    A string with the full path of the loaded table file.
)", py::arg("tables_folder"));

    py_class.def("geodesicNeighborhood", [](const TT & self, const TTPoint & p, unsigned int k) {
                return self.geodesicNeighborhood(self.adjacency(), p, k);
            },
R"(Geodesic neighborhood of point [point] and order [k] in the object for this Object foreground adjacency.

Parameters
----------
point: Point
    Point of interest to compute its geodesicNeighborhood.
k: Int
    Order

Return
------
    Object with the geodesic Neighborhood of the input point.
)", py::arg("point"), py::arg("k"));

    py_class.def("geodesicNeighborhoodInComplement", [](const TT & self, const TTPoint & p, unsigned int k) {
                return self.geodesicNeighborhoodInComplement(self.adjacency(), p, k);
            },
R"(Geodesic neighborhood of point [point] and order [k] in the complemented object for this Object foreground adjacency.

Parameters
----------
point: Point
    Point of interest to compute its geodesicNeighborhood.
k: Int
    Order

Return
------
    Object with the geodesic Neighborhood of the complemented object of the input point.
)", py::arg("point"), py::arg("k"));

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });
    py_class.def_property_readonly_static("TVertex",
            [](py::object /* self */) {
            return py::type::of<TTVertex>();
            });
    py_class.def_property_readonly_static("TDomain",
            [](py::object /* self */) {
            return py::type::of<TTDomain>();
            });
    py_class.def_property_readonly_static("TDigitalSet",
            [](py::object /* self */) {
            return py::type::of<TTDigitalSet>();
            });
    py_class.def_property_readonly_static("TDigitalTopology",
            [](py::object /* self */) {
            return py::type::of<TTDigitalTopology>();
            });
    py_class.def_property_readonly_static("TForegroundAdjacency",
            [](py::object /* self */) {
            return py::type::of<TTForegroundAdjacency>();
            });
    py_class.def_property_readonly_static("TBackgroundAdjacency",
            [](py::object /* self */) {
            return py::type::of<TTBackgroundAdjacency>();
            });

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
