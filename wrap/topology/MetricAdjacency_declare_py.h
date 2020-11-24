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

#ifndef DGTAL_METRICADJACENCY_DECLARE_PY_H
#define DGTAL_METRICADJACENCY_DECLARE_PY_H

#include "dgtal_pybind11_common.h"

#include "DGtal/topology/MetricAdjacency.h"
#include "MetricAdjacency_types_py.h"

template<typename TMetricAdjacency>
pybind11::class_<TMetricAdjacency> declare_MetricAdjacency(pybind11::module &m,
    const std::string &typestr) {
    namespace py = pybind11;
    using TT = TMetricAdjacency;
    using TTPoint = typename TT::Point;
    // Vertex is alias to Point
    using TTVertex = typename TT::Vertex;
    auto py_class = py::class_<TT>(m, typestr.c_str());

    // ----------------------- Constructors -----------------------------------
    py_class.def(py::init());

    // ----------------------- Python operators -------------------------------

    // ----------------------- Class operators --------------------------------

    // ----------------------- Class functions --------------------------------
    // ----------------------- Adjacency services -----------------------------
    py_class.def_static("isAdjacentTo", &TT::isAdjacentTo,
R"(True iff [p1] is adjacent to [p2] according to this adjacency relation.

Parameters
----------
p1: Point
    Any point in this space
p2: Point
    Any point in this space

Return
------
    True iff [p1] is adjacent to [p2] according to this adjacency relation.
)", py::arg("p1"), py::arg("p2"));

    py_class.def_static("isProperlyAdjacentTo", &TT::isProperlyAdjacentTo,
R"(True iff [p1] is adjacent to [p2] according to this adjacency relation and p1 != p2.

Parameters
----------
p1: Point
    Any point in this space
p2: Point
    Any point in this space

Return
------
    True iff [p1] is adjacent to [p2] according to this adjacency relation and p1 != p2.
)", py::arg("p1"), py::arg("p2"));

    // ----------------------- Local graph services --------------------------
    py_class.def_static("bestCapacity", &TT::bestCapacity,
R"(Returns maximum number of neighbors for this adjacency.)");

    py_class.def_static("degree", &TT::degree,
R"(Returns the number of neighbors of the input vertex (point).

Parameters
----------
vertex: Point
    Any vertex (i.e. point) in this space

Return
------
    Returns the number of neighbors of the input vertex (point).
)", py::arg("vertex"));

    py_class.def_static("writeNeighbors", [](const TTVertex & v) {
        using OutType = std::vector<TTPoint>;
        OutType neighs;
        std::back_insert_iterator<OutType> bii(neighs);
        TT::writeNeighbors(bii, v);
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

    // ----------------------- Class data -------------------------------------
    py_class.def_property_readonly_static("TPoint",
            [](py::object /* self */) {
            return py::type::of<TTPoint>();
            });
    py_class.def_property_readonly_static("TVertex",
            [](py::object /* self */) {
            return py::type::of<TTVertex>();
            });
    py_class.def_property_readonly_static("dimension",
            [](py::object /* self */) { return TT::Space::dimension; },
            R"(The dimension of the Space.)");

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
