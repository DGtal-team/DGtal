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

#ifndef DGTAL_DIGITALTOPOLOGY_DECLARE_PY_H
#define DGTAL_DIGITALTOPOLOGY_DECLARE_PY_H

#if defined (_MSC_VER) and !defined(ssize_t)
    // ssize_t is not standard, only posix which is not supported by MSVC
    #define ssize_t ptrdiff_t
#endif

#include "dgtal_nanobind_common.h"

#include "DGtal/topology/DigitalTopology.h"
#include "DigitalTopology_types_py.h"

template<typename TDigitalTopology>
nanobind::class_<TDigitalTopology> declare_DigitalTopology(nanobind::module_ &m,
    const std::string &typestr) {
    namespace nb = nanobind;
    using namespace nanobind::literals;
    using TT = TDigitalTopology;
    using TTPoint = typename TT::Point;
    using TTForegroundAdjacency = typename TT::ForegroundAdjacency;
    using TTBackgroundAdjacency = typename TT::BackgroundAdjacency;
    using DigitalTopologyProperties = DGtal::DigitalTopologyProperties;
    const std::string docs =
R"(Represents a digital topology as a couple of adjacency relations.

The most famous are the (4,8) and (8,4) topologies on Z^2 (see
seminal Rosenfeld paper). The two given adjacency relations
should be defined for all digital points used afterwards. For
instance, they should operate on points of the same dimension.
The first adjacency defines the foreground topology while the
second adjacency defines the background topology. The opposite
topology is the reverse couple. Both adjacencies should be
instantiable.

A digital topology is classically denoted by a couple
(kappa,lambda), which explains the notations in the class.

Example of usage:

    import dgtal
    Topo = dgtal.topology.DT26_6
    ForwardAdj = Topo.TForegroundAdjacency
    BackwardAdj = Topo.TBackgroundAdjacency
    fadj = ForwardAdj()
    badj = BackwardAdj()
    topo = Topo(foreground=fadj, background=badj,
                properties=dgtal.topology.DigitalTopologyProperties.JORDAN_DT)
    print(topo)

)";
    auto nb_class = nb::class_<TT>(m, typestr.c_str(), docs.c_str());

    // ----------------------- Constructors -----------------------------------
    nb_class.def(nb::init<const TT &>());
    nb_class.def(nb::init([]( const TTForegroundAdjacency & kappa,
                    const TTBackgroundAdjacency & lambda,
                    DigitalTopologyProperties properties) {
                return TT(kappa, lambda, properties);
                }),
R"(Defines the digital topology (kappa,lambda).

Parameters
----------
foreground: MetricAdjacency
    (Kappa) The adjacency object chosen for the foreground topology.
background: MetricAdjacency
    (Lambda) The adjacency object chosen for the background topology.
properties: DigitalTopologyProperties
    A hint of the properties of this digital topology, default is UNKNOWN_DT.
)", nb::arg("foreground"), nb::arg("background"), nb::arg("properties")=DigitalTopologyProperties::UNKNOWN_DT);

    // ----------------------- Python operators -------------------------------

    // ----------------------- Class operators --------------------------------

    // ----------------------- Class functions --------------------------------
    // Note, kappa, lambda, names changed because lambda is a restricted keyword in python.
    nb_class.def("foreground", &TT::kappa,
            nb::return_value_policy::reference_internal,
            R"(Reference to foreground adjacency (connectedness))");
    nb_class.def("background", &TT::lambda,
            nb::return_value_policy::reference_internal,
            R"(Reference to background adjacency (connectedness))");
    nb_class.def("properties", &TT::properties, R"(JORDAN iff the topology is Jordan,
NOT_JORDAN iff the topology is known to be NOT_JORDAN, UNKNOWN otherwise.)");

    // ----------------------- Class data -------------------------------------
    nb_class.def_property_readonly_static("TPoint",
            [](nb::object /* self */) {
            return nb::type::of<TTPoint>();
            });
    nb_class.def_property_readonly_static("TForegroundAdjacency",
            [](nb::object /* self */) {
            return nb::type::of<TTForegroundAdjacency>();
            });
    nb_class.def_property_readonly_static("TBackgroundAdjacency",
            [](nb::object /* self */) {
            return nb::type::of<TTBackgroundAdjacency>();
            });
    nb_class.def_property_readonly_static("adjacency_pair_string",
            [](nb::object /*self*/) {
            return std::to_string(TTForegroundAdjacency::bestCapacity()) +
            "_" + std::to_string(TTBackgroundAdjacency::bestCapacity());
            });

    // ----------------------- Print / Display --------------------------------
    nb_class.def("__str__", [](const TT & self) {
        std::stringstream os;
        self.selfDisplay(os);
        return os.str();
    });

    nb_class.def("__repr__", [typestr](const TT & self) {
        std::stringstream os;
        os << typestr;
        os << ": ";
        self.selfDisplay(os);
        return os.str();
    });

    return nb_class;
}
#endif
