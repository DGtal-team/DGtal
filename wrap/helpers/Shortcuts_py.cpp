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
 **/

/**
 * @file Shortcuts_py.cpp
 * @author Bastien Doignies <bastien.doignies@liris.cnrs.fr>
 *
 * @date 2025/05/11
 *
 * Main file for helpers/Shortcuts binding
 *
 * This file is part of the DGtal library.
 */

#include "dgtal_pybind11_common.h"
#include "pybind11/numpy.h"
namespace py = pybind11;

#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"

using namespace DGtal;
using SH3 = Shortcuts<Z3i::KSpace>;
using SHG3 = ShortcutsGeometry<Z3i::KSpace>;

// Defines to avoid renaming types
using L1Metric = ExactPredicateLpSeparableMetric<SHG3::Space, 1>;
using L2Metric = ExactPredicateLpSeparableMetric<SHG3::Space, 2>;
using L1DistanceTransform = DistanceTransformation<SHG3::Space, SHG3::VoronoiPointPredicate, L1Metric>;
using L2DistanceTransform = DistanceTransformation<SHG3::Space, SHG3::VoronoiPointPredicate, L2Metric>;

template<typename T>
std::vector<Color> apply_colormap(const std::vector<T>& data, const SH3::ColorMap& cmap) { 
    std::vector<Color> colors(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        colors[i] = cmap(data[i]);
    }
    return colors;
}
// Named apply_colormapV to ease binding and not conflict with above declaration
template<typename T>
std::vector<Color> apply_colormapV(const std::vector<T>& data, const Parameters& params) { 
    if (data.size() == 0) return {};

    T min = data[0];
    T max = data[0];
    for (size_t i = 1; i < data.size(); ++i) {
        min = std::min(min, data[i]);
        max = std::max(max, data[i]);
    }
    
    auto cmap = SH3::getColorMap(min, max, params);
    return apply_colormap(data, cmap);
}

template<typename It, typename Embd>
py::array_t<double> getEmbeddedPosition(Embd& embedder, size_t size, It it, It end) {
    constexpr size_t eSize  = sizeof(double);
    const size_t shape[2]   = {size, 3};
    const size_t strides[2] = {3 * eSize, eSize};
    auto array = py::array_t<double>(shape, strides);
    auto view  = array.mutable_unchecked<2>();

    for (size_t i = 0; it != end; ++it, ++i) {
        auto pos = embedder(*it);
        
        view(i, 0) = pos[0];
        view(i, 1) = pos[1];
        view(i, 2) = pos[2];
    }

    return array;
}

void define_types(py::module& m) {
    // We define some "opaques" type that are only be meant to be returned to the user
    // and pass to other functions afterwards. These are not meant to be full binding 
    // of the types.
    
    // Non bound types assumed to be bound within other modules:
    // (this was checked by trial and error)
    //  SH3::KSpace
    //  Color
    //  Points
    //  CellEmbedder
    //  SCellEmbedder
    
    // TODO: In IO module?
    py::class_<SH3::ColorMap>(m, "ColorMap")
        .def(py::init<double, double>())
        .def("addColor", &SH3::ColorMap::addColor)
        .def("__call__", &SH3::ColorMap::operator());

    // Ranges
    py::class_<SH3::SurfelRange>(m, "SurfelRange"); // This is a vector, so len() is available !
    py::class_<SH3::IdxRange>(m, "IdxRange");

    // Containers
    py::class_<CountedPtr<SH3::BinaryImage>>(m, "__PtrBinaryImage");
    py::class_<CountedPtr<SH3::GrayScaleImage>>(m, "__PtrGrayScaleImage");
    py::class_<CountedPtr<SH3::LightDigitalSurface>>(m, "__PtrLightDigitalSurface")
        .def("__len__", [](const CountedPtr<SH3::LightDigitalSurface>& surf) {
            return surf->size();
        })
        .def("size", [](const CountedPtr<SH3::LightDigitalSurface>& surf) {
            return surf->size();
        });
    py::class_<CountedPtr<SH3::TriangulatedSurface>>(m, "__PtrTriangulatedSurface");
    py::class_<CountedPtr<SH3::Mesh>>(m, "__PtrMesh")
        .def("__len__", [](const CountedPtr<SH3::LightDigitalSurface>& surf) {
            return surf->size();
        })
        .def("nbVertex", [](const CountedPtr<SH3::Mesh>& mesh) {
            return mesh->nbVertex();
        });
    py::class_<CountedPtr<SH3::ImplicitShape3D>>(m, "__PtrImplicitShape3D");
    py::class_<CountedPtr<SH3::DigitizedImplicitShape3D>>(m, "__PtrDigitizedImplicitShape3D");
    py::class_<SH3::Cell2Index>(m, "Cell2Index")
        .def(py::init<>());

}

void defines_types_geometry(py::module& m) {
    // TODO: Should probably be bound in math header instead (but it does not exist yet)
    py::class_<SHG3::ScalarStatistic>(m, "ScalarStatistic")
        .def(py::init<>())
        .def("samples", &SHG3::ScalarStatistic::samples)
        .def("size", &SHG3::ScalarStatistic::size)
        .def("mean", &SHG3::ScalarStatistic::mean)
        .def("variance", &SHG3::ScalarStatistic::variance)
        .def("unbiasedVariance", &SHG3::ScalarStatistic::unbiasedVariance)
        .def("min", &SHG3::ScalarStatistic::min)
        .def("max", &SHG3::ScalarStatistic::max)
        .def("values", [](const SHG3::ScalarStatistic& in) {
            return std::vector(in.begin(), in.end());
        })
        .def("__str__", [](const SHG3::ScalarStatistic& in) {
            std::stringstream ss;
            in.selfDisplay(ss);
            return ss.str();
        });

    // We map distance transformation and voronoi map under the same name. 
    // We just "cheat" by binding functions under different names
    py::class_<L1DistanceTransform>(m, "L1VoronoiMap")
        // No constructor, we let the user build it with getDistanceTransformation
        .def("distanceToClosestSite", [](const L1DistanceTransform& vmap, const SHG3::Point& p){
                return vmap(p);
            })
        .def("distanceToClosestSite", [](const L1DistanceTransform& vmap, const std::vector<int>& p){
                return vmap(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("distanceToClosestSite", [](const L1DistanceTransform& vmap, const std::vector<float>& p){
                return vmap(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("directionToClosestSite", [](const L1DistanceTransform& vmap, const SHG3::Point& p){
                return vmap.getVoronoiSite(p);
            })
        .def("directionToClosestSite", [](const L1DistanceTransform& vmap, const std::vector<int>& p){
                return vmap.getVoronoiSite(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("directionToClosestSite", [](const L1DistanceTransform& vmap, const std::vector<float>& p){
                return vmap.getVoronoiSite(SHG3::Point(p[0], p[1], p[2]));
            });

    py::class_<L2DistanceTransform>(m, "L2VoronoiMap")
        // No constructor, we let the user build it with getDistanceTransformation
        .def("distanceToClosestSite", [](const L2DistanceTransform& vmap, const SHG3::Point& p){
                return vmap(p);
            })
        .def("distanceToClosestSite", [](const L2DistanceTransform& vmap, const std::vector<int>& p){
                return vmap(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("distanceToClosestSite", [](const L2DistanceTransform& vmap, const std::vector<float>& p){
                return vmap(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("directionToClosestSite", [](const L2DistanceTransform& vmap, const SHG3::Point& p){
                return vmap.getVoronoiSite(p);
            })
        .def("directionToClosestSite", [](const L2DistanceTransform& vmap, const std::vector<int>& p){
                return vmap.getVoronoiSite(SHG3::Point(p[0], p[1], p[2]));
            })
        .def("directionToClosestSite", [](const L2DistanceTransform& vmap, const std::vector<float>& p){
                return vmap.getVoronoiSite(SHG3::Point(p[0], p[1], p[2]));
            });}

void bind_voronoimap(py::module& mg);

void bind_shortcuts(py::module& m_helpers) {
    auto m = m_helpers.def_submodule("SH3", "Shortcuts 3D"); 
    define_types(m);

    // Create a reference for easy split between SH3 and SHG3 if needed
    auto& mg = m;
    defines_types_geometry(mg);

    
    // Returns both for now
    m.def("defaultParameters", []() {
            return SH3::defaultParameters() | SHG3::defaultParameters();
        });

    // Note: We use lambda because default parameters or partial specialization results 
    // in multiple overloads that are not disambiguated with py::overload_cast...
    // Even when it could, we use lambda for consistency
    
    // Create objects
    m.def("makeBinaryImage", [](const std::string& fname, Parameters params){
            return SH3::makeBinaryImage(fname, params);
        });
    m.def("makeBinaryImage", [](CountedPtr<SH3::DigitizedImplicitShape3D> shape, Parameters params) {
            return SH3::makeBinaryImage(shape, params);
        });
    m.def("makeBinaryImage", [](CountedPtr<SH3::DigitizedImplicitShape3D> shape, const SH3::Domain& d, Parameters params) {
            return SH3::makeBinaryImage(shape, d, params);
        });
    m.def("makeBinaryImage", [](const std::vector<SH3::Point>& positions) {
            return SH3::makeBinaryImage(positions);
        });
    m.def("makeBinaryImage", [](const std::vector<float>& positions, const SH3::Domain& d) {
            return SH3::makeBinaryImage(positions, d);
        });
    m.def("makeGrayScaleImage", [](const std::string& fname){
            return SH3::makeGrayScaleImage(fname);
        });
    m.def("makeGrayScaleImage", [](const std::vector<float>& positions, const SH3::Domain& d) {
            return SH3::makeGrayScaleImage(positions, d);
        });
    m.def("makeGrayScaleImage", [](const std::vector<SH3::Point>& positions, const std::vector<float>& values) {
            return SH3::makeGrayScaleImage(positions, values);
        });
    m.def("makeDigitalSurface", [](CountedPtr<SH3::BinaryImage> im, const SH3::KSpace& K, const Parameters& params) {
            return SH3::makeDigitalSurface(im, K, params);
        });
    m.def("makeLightDigitalSurface", &SH3::makeLightDigitalSurface);
    m.def("makeLightDigitalSurfaces", [](CountedPtr<SH3::BinaryImage> im, const SH3::KSpace& K, const Parameters& params) {
            return SH3::makeLightDigitalSurfaces(im, K, params);
        });
    m.def("makeTriangulatedSurface", [](CountedPtr<SH3::GrayScaleImage> im, Parameters params) {
            return SH3::makeTriangulatedSurface(im, params);
        });
    m.def("makeMesh", [](CountedPtr<SH3::TriangulatedSurface> surf, Color c) {
            return SH3::makeMesh(surf, c);
        });
    m.def("makeImplicitShape3D", &SH3::makeImplicitShape3D);
    m.def("makeDigitizedImplicitShape3D", &SH3::makeImplicitShape3D);
    m.def("makeIdxDigitalSurface", [](CountedPtr<SH3::BinaryImage> im, const SH3::KSpace& K, Parameters params) {
            return SH3::makeIdxDigitalSurface(im, K, params);
        });

    // Save objects
    m.def("saveBinaryImage", &SH3::saveBinaryImage);
    m.def("saveOBJ", [](CountedPtr<SH3::TriangulatedSurface> mesh, const std::string& fName) {
            return SH3::saveOBJ(mesh, fName);
        });
    m.def("saveOBJ", [](
        CountedPtr<SH3::TriangulatedSurface> mesh, 
        const std::vector<SH3::RealPoint>& normals, const std::vector<Color>& colors, 
        const std::string& fName) { // TODO: Ambient, diffuse and specular
            return SH3::saveOBJ(mesh, normals, colors, fName);
        });
    m.def("saveOBJ", [](CountedPtr<SH3::LightDigitalSurface> surf, const std::string& fName) {
            return SH3::saveOBJ(surf, fName);
        });
    m.def("saveOBJ", [](CountedPtr<SH3::DigitalSurface> surf, const std::string& fName) {
            return SH3::saveOBJ(surf, fName);
        });
    m.def("saveOBJ", [](
        CountedPtr<SH3::LightDigitalSurface> surf, 
        const std::vector<SH3::RealPoint>& normals, const std::vector<Color>& colors, 
        const std::string& fName) { // TODO: Ambient, diffuse and specular
            return SH3::saveOBJ(surf, normals, colors, fName);
        });
    // With lambda to avoid setting all parameters for default colors to work
    m.def("saveVectorFieldOBJ", [](
        const SH3::RealPoints& positions, const SH3::RealVectors& vf, 
        double thickness, const SH3::Colors& diffuse_colors, std::string path) {
            return SH3::saveVectorFieldOBJ(positions, vf, thickness, diffuse_colors, path);
        });
    // Getters
    m.def("getKSpace", [](CountedPtr<SH3::BinaryImage> im, Parameters params) {
            return SH3::getKSpace(im, params);
        });
    m.def("getKSpace", [](const SH3::Point& lb, const SH3::Point& ub, Parameters params) {
            return SH3::getKSpace(lb, ub, params);
        });
    m.def("getKSpace", [](const std::vector<int>& lb, const std::vector<int>& ub, Parameters params) {
            if (lb.size() != 3) throw std::runtime_error("Expected a lower bound of dimension 3");
            if (ub.size() != 3) throw std::runtime_error("Expected an upper bound of dimension 3");
            
            return SH3::getKSpace(SH3::Point(lb[0], lb[1], lb[2]), SH3::Point(ub[0], ub[1], ub[2]), params);
        });
    m.def("getCellEmbedder", [](const SH3::KSpace& space) {
            return SH3::getCellEmbedder(space);
        });
    m.def("getSCellEmbedder", [](const SH3::KSpace& space) {
            return SH3::getSCellEmbedder(space);
        });
    m.def("getSurfelRange", [](CountedPtr<SH3::LightDigitalSurface> im, Parameters params) {
            return SH3::getSurfelRange(im, params);
        });
    m.def("getSurfelRange", [](CountedPtr<SH3::DigitalSurface> im, Parameters params) {
            return SH3::getSurfelRange(im, params);
        });
    m.def("getPointelRange", [](SH3::Cell2Index& c2i, CountedPtr<SH3::LightDigitalSurface> surf) {
            return SH3::getPointelRange(c2i, surf);
        });
    m.def("getCellRange", [](CountedPtr<SH3::LightDigitalSurface> surf, int dim) {
        return SH3::getCellRange(surf, dim);
        });
    m.def("getCellRange", [](CountedPtr<SH3::DigitalSurface> surf, int dim) {
        return SH3::getCellRange(surf, dim);
        });
    m.def("getRangeMatch", &SH3::getRangeMatch<SH3::Surfel>);
    m.def("getMatchedRange", &SH3::getMatchedRange<SH3::RealVectors>);

    // Various helper functions
    m.def("getColorMap", &SH3::getColorMap);
    m.def("applyColorMap", &apply_colormap<int>);
    m.def("applyColorMap", &apply_colormap<float>);
    m.def("applyColorMap", &apply_colormap<double>);
    m.def("applyColorMap", &apply_colormapV<int>);
    m.def("applyColorMap", &apply_colormapV<float>);
    m.def("applyColorMap", &apply_colormapV<double>);

    m.def("getEmbeddedPositions", [](const SH3::KSpace& space, const SH3::SurfelRange& range) {
            auto embedder = SH3::getSCellEmbedder(space);
            return getEmbeddedPosition(embedder, range.size(), range.begin(), range.end());
        }); 



    // SHG3 Shortcuts :
    mg.def("getIIMeanCurvatures", [](
        CountedPtr<SH3::BinaryImage> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIIMeanCurvatures(im, range, params);
        });
    mg.def("getIIMeanCurvatures", [](
        CountedPtr<SH3::DigitizedImplicitShape3D> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIIMeanCurvatures(im, range, params);
        });
    mg.def("getIIGaussianCurvatures", [](
        CountedPtr<SH3::BinaryImage> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIIGaussianCurvatures(im, range, params);
        });
    mg.def("getIIGaussianCurvatures", [](
        CountedPtr<SH3::DigitizedImplicitShape3D> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIIGaussianCurvatures(im, range, params);
        });
    mg.def("getIINormalVectors", [](
        CountedPtr<SH3::BinaryImage> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIINormalVectors(im, range, params);
        });
    mg.def("getIINormalVectors", [](
        CountedPtr<SH3::DigitizedImplicitShape3D> im, const SH3::SurfelRange& range, 
        const Parameters& params) {
            return SHG3::getIINormalVectors(im, range, params);
        });
    mg.def("getPositions", [](
        CountedPtr<SH3::ImplicitShape3D> shape, const SH3::KSpace& K, 
        const SH3::SurfelRange& surfels, const Parameters& params) {
            return SHG3::getPositions(shape, K, surfels, params);
        });
    mg.def("getNormalVectors", [](
        CountedPtr<SH3::ImplicitShape3D> shape, const SH3::KSpace& K, 
        const SH3::SurfelRange& surfels, const Parameters& params) {
            return SHG3::getNormalVectors(shape, K, surfels, params);
        });
    mg.def("getMeanCurvatures", [](
        CountedPtr<SH3::ImplicitShape3D> shape, const SH3::KSpace& K, 
        const SH3::SurfelRange& surfels, const Parameters& params) {
            return SHG3::getMeanCurvatures(shape, K, surfels, params);
        });
    mg.def("getGaussianCurvatures", [](
        CountedPtr<SH3::ImplicitShape3D> shape, const SH3::KSpace& K, 
        const SH3::SurfelRange& surfels, const Parameters& params) {
            return SHG3::getGaussianCurvatures(shape, K, surfels, params);
        });
    mg.def("getFirstPrincipalCurvatures", &SHG3::getFirstPrincipalCurvatures);
    mg.def("getSecondPrincipalCurvatures", &SHG3::getSecondPrincipalCurvatures);
    mg.def("getFirstPrincipalDirections", &SHG3::getFirstPrincipalDirections);
    mg.def("getSecondPrincipalDirections", &SHG3::getSecondPrincipalDirections);
    mg.def("getVCMNormalVectors", 
        [](CountedPtr<SH3::LightDigitalSurface> surf, 
           const SH3::SurfelRange& range, const Parameters& params) {
            return SHG3::getVCMNormalVectors(surf, range, params);
        });
    mg.def("getVCMNormalVectors", 
        [](CountedPtr<SH3::DigitalSurface> surf, 
           const SH3::SurfelRange& range, const Parameters& params) {
            return SHG3::getVCMNormalVectors(surf, range, params);
        });
    mg.def("getATScalarFieldApproximation", [](
        std::vector<double>& scalars, 
        const SH3::CellRange& range, 
        CountedPtr<SH3::LightDigitalSurface> lsurface, 
        const SH3::SurfelRange& surfels, 
        const std::vector<double>& input, 
        const Parameters& params) {
            return SHG3::getATScalarFieldApproximation(scalars, range.cbegin(), range.cend(), lsurface, surfels, input, params);
        });
    mg.def("getATScalarFieldApproximation", [](
        std::vector<double>& scalars, 
        const SH3::CellRange& range, 
        CountedPtr<SH3::DigitalSurface> surface, 
        const SH3::SurfelRange& surfels, 
        const std::vector<double>& input, 
        const Parameters& params) {
            return SHG3::getATScalarFieldApproximation(scalars, range.cbegin(), range.cend(), surface, surfels, input, params);
        });
    mg.def("getScalarsAbsoluteDifference", &SHG3::getScalarsAbsoluteDifference);
    mg.def("getStatistic", &SHG3::getStatistic);

    bind_voronoimap(mg);
}

void bind_voronoimap(py::module& mg) {
    mg.def("makeL1VoronoiMap", [](const SH3::Domain& d, const std::vector<SHG3::Point>& points, const Parameters& params) {
            return SHG3::getDistanceTransformation<1>(d, points, params);
        });
    mg.def("makeL1VoronoiMap", [](const CountedPtr<SH3::Domain>& d, const std::vector<SHG3::Point>& points, const Parameters& params) {
            return SHG3::getDistanceTransformation<1>(*d, points, params);
        });
    mg.def("makeL2VoronoiMap", [](const SH3::Domain& d, const std::vector<SHG3::Point>& points, const Parameters& params) {
            return SHG3::getDistanceTransformation<2>(d, points, params);
        });
    mg.def("makeL2VoronoiMap", [](const CountedPtr<SH3::Domain>& d, const std::vector<SHG3::Point>& points, const Parameters& params) {
            return SHG3::getDistanceTransformation<2>(*d, points, params);
        });
    
    mg.def("getL1DistanceToClosestSite", [](const SH3::Domain& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDistanceToClosestSite<1>(points, sites, params);
        });
    mg.def("getL1DistanceToClosestSite", [](const std::vector<SH3::Point>& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDistanceToClosestSite<1>(points, sites, params);
        });
    mg.def("getL1DistanceToClosestSite", [](py::buffer b, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            py::buffer_info info = b.request();

            if (info.format != py::format_descriptor<int>::format())
                throw std::runtime_error("Only arrays of int are supported");

            if (info.ndim != 2)
                throw std::runtime_error("Only (N, 3) sets are supported");

            if (info.shape[1] != 3)
                throw std::runtime_error("Only (N, 3) sets are supported");
                
            // TODO: check for contiguity
            const int* data = static_cast<int*>(info.ptr);
            std::vector<SHG3::Point> points(info.shape[0]);
            for (long int i = 0; i < info.shape[0]; ++i)
                for (size_t j = 0; j < 3; ++j)
                    points[i][j] = data[j + i * 3];

            return SHG3::getDistanceToClosestSite<1>(points, sites, params);
        });
    mg.def("getL2DistanceToClosestSite", [](const SH3::Domain& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDistanceToClosestSite<2>(points, sites, params);
        });
    mg.def("getL2DistanceToClosestSite", [](const std::vector<SH3::Point>& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDistanceToClosestSite<2>(points, sites, params);
        });
    mg.def("getL2DistanceToClosestSite", [](py::buffer b, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            py::buffer_info info = b.request();

            if (info.format != py::format_descriptor<int>::format())
                throw std::runtime_error("Only arrays of int are supported");

            if (info.ndim != 2)
                throw std::runtime_error("Only (N, 3) sets are supported");

            if (info.shape[1] != 3)
                throw std::runtime_error("Only (N, 3) sets are supported");
                
            // TODO: check for contiguity
            const int* data = static_cast<int*>(info.ptr);
            std::vector<SHG3::Point> points(info.shape[0]);
            for (long int i = 0; i < info.shape[0]; ++i)
                for (size_t j = 0; j < 3; ++j)
                    points[i][j] = data[j + i * 3];

            return SHG3::getDistanceToClosestSite<2>(points, sites, params);
        });

    mg.def("getL1DirectionToClosestSite", [](const SH3::Domain& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDirectionToClosestSite<1>(points, sites, params);
        });
    mg.def("getL1DirectionToClosestSite", [](const std::vector<SH3::Point>& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDirectionToClosestSite<1>(points, sites, params);
        });
    mg.def("getL1DirectionToClosestSite", [](py::buffer b, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            py::buffer_info info = b.request();

            if (info.format != py::format_descriptor<int>::format())
                throw std::runtime_error("Only arrays of int are supported");

            if (info.ndim != 2)
                throw std::runtime_error("Only (N, 3) sets are supported");

            if (info.shape[1] != 3)
                throw std::runtime_error("Only (N, 3) sets are supported");
                
            // TODO: check for contiguity
            const int* data = static_cast<int*>(info.ptr);
            std::vector<SHG3::Point> points(info.shape[0]);
            for (long int i = 0; i < info.shape[0]; ++i)
                for (size_t j = 0; j < 3; ++j)
                    points[i][j] = data[j + i * 3];

            return SHG3::getDirectionToClosestSite<1>(points, sites, params);
        });
    mg.def("getL2DirectionToClosestSite", [](const SH3::Domain& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDirectionToClosestSite<2>(points, sites, params);
        });
    mg.def("getL2DirectionToClosestSite", [](const std::vector<SH3::Point>& points, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            return SHG3::getDirectionToClosestSite<2>(points, sites, params);
        });
    mg.def("getL2DirectionToClosestSite", [](py::buffer b, const std::vector<SHG3::Point>& sites, const Parameters& params) {
            py::buffer_info info = b.request();

            if (info.format != py::format_descriptor<int>::format())
                throw std::runtime_error("Only arrays of int are supported");

            if (info.ndim != 2)
                throw std::runtime_error("Only (N, 3) sets are supported");

            if (info.shape[1] != 3)
                throw std::runtime_error("Only (N, 3) sets are supported");
                
            // TODO: check for contiguity
            const int* data = static_cast<int*>(info.ptr);
            std::vector<SHG3::Point> points(info.shape[0]);
            for (long int i = 0; i < info.shape[0]; ++i)
                for (size_t j = 0; j < 3; ++j)
                    points[i][j] = data[j + i * 3];

            return SHG3::getDirectionToClosestSite<2>(points, sites, params);
        });
}
