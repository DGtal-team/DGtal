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
namespace py = pybind11;

#include "DGtal/helpers/Shortcuts.h"

using namespace DGtal;
using SH3 = Shortcuts<Z3i::KSpace>;
template<typename T>

std::vector<Color> apply_colormap(const T* data, size_t count, const Parameters& params) {
    if (count == 0) return {};

    T min = data[0];
    T max = data[0];
    for (size_t i = 1; i < count; ++i) {
        min = std::min(min, data[i]);
        max = std::max(max, data[i]);
    }
    
    auto cmap = SH3::getColorMap(min, max, params);

    std::vector<Color> colors(count);
    for (size_t i = 0; i < count; ++i) {
        colors[i] = cmap(data[i]);
    }
    return colors;
}

void define_types(py::module& m) {
    // We define some "opaques" type that are only be meant to be returned to the user
    // and pass to other functions afterwards. These are not meant to be full binding 
    // of the types.
    
    // Non bound types assumed to be bound within other modules:
    // (this was checked by trial and error)
    //  SH3::KSpace
    //  Color
    
    // Ranges
    py::class_<SH3::SurfelRange>(m, "__SurfelRange"); // This is a vector, so len() is available !

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
}

void bind_shortcuts(py::module& m) {
    define_types(m);

    m.def("defaultParameters", &SH3::defaultParameters);

    // Note: We use lambda because default parameters or partial specialization results 
    // in multiple overloads that are not disambiguated with py::overload_cast...
    // Even when it could, we use lambda for consistency
    
    // Create objects
    m.def("makeBinaryImage", [](const std::string& fname, Parameters params = SH3::defaultParameters()){
            return SH3::makeBinaryImage(fname, params);
        });
    m.def("makeGrayScaleImage", [](const std::string& fname){
            return SH3::makeGrayScaleImage(fname);
        });
    m.def("makeLightDigitalSurface", &SH3::makeLightDigitalSurface);
    m.def("makeTriangulatedSurface", [](CountedPtr<SH3::GrayScaleImage> im, Parameters params = SH3::defaultParameters()) {
            return SH3::makeTriangulatedSurface(im, params);
        });
    m.def("makeMesh", [](CountedPtr<SH3::TriangulatedSurface> surf, Color c) {
            return SH3::makeMesh(surf, c);
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
    m.def("saveOBJ", [](
        CountedPtr<SH3::LightDigitalSurface> surf, 
        const std::vector<SH3::RealPoint>& normals, const std::vector<Color>& colors, 
        const std::string& fName) { // TODO: Ambient, diffuse and specular
            return SH3::saveOBJ(surf, normals, colors, fName);
        });
    // Getters
    m.def("getKSpace", [](CountedPtr<SH3::BinaryImage> im, Parameters params = SH3::defaultParameters()) {
                return SH3::getKSpace(im, params);
        });
    m.def("getSurfelRange", [](CountedPtr<SH3::LightDigitalSurface> im, Parameters params = SH3::defaultParameters()) {
            return SH3::getSurfelRange(im, params);
        });

    // Various helper functions
    // TODO: Takes a std::vector instead (to be compatible with list)
    m.def("applyColorMap", [](py::buffer array, Parameters params = SH3::defaultParameters()) {
        py::buffer_info info = array.request();

        if (info.ndim != 1) 
            throw std::runtime_error("Expected a linear buffer");

        // Integral types
        if (info.format == py::format_descriptor<char>::format())                  return apply_colormap<char>(static_cast<char*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<short>::format())                 return apply_colormap<short>(static_cast<short*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<int>::format())                   return apply_colormap<int> (static_cast<int*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<long int>::format())              return apply_colormap<long int>(static_cast<long int*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<long long int>::format())         return apply_colormap<long long int>(static_cast<long long int*>(info.ptr), info.shape[0], params);

        // Integral unsigned types
        if (info.format == py::format_descriptor<unsigned char>::format())          return apply_colormap<unsigned char>(static_cast<unsigned char*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<unsigned short>::format())         return apply_colormap<unsigned short>(static_cast<unsigned short*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<unsigned int>::format())           return apply_colormap<unsigned int> (static_cast<unsigned int*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<unsigned long int>::format())      return apply_colormap<unsigned long int> (static_cast<unsigned long int*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<unsigned long long int>::format()) return apply_colormap<unsigned long long int> (static_cast<unsigned long long int*>(info.ptr), info.shape[0], params);
        
        // Floating point types
        if (info.format == py::format_descriptor<float>::format())  return apply_colormap<int>(static_cast<int*>(info.ptr), info.shape[0], params);
        if (info.format == py::format_descriptor<double>::format()) return apply_colormap<int>(static_cast<int*>(info.ptr), info.shape[0], params);

        throw std::runtime_error("Unknown format: expected (unsigned) char/short/int/long int/long long int or float/double arrays");
        
    });
    

}
