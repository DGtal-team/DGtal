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

// This file should be included in all cpp files using pybind11
// to avoid -Wodr violation (Warning).
// See https://github.com/pybind/pybind11/issues/1055
#ifndef DGTAL_PYBIND11_COMMON_H
#define DGTAL_PYBIND11_COMMON_H

#include <pybind11/pybind11.h>
#include <pybind11/functional.h>
#include <pybind11/stl_bind.h>
#include <pybind11/stl.h>

#include "topology/KhalimskyPreSpaceND_types_py.h" // For KPreSpace2D
#include "topology/CubicalComplex_types_py.h" // For CellMap2|3D

// To avoid automatic conversion to dictionaries from pybind11/stl.h
// Later, we use py::bind_map to create bindings.
PYBIND11_MAKE_OPAQUE(DGtal::Python::CellMap2D);
PYBIND11_MAKE_OPAQUE(DGtal::Python::CellMap3D);

namespace pybind11 { namespace detail {

    // Specialize py::details::type_caster for KhalimskyPreSpaceND::AnyCellCollection.
    template<typename CellType>
    struct type_caster<
    typename DGtal::Python::KPreSpace2D::AnyCellCollection<CellType>>
        : list_caster<DGtal::Python::KPreSpace2D::AnyCellCollection<CellType>,
        CellType> { };
    template<typename CellType>
    struct type_caster<
    typename DGtal::Python::KPreSpace3D::AnyCellCollection<CellType>>
        : list_caster<DGtal::Python::KPreSpace3D::AnyCellCollection<CellType>,
        CellType> { };

}} //namespace pybind11::detail


#endif
