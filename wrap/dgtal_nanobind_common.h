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

// This file should be included in all cpp files using nanobind
// to avoid -Wodr violation (Warning).
#ifndef DGTAL_NANOBIND_COMMON_H
#define DGTAL_NANOBIND_COMMON_H

#if defined (_MSC_VER) and !defined(ssize_t)
    // ssize_t is not standard, only posix which is not supported by MSVC
    #define ssize_t ptrdiff_t
#endif

#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/bind_map.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/list.h>
#include <nanobind/operators.h>

#include "topology/KhalimskyPreSpaceND_types_py.h" // For KPreSpace2D
#include "topology/CubicalComplex_types_py.h" // For CellMap2|3D

// To avoid automatic conversion to dictionaries from nanobind STL bindings
// Later, we use nb::bind_map to create bindings.
NB_MAKE_OPAQUE(DGtal::Python::CellMap2D);
NB_MAKE_OPAQUE(DGtal::Python::CellMap3D);

namespace nanobind { namespace detail {

    // Specialize type_caster for KhalimskyPreSpaceND::AnyCellCollection.
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

}} //namespace nanobind::detail


#endif
