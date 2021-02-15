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

#ifndef DGTAL_OBJECT_TYPES_PY_H
#define DGTAL_OBJECT_TYPES_PY_H

#include "DGtal/topology/Object.h"

#include "kernel/DigitalSetBySTLVector_types_py.h" // For DigitalSetZ2i
#include "DigitalTopology_types_py.h" // For DT4_8, etc

namespace DGtal {
    namespace Python {
        // --- 2D ---
        using Object4_8 = DGtal::Object<DT4_8, DigitalSetZ2i>;
        using Object8_4 = DGtal::Object<DT8_4, DigitalSetZ2i>;
        // --- 3D ---
        using Object6_18 = DGtal::Object<DT6_18, DigitalSetZ3i>;
        using Object18_6 = DGtal::Object<DT18_6, DigitalSetZ3i>;
        using Object6_26 = DGtal::Object<DT6_26, DigitalSetZ3i>;
        using Object26_6 = DGtal::Object<DT26_6, DigitalSetZ3i>;
    } // namespace Python
} // namespace DGtal
#endif
