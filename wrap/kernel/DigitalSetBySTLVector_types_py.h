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

#ifndef DGTAL_DIGITALSETBYSTLVECTOR_TYPES_PY_H
#define DGTAL_DIGITALSETBYSTLVECTOR_TYPES_PY_H

#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"

#include "kernel/HyperRectDomain_types_py.h" // For DomainZ2i

namespace DGtal {
    namespace Python {
        using DigitalSetZ2i = DigitalSetBySTLVector<DomainZ2i>;
        using DigitalSetZ3i = DigitalSetBySTLVector<DomainZ3i>;
    } // namespace Python
} // namespace DGtal
#endif
