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

#ifndef DGTAL_HYPERRECTDOMAIN_TYPES_PY_H
#define DGTAL_HYPERRECTDOMAIN_TYPES_PY_H

#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/SpaceND.h"
#include "base/Common_types_py.h"

namespace DGtal {
    namespace Python {
        using Z2i  = SpaceND<2, DGtal::Python::Integer>;
        using DomainZ2i = HyperRectDomain <Z2i>;

        using Z3i  = SpaceND<3, DGtal::Python::Integer>;
        using DomainZ3i = HyperRectDomain <Z3i>;
    } // namespace Python
} // namespace DGtal
#endif
