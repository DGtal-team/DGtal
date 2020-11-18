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

#ifndef DGTAL_COMMON_TYPES_H
#define DGTAL_COMMON_TYPES_H

#include "DGtal/base/Common.h"

namespace DGtal {
    namespace Python {
        using Integer = DGtal::int32_t;
        using Real = double;
        const std::string Integer_str = "int";
        const std::string Real_str = "float";
    }
}

#endif
