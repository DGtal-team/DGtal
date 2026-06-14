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

#pragma once

/**
* @file GAVector.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(GAVector_RECURSES)
#error Recursive header files inclusion detected in GAVector.h
#else // defined(GAVector_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GAVector_RECURSES

#if !defined GAVector_h
/** Prevents repeated inclusion of headers. */
#define GAVector_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/kernel/domains/CDomain.h>

namespace DGtal {
    template<typename TSpace,typename TInputValue = typename TSpace::Point>
    struct GAVector{
        TInputValue my_gavec;

        /// @brief zero multivector
        explicit GAVector( const TInputValue& pt= TInputValue(0,0))
          : my_gavec( pt ) {}

        /// @brief scalar part of the a mv, i.e. compute the dot product part of the geometric product
        typename TInputValue::Component dot( GAVector other ) const
        {
            return my_gavec[0] * other.my_gavec[0] + my_gavec[1] * other.my_gavec[1];
        }

        /// @brief bivector part of the a mv, i.e. compute the outer product part of the geometric product
        typename TInputValue::Component bivectorPart( const GAVector& other ) const
        {
            return my_gavec[0] * other.my_gavec[1] - my_gavec[1] * other.my_gavec[0];
        }

        /// @brief geometric product with a scalar
        GAVector operator*( typename TInputValue::Component f ) const
        {
            return GAVector( TInputValue(my_gavec[0] * f, my_gavec[1] * f) );
        }

        /// @brief geometric product between two vectors
        GAVector operator*(const GAVector& v )// geometric product
        {
            return GAVector( TInputValue(this->dot(v), this->bivectorPart(v)) );
        }
        bool operator<( const GAVector& other ) const
        {
            return bivectorPart( other ) > 0;
        }

        double angleToXAxis() const
        {
            return atan2( my_gavec[1], my_gavec[0] );
        }

        bool operator==( const GAVector& other ) const
        {
            return bivectorPart( other ) == 0;
        }


    };
}

#endif //GAVector

#undef GAVector_RECURSES
#endif // else defined(GAVector_RECURSES)
