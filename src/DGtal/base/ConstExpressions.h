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
 * @file ConstExpressions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/11/16
 *
 * Header file for module ConstExpressions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConstExpressions_RECURSES)
#error Recursive header files inclusion detected in ConstExpressions.h
#else // defined(ConstExpressions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstExpressions_RECURSES

#if !defined ConstExpressions_h
/** Prevents repeated inclusion of headers. */
#define ConstExpressions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace functions {
    /// @tparam T any model of bounded number.
    /// @param b a number
    /// @param e a non negative integer
    /// @return the constant expression \f$ b^e \f$, computed at compile time.
    ///
    /// @code
    /// auto v = functions::const_pow( 5, 3 ); // 5^3
    /// @endcode
    template< typename T >
    constexpr T const_pow( T b, unsigned int e) {
      return e == 0 ? T(1) : b * const_pow( b, e - 1 );
    }

    /// @tparam T any model of bounded number.
    /// @param K a non negative number
    /// @param e a non negative integer
    /// @return the index of the middle element in the `e`-dimensional array of width \f$ 2K+1 \f$, computed at compile time.
    ///
    /// @code
    /// auto m1 = functions::const_middle( 1, 2 ); // 4, dans le tableau 3x3
    /// auto m2 = functions::const_middle( 2, 2 ); // 12, dans le tableau 5x5
    /// auto m3 = functions::const_middle( 1, 3 ); // 13, dans le tableau 3x3x3
    /// @endcode
    template< typename T >
    constexpr T const_middle( T K, unsigned int e ) {
      return e <= 1
        ? T( K )
        : K * const_pow( 2 * K + 1, e - 1 ) + const_middle( K, e - 1 );
    }
  } // namespace functions

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConstExpressions_h

#undef ConstExpressions_RECURSES
#endif // else defined(ConstExpressions_RECURSES)
