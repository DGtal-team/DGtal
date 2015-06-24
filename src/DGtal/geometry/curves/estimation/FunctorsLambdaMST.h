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
 * @file FunctorsLambdaMST.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/08
 *
 * Header file for module FunctorsLambdaMST.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FunctorsLambdaMST_RECURSES)
#error Recursive header files inclusion detected in FunctorsLambdaMST.h
#else // defined(FunctorsLambdaMST_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FunctorsLambdaMST_RECURSES

#if !defined FunctorsLambdaMST_h
/** Prevents repeated inclusion of headers. */
#define FunctorsLambdaMST_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <functional>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class FunctorsLambdaMST
/**
 * Description of class 'FunctorsLambdaMST' <p>
 * \brief Aim:
 */

namespace functors
{
  struct Lambda64Function : std::unary_function < double, double >
  {
      double operator() (double x) const
      {
          double e2 = x * x;
          double e3 = e2 * x;
          return 64.0 * ( -e3 * e3 + 3.0 * e3 * e2 - 3.0 * e2 * e2 + e3 );
      }
  };

  struct LambdaSinFromPiFunction : std::unary_function < double, double >
  {
      double operator() (double x) const
      {
          return std::sin ( M_PI * x );
      }
  };

  struct LambdaExponentialFunction : std::unary_function < double, double >
  {
      double operator() (double x) const
      {
          return 2.0 / ( std::exp ( 15.0 * ( x - 0.5 ) ) + std::exp ( -15.0 * ( x - 0.5 ) ) );
      }
  };
}

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FunctorsLambdaMST_h

#undef FunctorsLambdaMST_RECURSES
#endif // else defined(FunctorsLambdaMST_RECURSES)
