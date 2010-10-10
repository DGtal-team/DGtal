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
//LICENSE-END
#pragma once

/**
 * @file BasicFunctors.h
 * @author Guillaume Damiand (\c guillaume.damiand@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/02
 *
 * Header file for module BasicFunctors.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(BasicFunctors_RECURSES)
#error Recursive header files inclusion detected in BasicFunctors.h
#else // defined(BasicFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define BasicFunctors_RECURSES

#if !defined BasicFunctors_h
/** Prevents repeated inclusion of headers. */
#define BasicFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <algorithm>
#include <functional>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  template<typename T>
  struct MinFunctor
  {
    inline
    T operator() (const T&a, const T&b) const
    { return std::min(a,b); }
  };
  
  template<typename T>
  struct MaxFunctor
  {
    inline
    T operator() (const T&a, const T&b) const
    { return std::max(a,b); }
  };

  /**
   * Copy of the std::minus binary operator (not implemented on MS-VS)
   */
  template <class T> 
  struct MinusFunctor : binary_function <T,T,T>
  {
    T operator() (const T& x, const T& y) const
    {return x-y;}
  };

}
///////////////////////////////////////////////////////////////////////////////

#endif // !defined BasicFunctors_h

#undef BasicFunctors_RECURSES
#endif // else defined(BasicFunctors_RECURSES)
