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
 * @file GlobalFunctions.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/05/06
 *
 * Header file for module GlobalFunctions.h
 *
 * This file contains the definition of basic functors.
 *
 * This file is part of the DGtal library.
 */

#if defined(GlobalFunctions_RECURSES)
#error Recursive header files inclusion detected in GlobalFunctions.h
#else // defined(GlobalFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GlobalFunctions_RECURSES

#if !defined GlobalFunctions_h
/** Prevents repeated inclusion of headers. */
#define GlobalFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <algorithm>
#include <functional>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal 
{
  
  /** 
   * Return the min bewteen to instance of type T.
   *
   * @tparam T the type of elements to compare (model of
   * boost::LessThanComparable and EqualityComparable).
   * @param a first value
   * @param b second value
   * 
   * @return the minimum between a and b.
   */
  template<typename T>
  T minDGtal(const T & a, const T & b)
  {
    BOOST_CONCEPT_ASSERT((boost::EqualityComparable<T>));
    BOOST_CONCEPT_ASSERT((boost::LessThanComparable<T>));
    if (a<=b) 
      return a;
    else
      return b;
  }
  
  /** 
   * Return the max bewteen to instance of type T.
   *
   * @tparam T the type of elements to compare (model of
   * boost::LessThanComparable and EqualityComparable).
   * @param a first value
   * @param b second value
   * 
   * @return the maximum between a and b.
   */
  template<typename T>
  T maxDGtal(const T & a, const T & b)
  {
    BOOST_CONCEPT_ASSERT((boost::EqualityComparable<T>));
    BOOST_CONCEPT_ASSERT((boost::LessThanComparable<T>));
    if (a>=b) 
      return a;
    else
      return b;
  }
  
  /** 
   * Return the absolute value of an instance of type T.
   *
   * @tparam T the type of elements to compare (model of
   * boost::LessThanComparable).
   * @param a first value
   *
   * @return the absolute value |a|.
   */
  template<typename T>
  T abs(const T & a)
  {
    BOOST_CONCEPT_ASSERT((boost::LessThanComparable<T>));
    if (a<0) 
      return -a;
    else
      return a;
  }
  
  
}
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GlobalFunctions_h

#undef GlobalFunctions_RECURSES
#endif // else defined(GlobalFunctions_RECURSES)
