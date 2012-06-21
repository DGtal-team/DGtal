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
 * @file IteratorFunctions.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/18
 *
 * Header file for module IteratorFunctions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IteratorFunctions_RECURSES)
#error Recursive header files inclusion detected in IteratorFunctions.h
#else // defined(IteratorFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IteratorFunctions_RECURSES

#if !defined IteratorFunctions_h
/** Prevents repeated inclusion of headers. */
#define IteratorFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include<iterator>

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template functions isNotEmpty

namespace detail {

  template< typename IC > 
  inline
  bool isNotEmpty( const IC& itb, const IC& ite, IteratorType ) {
    return (itb != ite);
  }

  template< typename IC > 
  inline
  bool isNotEmpty( const IC& c1, const IC& c2, CirculatorType) {
// using isValid method does not work with reverse circulator 
//(generally speaking adapters of circulators that does not have any isValid method)
//    return ( ( c1.isValid() ) && ( c2.isValid() ) );  
    IC c; //c is not valid
    return ( (c1 != c) && (c2 != c) ); 
  }

} 

template< typename IC> 
inline
bool isEmpty( const IC& itb, const IC& ite ){
  return !detail::isNotEmpty<IC>( itb, ite, typename IteratorCirculatorTraits<IC>::Type() );
}

template< typename IC> 
inline
bool isNotEmpty( const IC& itb, const IC& ite ){
  return detail::isNotEmpty<IC>( itb, ite, typename IteratorCirculatorTraits<IC>::Type() );
}
  
} // namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/IteratorFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IteratorFunctions_h

#undef IteratorFunctions_RECURSES
#endif // else defined(IteratorFunctions_RECURSES)
