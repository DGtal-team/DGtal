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
 * @file FakeKeyValuePair.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/07/09
 *
 * Header file for module FakeKeyValuePair.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FakeKeyValuePair_RECURSES)
#error Recursive header files inclusion detected in FakeKeyValuePair.h
#else // defined(FakeKeyValuePair_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FakeKeyValuePair_RECURSES

#if !defined FakeKeyValuePair_h
/** Prevents repeated inclusion of headers. */
#define FakeKeyValuePair_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class FakeKeyValuePair
  /**
   * Description of template class 'FakeKeyValuePair' <p> \brief Aim: To
   * represents a pair<const Key, Value&>, where the first member is
   * not mutable and the second member may be used as a lvalue.
   */
  template <typename T1, typename T2>
  class FakeKeyValuePair
  {
    // ----------------------- Standard services ------------------------------
  public:
    typedef const T1 FirstType;
    typedef T2 & SecondType;

    typedef FirstType first_type;
    typedef SecondType second_type;

    FirstType first;
    SecondType second;

    /**
     * Destructor.
     */
    inline
    ~FakeKeyValuePair() {}

    /**
       Pair constructor.
     */
    inline
    FakeKeyValuePair( FirstType a1, SecondType a2)
      : first( a1 ), second( a2 )
    {}

  private:
    /**
     * Default constructor.
     */
    FakeKeyValuePair();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    FakeKeyValuePair ( FakeKeyValuePair & other );
    //   : first( other.first ), second( other.second )
    // {}

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    FakeKeyValuePair & operator= ( const FakeKeyValuePair & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class FakeKeyValuePair


  template <typename T1, typename T2>
  inline 
  bool operator==( const std::pair<const T1, T2> & p1, 
                   const FakeKeyValuePair<T1, T2> & p2 )
  {
    return ( p1.first == p2.first ) && ( p1.second == p2.second );
  }
  template <typename T1, typename T2>
  inline 
  bool operator==( const FakeKeyValuePair<T1, T2> & p2, 
                   const std::pair<const T1, T2> & p1 )
  {
    return ( p1.first == p2.first ) && ( p1.second == p2.second );
  }


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FakeKeyValuePair_h

#undef FakeKeyValuePair_RECURSES
#endif // else defined(FakeKeyValuePair_RECURSES)
