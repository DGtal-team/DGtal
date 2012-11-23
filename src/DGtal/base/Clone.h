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
 * @file Clone.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/23
 *
 * Header file for module Clone.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Clone_RECURSES)
#error Recursive header files inclusion detected in Clone.h
#else // defined(Clone_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Clone_RECURSES

#if !defined Clone_h
/** Prevents repeated inclusion of headers. */
#define Clone_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Clone
  /**
     Description of template class 'Clone' <p> \brief Aim: This class
     encapsulates its parameter class so that to indicate to the user
     that the object will be duplicated (or cloned). Note that an
     instance of Clone<T> is itself a light object (it holds only a
     const reference), the duplication takes place when the user
     instantiates its member of type T.

     @tparam T is any type.

     It can be used as follows. Consider this simple example where
     class A is a big object. Then we define two classes B1 and B2
     that uses some instance of A. 
     
     @code
     const int N = 10000;
     struct A { ...
       int table[N];
     };

     // Both B1 and B2 uses A, but we do not know if A will be copied
     // or just const-reference by only looking at the declaration of
     // the method. Generally the ambiguity is removed by adding
     // comments or, for the experienced developper, by looking at
     // other parts of the code.
     struct B1 {
       B1( const A & a ) // ambiguous, cost is O(1) here
       : myA( a ) {}
     ...
       const A & myA;
     };
     struct B2 {
       B2( const A & a ) // ambiguous, cost is O(N) here
       : myA( a ) {}
     ...
       A myA;
     };
     @endcode

     Sometimes it is also very important that the developper that uses
     the library is conscious that an object, say \a b, may require
     that an instance \a a given as parameter should have a lifetime
     longer than \a b itself (case for an instance of \a B1
     above). Classes Clone, Alias, ConstAlias exist for these
     reasons. The classes above may be rewritten as follows.
     
     @code
     struct B1 {
       B1( ConstAlias<A> a ) // not ambiguous, cost is O(1) here and lifetime of a should be long enough
       : myA( a ) {}
     ...
       const A & myA;
     };
     struct B2 {
       B2( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
     ...
       A myA;
     };
     ...
     A a1;
     B2 b( a1 );   // duplicate a1
     B2 bb( &a1 ); // also duplicate a1 !
     @endcode

     A last question could be why are we not just passing the instance
     of A by value. This, for sure, would tell the developper that the
     instance is duplicated somehow. The problem is that it induces
     generally two duplications, and not only one ! It may be possible
     that the compiler optimizes things nicely but it is unclear if
     the compiler will always do it.

     @code
     struct B3 {
       B3( A a ) // not ambiguous, but cost is O(2N) here. 
       : myA( a ) {}
     ...
       A myA;
     };
     A a1;
     B3 b3( a1 ) // The object \a a1 is copied once on the heap as the parameter \a a, and once as the member \a b3.myA.
     @endcode
   */
  template <typename T>
  class Clone
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor. Does nothing.
     */
    ~Clone();

    /**
       Constructor from an instance of T. The object is referenced in
       'this' and is generally immediately duplicated by the user to
       instantiate a data member.
       @param t any object of type T.
    */
    Clone( const T & t );

    /**
       Constructor from a pointer to a valid instance of T. The object is referenced in
       'this' and is generally immediately duplicated by the user to
       instantiate a data member.
       @param ptrT any valid pointer to a object of type T.
       @pre ptrT != 0
    */
    Clone( const T* ptrT );

    /**
       Cast operator to a T instance. This is only at this moment that
       the object is duplicated.  This allows things like: A a2 = a1;
       where a1 is of type Clone<A>.
    */
    operator T() const;

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:
    /// The reference to the T instance that is to be duplicated.
    const T & myRefT;


    // ------------------------- Hidden services ------------------------------
  private:

    /**
     * Constructor.
     * Forbidden.
     */
    Clone();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden (otherwise the user might be tempted to use it as a member).
     */
    Clone ( const Clone & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden (otherwise the user might be tempted to use it as a member).
     */
    Clone & operator= ( const Clone & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Clone

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/Clone.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Clone_h

#undef Clone_RECURSES
#endif // else defined(Clone_RECURSES)
