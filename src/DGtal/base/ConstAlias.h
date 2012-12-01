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
 * @file ConstAlias.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/23
 *
 * Header file for module ConstAlias.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ConstAlias_RECURSES)
#error Recursive header files inclusion detected in ConstAlias.h
#else // defined(ConstAlias_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ConstAlias_RECURSES

#if !defined ConstAlias_h
/** Prevents repeated inclusion of headers. */
#define ConstAlias_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class ConstAlias
  /**
     Description of template class 'ConstAlias' <p> \brief Aim: This class
     encapsulates its parameter class so that to indicate to the user
     that the object/pointer will be only aliased. Therefore the user
     is reminded that the argument parameter is given to the function
     without any additional cost and may be modified, while he is
     aware that the lifetime of the argument parameter must be at
     least as long as the object itself. Note that an instance of
     ConstAlias<T> is itself a light object (it holds only a pointer).

     It is used in methods or functions to encapsulate the parameter
     types.

     @note The usage of \c ConstAlias<T> instead of \c const \c T \c &
     or of \c const \c T \c * in parameters is \b recommended when the
     lifetime of the parameter must exceed the lifetime of the called
     method/function/constructor (often the case in constructor or
     init methods). The usage of \c const \c T \c & or \c const \c T
     \c * instead of \c ConstAlias<T> is \b recommended when the
     lifetime of the parameter is not required to exceed the lifetime
     of the called method/function/constructor (often the case in
     standard methods, where the parameter is only used at some point,
     but not referenced after in some data member).

     @tparam T is any type.

     @see ConstAlias
     @see Clone

     It can be used as follows. Consider this simple example where
     class A is a big object. Then we define two classes B1 and B2
     that uses some instance of A. 
     
     @code
     const int N = 10000;
     struct A { ...
       int table[N];
     };

     // Both B1 and B2 uses A, but we do not know if A will be copied
     // or just referenced by only looking at the declaration of
     // the method. Generally the ambiguity is removed by adding
     // comments or, for the experienced developper, by looking at
     // other parts of the code.
     struct B1 {
       B1( A & a ) // ambiguous, cost is O(1) here
       : myA( a ) {}
     ...
       A & myA;
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
       // or Const A* myA;
     };
     struct B2 {
       B2( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
     ...
       A myA;
     };
     ...
     A a1;
     B1 b( a1 );    // do not duplicate a1
     B2 bb( a1 );   // duplicate a1
     B2 bbb( &a1 ); // also duplicate a1 !
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

     @note The user can use either ConstAlias<T> or const T* for data
     members. Normally (depending on the compiler), there is no
     overhead.

     @note ConstAlias have no copy constructor. Indeed, if they had one,
     there is an ambiguity when duplicating an alias between the copy
     constructor or the T* cast followed by the T* constructor.
   */
  template <typename T>
  class ConstAlias
  {
    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor. Does nothing.
     */
    ~ConstAlias();

    /**
     * Constructor.
     */
    ConstAlias();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    // ConstAlias ( ConstAlias & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ConstAlias & operator= ( ConstAlias & other );

    /**
       Constructor from an instance of T. The object is pointed in
       'this'.
       @param t any object of type T.
    */
    ConstAlias( const T & t );

    /**
       Constructor from a pointer of T. The pointer is copied in
       'this'.
       @param ptrT any pointer to a object of type T or 0.
    */
    ConstAlias( const T* ptrT );

    /**
       Cast operator to a reference to T instance. Gives access to the
       instance of T.  This allows things like: A a2 = a1; where a1 is
       of type ConstAlias<A>.
    */
    operator const T&() const;
    /**
       Cast operator to a pointer to a T instance. Gives access to the
       instance of T.  This allows things like: A* a2 = a1; where a1 is
       of type ConstAlias<A>.
    */
    operator const T*() const;

    // ------------------------- Private Datas --------------------------------
  private:
    /// The pointer to the instance of T.
    const T* myPtrT;

    // ------------------------- Hidden services ------------------------------
  protected:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ConstAlias


  /**
   * Overloads 'operator<<' for displaying objects of class 'ConstAlias'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ConstAlias' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const ConstAlias<T> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/ConstAlias.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ConstAlias_h

#undef ConstAlias_RECURSES
#endif // else defined(ConstAlias_RECURSES)
