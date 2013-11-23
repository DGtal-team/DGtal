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
 * @file OldConstAlias.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/23
 *
 * Header file for module OldConstAlias.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(OldConstAlias_RECURSES)
#error Recursive header files inclusion detected in OldConstAlias.h
#else // defined(OldConstAlias_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OldConstAlias_RECURSES

#if !defined OldConstAlias_h
/** Prevents repeated inclusion of headers. */
#define OldConstAlias_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace deprecated {

  /////////////////////////////////////////////////////////////////////////////
  // template class deprecated::ConstAlias
  /**
     Description of template class 'deprecated::ConstAlias' <p> \brief Aim: This class
     encapsulates its parameter class so that to indicate to the user
     that the object/pointer will be only aliased. Therefore the user
     is reminded that the argument parameter is given to the function
     without any additional cost and may be modified, while he is
     aware that the lifetime of the argument parameter must be at
     least as long as the object itself. Note that an instance of
     deprecated::ConstAlias<T> is itself a light object (it holds only a pointer).

     It is used in methods or functions to encapsulate the parameter
     types.

     @remark since 0.7. Use ::DGtal::ConstAlias instead.

     @note The usage of \c deprecated::ConstAlias<T> instead of \c const \c T \c &
     or of \c const \c T \c * in parameters is \b recommended when the
     lifetime of the parameter must exceed the lifetime of the called
     method/function/constructor (often the case in constructor or
     init methods). The usage of \c const \c T \c & or \c const \c T
     \c * instead of \c deprecated::ConstAlias<T> is \b recommended when the
     lifetime of the parameter is not required to exceed the lifetime
     of the called method/function/constructor (often the case in
     standard methods, where the parameter is only used at some point,
     but not referenced after in some data member).

     @tparam T is any type.

     @see deprecated::Alias
     @see deprecated::Clone

     It can be used as follows. Consider this simple example where
     class \e A is a big object. Then we define three classes \e B1,
     \e B2 and \e B3, that uses some instance of \e A.

     @code
     const int N = 10000;
     struct A { ...
       int table[N];
     };

     // Each B1, B2 or B3 uses A, but we do not know if A will be copied
     // or just referenced by only looking at the declaration of
     // the method. Generally the ambiguity is removed by adding
     // comments or, for the experienced developper, by looking at
     // other parts of the code.

     // Only aliasing, but for a long lifetime.
     struct B1 {
       B1( const A & a ) // ambiguous, cost is O(1) here and lifetime of a should exceed constructor.
       : myA( a ) {}
     ...
       const A & myA;
     };
     // Copying as data member (stack or heap depending on B2 instance)
     struct B2 {
       B2( const A & a ) // ambiguous, cost is O(N) here
       : myA( a ) {}
     ...
       A myA;
     };
     // Copying on heap and data member pointing on it.
     struct B3 {
       B3( const A & a ) // ambiguous, cost is O(N) here
       { myA = new A( a ); }
       ~B3()
       { if ( myA != 0 ) delete myA; }
     ...
       A* myA;
     };
     @endcode

     Sometimes it is also very important that the developper that uses
     the library is conscious that an object, say \a b, may require
     that an instance \a a given as parameter should have a lifetime
     longer than \a b itself (case for an instance of \a B1
     above). Classes Clone, Alias, deprecated::ConstAlias exist for these
     reasons. The classes above may be rewritten as follows.

     @code
     // Aliasing for a long lifetime is visible.
     struct B1 {
       B1( ConstAlias<A> a ) // not ambiguous, cost is O(1) here and lifetime of a should be long enough
       : myA( a ) {}
     ...
       const A & myA;
       // or Const A* myA;
     };
     // Cloning as data member is visible.
     struct B2 {
       B2( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
     ...
       A myA;
     };
     // Cloning on the heap requires call to allocate(), so that the user remembers calling \c delete.
     struct B3_v1 {
       B3_v1( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a.allocate() ) {}
       ~B3_v1() { if ( myA != 0 ) delete myA; }
     ...
       A* myA;
     };
     // Cloning on the heap with CountedPtr mechanism is straightforward.
     struct B3_v2 {
       B3_v2( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
       ~B3_v2() {} // CountedPtr takes care of delete.
     ...
       CountedPtr<A> myA;
     };
     // Cloning on the heap with CowPtr mechanism is straightforward.
     struct B3_v3 {
       B3_v3( Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
       ~B3_v3() {} // CountedPtr takes care of delete.
     ...
       CowPtr<A> myA;
     };
     ...
     A a1;
     B1 b( a1 );    // do not duplicate a1
     B2 bb( a1 );   // duplicate a1
     B2 bbb( &a1 ); // also duplicate a1 !
     B3_v1 c1( a1 ); // duplicate a1 on the heap
     B3_v2 c1( a1 ); // duplicate a1 on the heap
     B3_v3 c1( a1 ); // duplicate a1 on the heap
     @endcode

     A last question could be why are we not just passing the instance
     of A by value. This, for sure, would tell the developper that the
     instance is duplicated somehow. The problem is that it induces
     generally two duplications, and not only one ! It may be possible
     that the compiler optimizes things nicely but it is unclear if
     the compiler will always do it.

     @code
     struct B4 {
       B4( A a ) // not ambiguous, but cost is O(2N) here.
       : myA( a ) {}
     ...
       A myA;
     };
     A a1;
     B4 b4( a1 ) // The object \a a1 is copied once on the heap as the parameter \a a, and once as the member \a b3.myA.
     @endcode

     @note The user should not use deprecated::ConstAlias<T> instead of const T*
     for data members. It works in most cases, but there are some
     subtle differences between the two behaviors.

     @note deprecated::ConstAlias has no copy constructor. Indeed, if it had one,
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
      Copy constructor.
      @param other the object to clone.

      @note Keep in mind that Alias<T> type should not be used in class
      members (efficiency issue).

    */
    ConstAlias ( const DGtal::deprecated::ConstAlias<T> & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    DGtal::deprecated::ConstAlias<T> & operator= ( DGtal::deprecated::ConstAlias<T> & other );

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
       of type deprecated::ConstAlias<A>.
    */
    operator const T&() const;

    /**
       @return the address of the const-aliased object.
       @since 0.7
    */
    inline const T* operator&() const { return myPtrT; }

    // ------------------------- Private Datas --------------------------------
  private:
    /// The pointer to the instance of T.
    const T* myPtrT;

    // ------------------------- Hidden services ------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class deprecated::ConstAlias


  /**
   * Overloads 'operator<<' for displaying objects of class 'deprecated::ConstAlias'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'deprecated::ConstAlias' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const DGtal::deprecated::ConstAlias<T> & object );

  } // namespace deprecated
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/OldConstAlias.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined OldConstAlias_h

#undef OldConstAlias_RECURSES
#endif // else defined(OldConstAlias_RECURSES)
