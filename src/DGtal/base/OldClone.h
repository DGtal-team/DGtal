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
 * @file OldClone.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/11/23
 *
 * Header file for module OldClone.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(OldClone_RECURSES)
#error Recursive header files inclusion detected in OldClone.h
#else // defined(OldClone_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OldClone_RECURSES

#if !defined OldClone_h
/** Prevents repeated inclusion of headers. */
#define OldClone_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace deprecated {

  /////////////////////////////////////////////////////////////////////////////
  // template class deprecated::Clone
  /**
     Description of template class 'deprecated::Clone' <p> \brief Aim: This class
     encapsulates its parameter class so that to indicate to the user
     that the object will be duplicated (or cloned). Therefore the
     user is reminded to take care of the possible cost of duplicating
     the argument parameter, while he is aware that he does not have to take care
     of the lifetime of the parameter.  Note that an instance of
     deprecated::Clone<T> is itself a light object (it holds only a const
     reference), the duplication takes place when the user
     instantiates its member of type T.

     @remark deprecated since 0.7. Use Clone instead.

     @note The usage of \c deprecated::Clone<T> instead of \c const \c T \c & or
     \c const \c T \c * in parameters is \b always \b recommended when
     the user duplicates the parameter and stores a clone of it as a
     data member for later ise. The usage \c deprecated::Clone<T> instead of \c T
     is \b recommended whenever \c T is big (the object is sometimes
     duplicated twice). When the object is small, writing either \c
     deprecated::Clone<T> or \c T is acceptable.

     @tparam T is any type.

     @see deprecated::Alias
     @see deprecated::ConstAlias

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
     above). Classes deprecated::Clone, Alias, ConstAlias exist for these
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
       B2( deprecated::Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
     ...
       A myA;
     };
     // Cloning on the heap requires call to allocate(), so that the user remembers calling \c delete.
     struct B3_v1 {
       B3_v1( deprecated::Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a.allocate() ) {}
       ~B3_v1() { if ( myA != 0 ) delete myA; }
     ...
       A* myA;
     };
     // Cloning on the heap with CountedPtr mechanism is straightforward.
     struct B3_v2 {
       B3_v2( deprecated::Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
       : myA( a ) {}
       ~B3_v2() {} // CountedPtr takes care of delete.
     ...
       CountedPtr<A> myA;
     };
     // Cloning on the heap with CowPtr mechanism is straightforward.
     struct B3_v3 {
       B3_v3( deprecated::Clone<A> a ) // not ambiguous, cost is O(N) here and lifetime of a is whatever.
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

     @note deprecated::Clone have no copy constructor.

     @note The user should not used deprecated::Clone<T> for data members (in
     fact, he cannot), only as a type for parameters.
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
      Copy constructor.
      @param other the object to clone.

      @note Keep in mind that Alias<T> type should not be used in class
      members (efficiency issue).
    */
    Clone ( const DGtal::deprecated::Clone<T> & other );

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
       where a1 is of type deprecated::Clone<A>.
    */
    operator T() const;

    /**
        Allocates a T instance on the heap and returns its adress.
        @return a pointer on the instance of T allocated on the process heap.
    */
    T* allocate() const;

    /**
       Cast operator to a CountedPtr<T> instance. This is only at this moment that
       the object is duplicated (and only once).  This allows things like: CountedPtr<A> a2 = a1;
       where a1 is of type deprecated::Clone<A>. It also allows CowPtr<A> a2 = a1;
    */
    operator CountedPtr<T>() const;

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
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden (otherwise the user might be tempted to use it as a member).
     */
    DGtal::deprecated::Clone<T> & operator= ( const DGtal::deprecated::Clone<T> & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class deprecated::Clone

  } // namespace deprecated
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/OldClone.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined OldClone_h

#undef OldClone_RECURSES
#endif // else defined(OldClone_RECURSES)
