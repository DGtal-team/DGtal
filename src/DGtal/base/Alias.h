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
 * @file Alias.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2013/11/5
 *
 * Header file for module Alias.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Alias_RECURSES)
#error Recursive header files inclusion detected in Alias.h
#else // defined(Alias_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Alias_RECURSES

#if !defined Alias_h
/** Prevents repeated inclusion of headers. */
#define Alias_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/CowPtr.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Alias
  /**
Description of template class 'Alias' <p> 

\brief Aim: This class encapsulates its parameter class so that
to indicate to the user that the object/pointer will be only
aliased. Therefore the user is reminded that the argument
parameter is given to the function without any additional cost
and may be modified, while he is aware that the lifetime of the
argument parameter must be at least as long as the object
itself. Note that an instance of Alias<T> is itself a light
object (it holds only an enum and a pointer).

It is used in methods or functions to encapsulate the parameter
types. The following conversion from input parameter to data
member or variable are possible:

|Input parameter   |    \c T&      |    \c T*      |\c CountedPtr<T>|
|------------------|---------------|---------------|----------------|
|To: \c T&         | Shared. O(1)  | Shared. O(1)  |                |
|To: \c T*         | Shared. O(1)  | Shared. O(1)  |                |
|To:\c CountedPtr<T>|              |               | Shared. O(1)   |
|To:\c CowPtr<T>   |               |               | Shared. O(1)   |


@note The usage of \c Alias<T> instead of \c T \c & or \c T \c *
in parameters is \b recommended when the lifetime of the
parameter must exceed the lifetime of the called
method/function/constructor (often the case in constructor or
init methods). The usage of \c T \c & or \c T \c * instead of \c
Alias<T> is \b recommended when the lifetime of the parameter is
not required to exceed the lifetime of the called
method/function/constructor (often the case in standard methods,
where the parameter is only used at some point, but not
referenced after in some data member).



@tparam T is any type.

@see ConstAlias
@see Clone

It can be used as follows. Consider this simple example where
class \e A is a big object.

@code
const int N = 10000;
struct A { ...
  int table[N];
};

// When looking at the signature of B1's constructor, there is an
// ambiguity on the role of parameter a and its life-time. Here
// a's lifetime should be longer than the construction. Generally
// the ambiguity is removed by adding comments or, for the
// experienced developper, by looking at other parts of the code.

// Only aliasing, but for a long lifetime.
struct B1 {
  B1( A & a ) // ambiguous, cost is O(1) here and lifetime of a should exceed constructor.
  : myA( a ) {}
...
  A & myA;
};
@endcode

Sometimes it is very important that the developper that uses
the library is conscious that an object, say \a b, may require
that an instance \a a given as parameter should have a lifetime
longer than \a b itself (case for an instance of \a B1
above). Classes \ref Clone, \ref Alias, \ref ConstAlias exist for these
reasons. The class above may be rewritten as follows.

@code
// Aliasing for a long lifetime is visible.
struct B1_v2_1 {
  B1_v2_1( Alias<A> a ) // not ambiguous, cost is O(1) here and lifetime of a should be long enough
  : myA( a ) {}
...
  A & myA; 
};

// Aliasing for a long lifetime is visible.
struct B1_v2_2 {
  B1_v2_2( Alias<A> a ) // not ambiguous, cost is O(1) here and lifetime of a should be long enough
  : myA( &a ) {}
...
  A* myA;
};

// Aliasing for a long lifetime is visible.
struct B1_v2_3 {
  B1_v2_3( Alias<A> a ) // not ambiguous, cost is O(1) here and lifetime of a should be long enough, requires typeid(a) == CountedPtr<A> because of member.
  : myA( &a ) {}
...
  CountedPtr<A> myA;
};

...
A a1;
CountedPtr<A> cptr_a1( new A( a1 ) );
B1 ( a1 ); // not duplicated
B1_v2_1 ( a1 ); // not duplicated
B1_v2_2 ( a1 ); // not duplicated
B1_v2_3 ( cptr_a1 ); // not duplicated
@endcode

@note The user should not use Alias<T> instead of T* for data
members. It works in most cases, but there are some subtle
differences between the two behaviors.

@note Alias have a default copy constructor, so as to let the
user forward an Alias<T> parameter.
   */
  template <typename T>
  class Alias
  {

    // ----------------------- Internal classes ------------------------------
  protected:

    /// Internal class that allows to distinguish the different types of parameters.
    enum Parameter { CONST_LEFT_VALUE_REF, LEFT_VALUE_REF, PTR, CONST_PTR, 
		     COW_PTR, COUNTED_PTR, RIGHT_VALUE_REF, CLONE_IS_ERROR };

    // ----------------------- Standard services ------------------------------
  public:

    /**
       Destructor. Does nothing.
     */
    inline ~Alias() {}

    /**
       Constructor from const reference to an instance of T. Invalid.
    */
    inline Alias( const T& ) : myParam( CONST_LEFT_VALUE_REF ), myPtr( 0 )
    { ASSERT(( false && "[Alias::Alias( const T& )] Aliasing a const-ref is an error. Consider ConstAlias instead." )); }

    /**
       Constructor from const pointer to an instance of T. Invalid.
    */
    inline Alias( const T* ) : myParam( CONST_PTR ), myPtr( 0 )
    { ASSERT(( false && "[Alias::Alias( const T& )] Aliasing a const-ptr is an error. Consider ConstAlias instead." )); }

    /**
       Constructor from a reference to an instance of T. The object is pointed in
       'this'.
       @param t any reference to an object of type T.
    */
    inline Alias( T& t ) 
      : myParam( LEFT_VALUE_REF ), myPtr( static_cast<const void*>( &t ) ) {}

    /**
       Constructor from a pointer to an instance of T. The object is pointed in
       'this'.
       @param t any pointer to an object of type T.
    */
    inline Alias( T* t ) 
      : myParam( PTR ), myPtr( static_cast<const void*>( t ) ) {}
    
    /**
       Constructor from a const reference to a copy-on-write pointer on T. Invalid.
    */
    inline Alias( const CowPtr<T>& ) 
      : myParam( COW_PTR ), myPtr( 0 )
    { ASSERT(( false && "[Alias::Alias( const CowPtr<T>& )] Aliasing a const-cow ptr is an error. Consider ConstAlias instead." )); }

    /**
       Constructor from a const reference to a shared pointer on T. The object is pointed in
       'this'.
       @param t a const-reference to any shared pointer to an object of type T.
    */
    inline Alias( const CountedPtr<T>& t ) 
      : myParam( COUNTED_PTR ), myPtr( static_cast<const void*>( &t ) ) {}

#ifdef CPP11_RREF_MOVE
    /**
       Constructor from right-reference value. Invalid.
    */
    inline Alias( T&& ) : myParam( RIGHT_VALUE_REF ), myPtr( 0 )
    { ASSERT(( false && "[Alias::Alias( T&& )] Aliasing a rvalue ref has no meaning. Consider Clone instead." )); }
#endif // CPP11_RREF_MOVE

    /**
       Cast operator to a T reference. The object is never
       duplicated. Allowed input parameters are:
       - T& -> T&                       // no duplication
       - T* -> T&                       // no duplication, exception if null
    */
    inline operator T&() const {
      switch( myParam ) {
      case LEFT_VALUE_REF:
      case PTR:
	return *( const_cast< T* >( static_cast< const T* >( myPtr ) ) );
      default: ASSERT( false && "[Alias::operator T&() const] Invalid cast for given type. Consider passing a left-value reference or a pointer as a parameter." );
        return *( const_cast< T* >( static_cast< const T* >( myPtr ) ) );
      }
    }

    /**
       Cast operator to a T pointer. The object is never
       duplicated. Allowed input parameters are:
     - T& -> T*                       // no duplication
     - T* -> T*                       // no duplication
    */
    inline T* operator&() const {
      switch( myParam ) {
      case LEFT_VALUE_REF: 
      case PTR:
	return const_cast< T* >( static_cast< const T* >( myPtr ) );
      default: ASSERT( false && "[T* Alias::operator&() const] Invalid address operator for given type. Consider passing a left-value reference or a pointer as a parameter." );
        return const_cast< T* >( static_cast< const T* >( myPtr ) );
      }
    }

    /**
       Cast operator to a shared pointer. The object is never
       duplicated. It may be lazily duplicated if casted to a
       CowPtr<T> and the writed. Allowed input parameters are:

       - CountedPtr<T> -> CountedPtr<T> // shared
       - CountedPtr<T> -> CowPtr<T>     // shared (not logical, but not preventable).
    */
    inline operator CountedPtr<T>() const {
      switch( myParam ) {
      case COUNTED_PTR:
	return CountedPtr<T>( *( const_cast< CountedPtr<T>* >( static_cast< const CountedPtr<T>* >( myPtr ) ) ) );
      default: ASSERT( false && "[Alias::operator CountedPtr<T>() const] Invalid cast for given type. Consider passing a CountedPtr as a parameter." );
        return CountedPtr<T>( 0 );
      }
    }

    // ------------------------- Private Datas --------------------------------
  private:
    /// Characterizes the type of the input parameter at clone instanciation.
    const Parameter myParam;
    /// Stores the address of the input parameter for further use.
    const void* const myPtr;
    

    // ------------------------- Internals ------------------------------------
  private:

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden (otherwise the user might be tempted to use it as a member).
     */
    Alias & operator= ( const Alias & other );

  }; // end of class Alias

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Alias_h

#undef Alias_RECURSES
#endif // else defined(Alias_RECURSES)
