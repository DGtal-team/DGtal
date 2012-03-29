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
 * @file CPositiveIrreducibleFraction.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/29
 *
 * Header file for concept CPositiveIrreducibleFraction.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CPositiveIrreducibleFraction_RECURSES)
#error Recursive header files inclusion detected in CPositiveIrreducibleFraction.h
#else // defined(CPositiveIrreducibleFraction_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CPositiveIrreducibleFraction_RECURSES

#if !defined CPositiveIrreducibleFraction_h
/** Prevents repeated inclusion of headers. */
#define CPositiveIrreducibleFraction_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CPositiveIrreducibleFraction
/**
Description of \b concept '\b CPositiveIrreducibleFraction' <p>
@ingroup Concepts

@brief Aim: Defines positive irreducible fractions, i.e. fraction p/q,
p and q non-negative integers, with gcd(p,q)=1.

Irreducible fractions are nicely represented with the Stern-Brocot
tree. Furthermore, the development of a fraction p/q into its simple
continued fraction with quotients \f$[u_0, \ldots, u_k]\f$ has
one-to-one correspondence with the position of the fraction in the
Stern-Brocot tree.

### Refinement of
- boost::CopyConstructible, boost::DefaultConstructible, boost_Assignable

### Associated types :

- Integer: the type for representing a numerator or a denominator.

- Size: the type for representing partial quotients, i.e. the integers
  that appear in the continued fractions of p/q. Might be the same as
  Integer but may be also smaller, since quotients are generally much
  smaller than the convergent numerators and denominators.

### Notation
 - \e X : A type that is a model of CPositiveIrreducibleFraction
 - \e x : object of type X, which is below some fraction written \f$[u_0, \ldots, u_k]\f$ as a continued fraction
 - \e p, \e q : object of type X::Integer
 - \e v, \e m : object of type X::Size

### Definitions

### Valid expressions and semantics

| Name          | Expression | Type requirements   | Return type | Precondition     | Semantics | Post condition | Complexity |
|---------------|------------|---------------------|-------------|------------------|-----------|----------------|------------|
| Constructor   | \c Fraction( \e p, \e q )|       | \e X        |     | creates the fraction p'/q', where p'=p/g, q'=q/g, g=gcd(p,q) | | o(\e p+\e q) |
| numerator     | \e x.\c p()|                     | \e Integer  | ! \e x.\c null() | returns the numerator|     | O(1) |
| denominator   | \e x.\c q()|                     | \e Integer  | ! \e x.\c null() | returns the denominator|   | O(1) |
| quotient      | \e x.\c u()|                     | \e Size     | ! \e x.\c null() | returns the quotient \f$u_k\f$ | | O(1) |
| depth         | \e x.\c k()|                     | \e Size     | ! \e x.\c null() | returns the depth \e k |   | O(1) |
| null test     | \e x.\c null()|                  | \c bool     |                  | returns 'true' if the fraction is null 0/0 (default fraction) | | O(1) |
| left descendant| \e x.\c left()|                 | \e X        | ! \e x.\c null() | returns the left descendant of p/q in the Stern-Brocot tree | | O(1) |
| right descendant|\e x.\c right()|                | \e X        | ! \e x.\c null() | returns the right descendant of p/q in the Stern-Brocot tree | | O(1) |
| even parity   |\e x.\c even()|                   | \c bool     | ! \e x.\c null() | returns 'true' iff the fraction is even, i.e. \e k is even | | O(1) |
| odd parity    |\e x.\c odd()|                    | \c bool     | ! \e x.\c null() | returns 'true' iff the fraction is odd, i.e. \e k is odd | | O(1) |
| father        |\e x.\c father()|                 | \e X        | ! \e x.\c null() | returns the father of this fraction, ie \f$[u_0,...,u_k - 1]\f$ | | O(1) |
| m-father      |\e x.\c father(\e m)|             | \e X        | ! \e x.\c null(), \e m>=0 | returns the \e m-father of this fraction, ie \f$[u_0,...,u_{k-1}, m]\f$ | | O( \e m) |

### Invariants

### Models

  - SternBrocot::Fraction, LighterSternBrocot::Fraction

### Notes

@tparam T the type that should be a model of CPositiveIrreducibleFraction.
 */
template <typename T>
struct CPositiveIrreducibleFraction 
  : boost::CopyConstructible<T>, 
  boost::DefaultConstructible<T>, 
  boost::Assignable<T>
{
    // ----------------------- Concept checks ------------------------------
public:
  typedef typename T::Integer Integer;
  typedef typename T::Size Size;
  BOOST_CONCEPT_ASSERT(( CInteger< Integer > ));
  BOOST_CONCEPT_ASSERT(( CInteger< Size > ));
  BOOST_CONCEPT_USAGE( CPositiveIrreducibleFraction )
  {
    ConceptUtils::sameType( myX, T( myP, myQ ) );
    checkConstConstraints();
  }
  void checkConstConstraints() const
  {
    ConceptUtils::sameType( myP, myX.p() );
    ConceptUtils::sameType( myQ, myX.q() );
    ConceptUtils::sameType( myU, myX.u() );
    ConceptUtils::sameType( myU, myX.k() );
    ConceptUtils::sameType( myBool, myX.null() );
    ConceptUtils::sameType( myX, myX.left() );
    ConceptUtils::sameType( myX, myX.right() );
    ConceptUtils::sameType( myBool, myX.even() );
    ConceptUtils::sameType( myBool, myX.odd() );
    ConceptUtils::sameType( myX, myX.father() );
    ConceptUtils::sameType( myX, myX.father( myU ) );
  }
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  Integer myP;
  Integer myQ;
  Size myU;
  bool myBool;
  // ------------------------- Internals ------------------------------------
private:

}; // end of concept CPositiveIrreducibleFraction

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPositiveIrreducibleFraction_h

#undef CPositiveIrreducibleFraction_RECURSES
#endif // else defined(CPositiveIrreducibleFraction_RECURSES)
