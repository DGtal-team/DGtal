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
#include <vector>
#include "DGtal/base/CBackInsertable.h"
#include "DGtal/base/CConstSinglePassRange.h"
#include "DGtal/kernel/CInteger.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace concepts
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

One can "visit" irreducible fractions by enumerating the sequence of
its partial quotients. Furthermore, one can push a new quotient at the
end of this fraction to get a new fraction which shares all quotients
except the last one. In this sense, a fraction is a sequence
(container) that can only grow.

# Refinement of
- boost::CopyConstructible, boost::DefaultConstructible, boost::Assignable
- CBackInsertable, CConstSinglePassRange

# Associated types

- \e Integer: the type for representing a numerator or a
  denominator. Must be a model of CInteger.

- \e Quotient: the type for representing partial quotients, i.e. the
  integers that appear in the continued fractions of p/q, and for
  representing the depth of the fraction. Might be the same as Integer
  but may be also smaller, since quotients are generally much smaller
  than the convergent numerators and denominators. Must be a model of
  CInteger since depths may be negative (1/0 is -1).

- \e Value and \e value_type: the type \c std::pair<Quotient,Quotient>, useful
  to create back insertion sequence.

- \e ConstIterator and \e const_iterator: the type for visiting the quotients of the
  fraction in sequence. The value of the iterator has type \e Value.

# Notation
 - \e X : A type that is a model of CPositiveIrreducibleFraction
 - \e x : object of type \e X, which is below some fraction written \f$[u_0, \ldots, u_k]\f$ as a continued fraction
 - \e x1, \e x2, \e y : other objects of type \e X
 - \e p, \e q : object of type \e Integer
 - \e m, \e n1, \e n2 : objects of type \e Quotient
 - \e quots : an object of type \c std::vector<Quotient>
 - \e pair : a object of \c std::pair<Quotient,Quotient>, here (m,k+1)  

# Definitions

# Valid expressions and semantics

| Name          | Expression | Type requirements   | Return type | Precondition     | Semantics | Post condition | Complexity |
|---------------|------------|---------------------|-------------|------------------|-----------|----------------|------------|
| Constructor   | \c Fraction( \e p, \e q )|       | \e X        |     | creates the fraction p'/q', where p'=p/g, q'=q/g, g=gcd(p,q) | | o(\e p+\e q) |
| numerator     | \e x.\c p()|                     | \e Integer  | ! \e x.\c null() | returns the numerator|     | O(1) |
| denominator   | \e x.\c q()|                     | \e Integer  | ! \e x.\c null() | returns the denominator|   | O(1) |
| quotient      | \e x.\c u()|                     | \e Quotient     | ! \e x.\c null() | returns the quotient \f$u_k\f$ | | O(1) |
| depth         | \e x.\c k()|                     | \e Quotient     | ! \e x.\c null() | returns the depth \e k |   | O(1) |
| null test     | \e x.\c null()|                  | \c bool     |                  | returns 'true' if the fraction is null 0/0 (default fraction) | | O(1) |
| even parity   |\e x.\c even()|                   | \c bool     | ! \e x.\c null() | returns 'true' iff the fraction is even, i.e. \e k is even | | O(1) |
| odd parity    |\e x.\c odd()|                    | \c bool     | ! \e x.\c null() | returns 'true' iff the fraction is odd, i.e. \e k is odd | | O(1) |
|               |            |                     |             |                  |           |                |            |
| left descendant| \e x.\c left()|                 | \e X        | ! \e x.\c null() | returns the left descendant of p/q in the Stern-Brocot tree | | O(1) |
| right descendant|\e x.\c right()|                | \e X        | ! \e x.\c null() | returns the right descendant of p/q in the Stern-Brocot tree | | O(1) |
| father        |\e x.\c father()|                 | \e X        | ! \e x.\c null() | returns the father of this fraction, ie \f$[u_0,...,u_k - 1]\f$ | | O(1) |
| m-father      |\e x.\c father(\e m)|             | \e X        | ! \e x.\c null(), \e m>=0 | returns the \e m-father of this fraction, ie \f$[u_0,...,u_{k-1}, m]\f$ | | O( \e m) |
| previousPartial|\e x.\c previousPartial()|       | \e X        | ! \e x.\c null() | returns the previous partial of this fraction, ie \f$[u_0,...,u_{k-1}]\f$ | | O(1) |
| inverse       |\e x.\c inverse()|                | \e X        | ! \e x.\c null() | returns the inverse of this fraction, ie \f$[0,u_0,...,u_k]\f$ if \f$u_0 \neq 0 \f$ or \f$[u_1,...,u_k]\f$ otherwise | | O(1) |
| \e m-th partial |\e x.\c partial(m)|             | \e X        | ! \e x.\c null() | returns the \e m-th partial of this fraction, ie \f$[u_0,...,u_m]\f$ | | O(1) |
| \e m-th reduced |\e x.\c reduced(m)|             | \e X        | ! \e x.\c null() | returns the \e m-th reduced of this fraction, equivalently the \f$k-m\f$ partial, ie \f$[u_0,...,u_{k-m}]\f$ | | O(1) |
| splitting formula |\e x.\c getSplit(\e x1, \e x2)| | \c void   | ! \e x.\c null() | modifies fractions \e x1 and \e x2 such that \f$ x1 \oplus x2 = x \f$| | O(1) |
| Berstel splitting formula |\e x.\c getSplitBerstel(\e x1, \e n1, \e x2, \e n2)| | \c void | ! \e x.\c null() | modifies fractions \e x1 and \e x2 and integers \e n1 and \e n2 such that \f$ (x1)^{n1} \oplus (x2)^{n2}  = x \f$| | O(1) |
| Continued fraction coefficients |\e x.\c getCFrac(\e quots)| | \c void |             | modifies the vector \e quots such that it contains the quotients \f$u_0,u_1,...,u_k \f$| | O(k) |
|               |            |                     |             |                  |           |                |            |
| equality      |\e x.\c equals(\e p, \e q)|       | \c bool     |                  | returns 'true' iff the fraction is equal to \f$ p / q \f$. | | O(1) |
| less than     |\e x.\c lessThan(\e p, \e q)|     | \c bool     |                  | returns 'true' iff the fraction is inferior to \f$ p / q \f$. | | O(1) |
| more than     |\e x.\c moreThan(\e p, \e q)|     | \c bool     |                  | returns 'true' iff the fraction is superior to \f$ p / q \f$. | | O(1) |
| equality ==   |\e x == \e y|                     | \c bool     |                  | returns 'true' iff the fraction is equal to \e y. | | O(1) |
| inequality != |\e x != \e y|                     | \c bool     |                  | returns 'true' iff the fraction is different from \e y. | | O(1) |
| less than <   |\e x <  \e y|                     | \c bool     |                  | returns 'true' iff the fraction is inferior to \e y. | | O(1) |
| more than >   |\e x >  \e y|                     | \c bool     |                  | returns 'true' iff the fraction is superior to \e y. | | O(1) |
|               |            |                     |             |                  |           |                |            |
| Next continued fraction | \e x.pushBack( pair )| |             |                  | transforms this fraction \f$[0,u_0,...,u_k]\f$ into \f$[0,u_0,...,u_k,m]\f$, where \e pair is \f$(m,k+1)\f$ | | O(m) |
| Next continued fraction | \e x.push_back( pair )| |            |                  | transforms this fraction \f$[0,u_0,...,u_k]\f$ into \f$[0,u_0,...,u_k,m]\f$, where \e pair is \f$(m,k+1)\f$ | | O(m) |
|               |            |                     |             |                  |           |                |            |
| Begin visiting quotients | \e x.begin()|         | \e ConstIterator |             | returns a forward iterator on the beginning of the sequence of quotients \f$[u_0,...,u_k]\f$ | | |
| End visiting quotients | \e x.end()|             | \e ConstIterator |             | returns a forward iterator after the end of the sequence of quotients \f$[u_0,...,u_k]\f$ | | |

# Invariants

# Models

- SternBrocot::Fraction, LighterSternBrocot::Fraction
- also LightSternBrocot::Fraction (but do not use).

# Notes

@tparam T the type that should be a model of CPositiveIrreducibleFraction.
*/
template <typename T> 
concept CPositiveIrreducibleFraction = 
  CBackInsertable<T> && 
  CConstSinglePassRange<T> && 
  CInteger<typename T::Integer> && 
  CInteger<typename T::Quotient> &&
  std::same_as<typename T::value_type, std::pair<typename T::Quotient, typename T::Quotient>> &&
  std::same_as<typename T::value_type, typename T::Value> &&
  std::is_constructible_v<T, typename T::Integer, typename T::Integer> &&
  requires(
    T myX, const T cmyX, 
    typename T::Integer myP, 
    typename T::value_type myValue, typename T::Quotient myU, std::vector<typename T::Quotient> myQuots)
  {
      myX.push_back(myValue);
      myX.pushBack(myValue);

      { cmyX.p() } -> std::same_as<typename T::Integer>;
      { cmyX.q() } -> std::same_as<typename T::Integer>;
      { cmyX.u() } -> std::same_as<typename T::Quotient>;
      { cmyX.k() } -> std::same_as<typename T::Quotient>;
      { cmyX.left() } -> std::same_as<T>;
      { cmyX.right() } -> std::same_as<T>;
      { cmyX.null() } -> std::same_as<bool>;
      { cmyX.even() } -> std::same_as<bool>;
      { cmyX.odd()  } -> std::same_as<bool>;
      { cmyX.father() } -> std::same_as<T>;
      { cmyX.father(myU) } -> std::same_as<T>;
      { cmyX.previousPartial() } -> std::same_as<T>;
      { cmyX.inverse() } -> std::same_as<T>;
      { cmyX.partial(myU) } -> std::same_as<T>;
      { cmyX.reduced(myU) } -> std::same_as<T>;

      cmyX.getSplit(myX, myX);
      cmyX.getSplitBerstel(myX, myU, myX, myU);
      cmyX.getCFrac(myQuots);

      { cmyX.equals(myP, myP) } -> std::same_as<bool>;
      { cmyX.lessThan(myP, myP) } -> std::same_as<bool>;
      { cmyX.moreThan(myP, myP) } -> std::same_as<bool>;
      
      // Does not implement <=...: requires std::totally_ordered<T>;
      { myX == myX } -> std::same_as<bool>;
      { myX != myX } -> std::same_as<bool>;
      { myX < myX } -> std::same_as<bool>;
      { myX > myX } -> std::same_as<bool>; 

      { cmyX.begin() } -> std::same_as<typename T::ConstIterator>;
      { cmyX.end() } -> std::same_as<typename T::ConstIterator>;
  };
} // namespace concepts

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CPositiveIrreducibleFraction_h

#undef CPositiveIrreducibleFraction_RECURSES
#endif // else defined(CPositiveIrreducibleFraction_RECURSES)
