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
 * @file CDifferentiable.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/16
 *
 * Header file for concept CDifferentiable.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDifferentiable_RECURSES)
#error Recursive header files inclusion detected in CDifferentiable.h
#else // defined(CDifferentiable_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDifferentiable_RECURSES

#if !defined CDifferentiable_h
/** Prevents repeated inclusion of headers. */
#define CDifferentiable_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CDifferentiable
/**
Description of \b concept '\b CDifferentiable' <p>
@ingroup Concepts
@brief Aim:
A differentiable object provides a gradient map that associates to each argument some real vector.

### Refinement of

### Associated types :
- \e Argument : the type of each element of the domain
- \e RealVector : the type for representing the gradient values.
- \e GradientMap : functor \e Argument -> \e RealVector, a model of CUnaryFunctor<T, Argument, RealVector > and boost::CopyConstructible.
- \e gradientMap() const: returns a \e GradientMap.

### Notation
 - \e X : A type that is a model of CDifferentiable
 - \e x : object of type X

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Gets the gradient map |\e x.gradientMap()| |\e GradientMap |          | returns the gradient functor | | |      |

### Invariants

### Models

### Notes

@tparam T the type that should be a model of CDifferentiable.
 */
template <typename T>
struct CDifferentiable
{
    // ----------------------- Concept checks ------------------------------
public:
  typedef typename T::Argument Argument;
  typedef typename T::RealVector RealVector;
  typedef typename T::GradientMap GradientMap;
  BOOST_CONCEPT_ASSERT(( boost::CopyConstructible< GradientMap > ));
  BOOST_CONCEPT_ASSERT(( CUnaryFunctor< GradientMap, Argument, RealVector > ));
  BOOST_CONCEPT_USAGE( CDifferentiable )
  {
    checkConstConstraints();
  }
  void checkConstConstraints() const
    {
      ConceptUtils::sameType( myGMap, myX.gradientMap() );
    }
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  GradientMap myGMap;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CDifferentiable

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDifferentiable_h

#undef CDifferentiable_RECURSES
#endif // else defined(CDifferentiable_RECURSES)
