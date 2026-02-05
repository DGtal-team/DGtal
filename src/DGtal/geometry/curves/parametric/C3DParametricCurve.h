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
 * @file C3DParametricCurve.h
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2014/10/01
 *
 * Header file for concept ParametricCurve.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(C3DParametricCurve_RECURSES)
#error Recursive header files inclusion detected in ParametricCurve.h
#else // defined(C3DParametricCurve_RECURSES)
/** Prevents recursive inclusion of headers. */
#define C3DParametricCurve_RECURSES

#if !defined C3DParametricCurve_h
/** Prevents repeated inclusion of headers. */
#define C3DParametricCurve_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
namespace concepts
{

/////////////////////////////////////////////////////////////////////////////
// class 3DParametricCurve
/**
Description of \b concept '\b 3DParametricCurve' <p>
@ingroup Concepts
@brief Aim:

# Refinement of

# Associated types

# Notation
 - \e X : A type that is a model of 3DParametricCurve
 - \e x, \e y : object of type X

# Definitions

# Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

# Invariants

# Models

   A dummy model (for concept checking) is C3DParametricCurveArchetype.

# Notes

@tparam T the type that should be a model of 3DParametricCurve.
 */
template <typename T>
struct C3DParametricCurve
{
    // ----------------------- Concept checks ------------------------------
public:
    typedef typename T::Space Space;
    typedef typename T::RealPoint RealPoint;
    typedef typename T::Point Point;
    BOOST_CONCEPT_ASSERT(( concepts::CSpace <Space> ));
    BOOST_STATIC_ASSERT(( Space::dimension == 3 ));
    // check the presence of data members, operators and methods with
    BOOST_CONCEPT_USAGE( C3DParametricCurve )
    {
        // check const methods.
        checkConstConstraints();
    }
    virtual void checkConstConstraints() const
    {
        ConceptUtils::sameType( x , self.xp ( 0.1f ) );
        ConceptUtils::sameType( x , self.x ( 0.1f ) );
    }
    // ------------------------- Private Data --------------------------------
protected:
    T self;
    RealPoint x;
    Point y;
    double c;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept 3DParametricCurve



/////////////////////////////////////////////////////////////////////////////
// class 3DParametricCurveDecorator
/**
Description of \b concept '\b 3DParametricCurveDecorator' <p>
@ingroup Concepts
@brief Aim:

# Refinement of 3DParametricCurve

# Associated types

# Notation
 - \e X : A type that is a model of DParametricCurveDecorator
 - \e x, \e y : object of type X

# Definitions

# Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
|       |            |                   |               |              |           |                |            |

# Invariants

# Models

   A dummy model (for concept checking) is C3DParametricCurveDecoratorArchtype.

# Notes

@tparam T the type that should be a model of 3DParametricCurve.
 */
template <typename T>
struct C3DParametricCurveDecorator : public C3DParametricCurve< T >
{
  // ----------------------- Concept checks ------------------------------
public:
  // check the presence of data members, operators and methods with
  BOOST_CONCEPT_USAGE( C3DParametricCurveDecorator )
  {
    // check const methods.
    checkConstConstraints();
  }
  void checkConstConstraints() const
  {
    ConceptUtils::sameType( curve, self.curve );
  }
  // ------------------------- Private Data --------------------------------
protected:
  T self;
  typename T::TypeCurve curve;
  // ------------------------- Private Data --------------------------------
  // ------------------------- Internals ------------------------------------
}; // end of concept 3DParametricCurve

   } // namespace concepts
} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ParametricCurve_h

#undef C3DParametricCurve_RECURSES
#endif // else defined(ParametricCurve_RECURSES)
