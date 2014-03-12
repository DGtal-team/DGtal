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
 * @file CLinearAlgebra.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/12
 *
 * Header file for concept CLinearAlgebra.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLinearAlgebra_RECURSES)
#error Recursive header files inclusion detected in CLinearAlgebra.h
#else // defined(CLinearAlgebra_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLinearAlgebra_RECURSES

#if !defined CLinearAlgebra_h
/** Prevents repeated inclusion of headers. */
#define CLinearAlgebra_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CLinearAlgebra
  /**
     Description of \b concept '\b CLinearAlgebra' <p>
     @ingroup Concepts
     @brief Aim:

     ### Refinement of

     ### Associated types :

     ### Notation
     - \e X : A type that is a model of CLinearAlgebra
     - \e x, \e y : object of type X

     ### Definitions

     ### Valid expressions and semantics

     | Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
     |-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
     |       |            |                   |               |              |           |                |            |

     ### Invariants

     ### Models

     A dummy model (for concept checking) is CCLinearAlgebraArchetype.

     ### Notes

     @tparam T the type that should be a model of CLinearAlgebra.
  */
  template <typename T>
  struct CLinearAlgebra
  // Use derivation for coarser concepts, like
  // : CoarserConcept<T>
  // Think to boost::CopyConstructible<T>, boost::DefaultConstructible<T>, ...
  // http://www.boost.org/doc/libs/1_49_0/libs/concept_check/reference.htm
  {
    // ----------------------- Concept checks ------------------------------
  public:

    typedef typename T::Index Index;
    typedef typename T::Scalar Scalar;
    typedef typename T::Vector Vector;
    typedef typename T::Matrix Matrix;
    typedef typename T::Solver Solver;

    BOOST_CONCEPT_ASSERT(( CVector<Vector> ));
    BOOST_CONCEPT_ASSERT(( CMatrix<Matrix> ));
    BOOST_CONCEPT_ASSERT(( boost::DefaultConstructible<Solver> ));


    BOOST_CONCEPT_USAGE( CLinearAlgebra )
    {
      // Static members of type A can be tested with
      //ConceptUtils::sameType( myA, T::staticMember );
      // non-const method dummy should take parameter myA of type A and return
      // something of type B
      ConceptUtils::sameType( myVector, mySolver.compute(myMatrix).solve(myVector ) );
      // look at CInteger.h for testing tags.
      // check const methods.
      checkConstConstraints();
    }
    void checkConstConstraints() const
    {
      // const method dummyConst should take parameter myA of type A and return
      // something of type B
      //ConceptUtils::sameType( myB, myX.dummyConst( myA ) );
    }
    // ------------------------- Private Datas --------------------------------
  private:
    T myX; // do not require T to be default constructible.
    Vector myVector;
    Matrix myMatrix;
    Solver mySolver;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CLinearAlgebra

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLinearAlgebra_h

#undef CLinearAlgebra_RECURSES
#endif // else defined(CLinearAlgebra_RECURSES)
