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
 * @file CLocalEstimatorFromSurfelFunctor.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/30
 *
 * Header file for concept CLocalEstimatorFromSurfelFunctor.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CLocalEstimatorFromSurfelFunctor_RECURSES)
#error Recursive header files inclusion detected in CLocalEstimatorFromSurfelFunctor.h
#else // defined(CLocalEstimatorFromSurfelFunctor_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CLocalEstimatorFromSurfelFunctor_RECURSES

#if !defined CLocalEstimatorFromSurfelFunctor_h
/** Prevents repeated inclusion of headers. */
#define CLocalEstimatorFromSurfelFunctor_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CSCellEmbedder.h"
#include "DGtal/base/CQuantity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// class CLocalEstimatorFromSurfelFunctor
/**
Description of \b concept '\b CLocalEstimatorFromSurfelFunctor' <p>
@ingroup Concepts
@brief Aim: this concept describes functors on digtal surface surfel
which can be used to define local estimator using the adapter
LocalEstimatorFromSurfelFunctorAdapter.

### Refinement of

### Associated types :

 - @e Surfel: a type of Surfel
 - @e Embedder: a model of CSCellEmbedder
 - @e Quantity: a return type for the estimator (model of CQuantity)

### Notation
 - \e X : A type that is a model of CLocalEstimatorFromSurfelFunctor
 - \e x, \e y : object of type X
 - \e aSurfel : object of type @e Surfel

### Definitions

### Valid expressions and semantics

| Name  | Expression | Type requirements | Return type   | Precondition | Semantics | Post condition | Complexity |
|-------|------------|-------------------|---------------|--------------|-----------|----------------|------------|
| Constructor from Embedder      | X(anEmbedder)  | anEmbedder of type @e SCellEmbedder   |               |              | construct an instance from an SCEll embedder  |                |            |
| push a surfel      | x.pushSurfel(aSurfel)  | aSurfel of type @e Surfel   |       void        |              | push a surfel to the estimator  |                |            |
| evaluate the estimator      | v = x.eval(h)  | h of type  const double |  v of type @e Quantity  |              | evaluate to the estimator  |                |            |
| reset       |  x.eval()  |  |  void  |              | reset the estimator  |                |            |

### Invariants

### Models

  DummyEstimatorFromSurfels

### Notes

@tparam T the type that should be a model of CLocalEstimatorFromSurfelFunctor.
 */
template <typename T>
struct CLocalEstimatorFromSurfelFunctor
{
    // ----------------------- Concept checks ------------------------------
public:
   
  typedef typename T::Quantity Quantity;
  typedef typename T::SCellEmbedder SCellEmbedder;
  typedef typename T::Surfel Surfel;

  BOOST_CONCEPT_ASSERT(( CQuantity< Quantity > ));
  BOOST_CONCEPT_ASSERT(( CSCellEmbedder< SCellEmbedder > ));

  // 2. then check the presence of data members, operators and methods with
  BOOST_CONCEPT_USAGE( CLocalEstimatorFromSurfelFunctor )
  {
    ConceptUtils::sameType( myQ, myX.eval( myH ) );
    myX.reset( );
    myX.pushSurfel( myA );
    
  }
  
  // ------------------------- Private Datas --------------------------------
private:
  T myX; // do not require T to be default constructible.
  Surfel myA;
  Quantity myQ;
  SCellEmbedder myEmb;
  double myH;

    // ------------------------- Internals ------------------------------------
private:

}; // end of concept CLocalEstimatorFromSurfelFunctor

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CLocalEstimatorFromSurfelFunctor_h

#undef CLocalEstimatorFromSurfelFunctor_RECURSES
#endif // else defined(CLocalEstimatorFromSurfelFunctor_RECURSES)
