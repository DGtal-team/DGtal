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
 * @file CSurfaceLocalGeometricEstimator.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/21
 *
 * Header file for concept CSurfaceLocalGeometricEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CSurfaceLocalGeometricEstimator_RECURSES)
#error Recursive header files inclusion detected in CSurfaceLocalGeometricEstimator.h
#else // defined(CSurfaceLocalGeometricEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CSurfaceLocalGeometricEstimator_RECURSES

#if !defined CSurfaceLocalGeometricEstimator_h
/** Prevents repeated inclusion of headers. */
#define CSurfaceLocalGeometricEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CQuantity.h"
#include <boost/iterator/iterator_archetypes.hpp>
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CSurfaceLocalGeometricEstimator
  /**
     Description of \b concept '\b CSurfaceLocalGeometricEstimator' <p>
     @ingroup Concepts
     @brief Aim: This concept describes an object that can process 
     a range over some (abstract) surface so as to return one estimated quantity for each element 
     of the range (or a given subrange). 

     ### Refinement of 

     ### Associated types :
     - Surfel : the type of elements of the surface
     - Quantity : the type of the (geometric) quantity that is estimated.

     ### Notation
     - \e X : A type that is a model of CSurfaceLocalGeometricEstimator
     - \e x : object of type X
     - \e h : double
     - \e itb, ite, it : instances of a model of forward iterators having Surfel as value type.
     - \e ito : an instance of a model of output iterator having Quantity as value type
 
     ### Definitions

     ### Valid expressions and semantics

     | Name           | Expression                    |   | Return type                  | Precondition | Semantics                                 |   | Complexity      |
     |----------------|-------------------------------|---|------------------------------|--------------|-------------------------------------------|---|-----------------|
     | Initialization | x.init( h, itb, ite )         |   | void                         | h > 0        | Grid step and range initialization        |   | constant        |
     | Evaluation     | x.eval( it )                  |   | Quantity                     |              | Estimation of the quantity at \e it       |   | model dependant |
     | Evaluation     | ito = x.eval( itb, ite, ito ) |   | a model of output iterator   |              | Estimation for each element of [itb, ite) |   | model dependant |

     ### Invariants

     ### Models

     - TrueDigitalSurfaceLocalEstimator, VCMDigitalSurfaceNormalEstimator.

     ### Notes

     @tparam T the type that should be a model of CSurfaceLocalGeometricEstimator.
  */
  template <typename T>
  struct CSurfaceLocalGeometricEstimator
  {

    // ----------------------- Concept checks ------------------------------
  public:

    typedef typename T::Quantity Quantity;
    BOOST_CONCEPT_ASSERT(( CQuantity< Quantity > ));
    typedef typename T::Surfel Surfel;

    BOOST_CONCEPT_USAGE( CSurfaceLocalGeometricEstimator )
    {
      //init method
      myX.init( myH, myItb, myIte ); 

      ConceptUtils::sameType( myQ, myX.eval( myItb ) );
      ConceptUtils::sameType( myIto, myX.eval( myItb, myIte, myIto ) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myX;
    
    double myH; 
    Quantity myQ;
    boost::iterator_archetype<Surfel,
			      boost::iterator_archetypes::readable_iterator_t,
			      boost::forward_traversal_tag > myItb, myIte; 
    boost::iterator_archetype<Quantity,
			      boost::iterator_archetypes::writable_iterator_t,
			      boost::incrementable_traversal_tag > myIto; 

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CSurfaceLocalGeometricEstimator

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CSurfaceLocalGeometricEstimator_h

#undef CSurfaceLocalGeometricEstimator_RECURSES
#endif // else defined(CSurfaceLocalGeometricEstimator_RECURSES)
