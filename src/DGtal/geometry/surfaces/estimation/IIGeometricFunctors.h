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
 * @file IIGeometricFunctors.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/05/14
 *
 * Header file for module IIGeometricFunctors.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(IIGeometricFunctors_RECURSES)
#error Recursive header files inclusion detected in IIGeometricFunctors.h
#else // defined(IIGeometricFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IIGeometricFunctors_RECURSES

#if !defined IIGeometricFunctors_h
/** Prevents repeated inclusion of headers. */
#define IIGeometricFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace IIGeometricFunctors 
  {

    /////////////////////////////////////////////////////////////////////////////
    // template class IINormalDirectionFunctor
    /**
    * Description of template class 'IINormalDirectionFunctor' <p>
    * \brief Aim: A functor Matrix -> RealVector that returns the outer normal
    * direction by diagonalizing the given covariance matrix.
    *
    * @tparam TSpace a model of CSpace, for instance SpaceND.
    */
    template  <typename TSpace>
    class IINormalDirectionFunctor
    {
      // ----------------------- Standard services ------------------------------
    public:
      typedef IINormalDirectionFunctor<TSpace> Self;
      typedef TSpace Space;
      typedef typename Space::RealVector RealVector;
      typedef typename RealVector::Component Component;
      typedef SimpleMatrix<Component,Space::dimension,Space::dimension> Matrix;
      typedef Matrix Argument;
      typedef RealVector Quantity;
      typedef Quantity Value;

      /// Default constructor.
      IINormalDirectionFunctor() {}
      /// Copy constructor. Nothing to do.
      IINormalDirectionFunctor( const Self& /* other */ ) {}
      /// Assignment. Nothing to do.
      /// @return itself
      Self& operator=( const Self& /* other */ ) { return *this; }
      /**
      * Apply operator.
      * @param arg any symmetric positive matrix (covariance matrix
      *
      * @return the normal direction for the II covariance matrix,
      * which is the eigenvector associated with the smallest
      * eigenvalue.
      */
      Value operator()( const Argument& arg ) const
      {
        EigenDecomposition<Space::dimension, Component>
          ::getEigenDecomposition( arg, eigenVectors, eigenValues );
        return eigenVectors.column( 0 ); // normal vector is associated to smallest eigenvalue.      
      }
      /// A data member only used for temporary calculations.
      mutable Matrix eigenVectors;
      /// A data member only used for temporary calculations.
      mutable RealVector eigenValues;
    }; // end of class IINormalDirectionFunctor

  } // namespace IIGeometricFunctors 

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IIGeometricFunctors_h

#undef IIGeometricFunctors_RECURSES
#endif // else defined(IIGeometricFunctors_RECURSES)
