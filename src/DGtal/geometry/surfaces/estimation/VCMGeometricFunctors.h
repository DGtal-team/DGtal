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
 * @file VCMGeometricFunctors.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/02/14
 *
 * Header file for module VCMGeometricFunctors.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VCMGeometricFunctors_RECURSES)
#error Recursive header files inclusion detected in VCMGeometricFunctors.h
#else // defined(VCMGeometricFunctors_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VCMGeometricFunctors_RECURSES

#if !defined VCMGeometricFunctors_h
/** Prevents repeated inclusion of headers. */
#define VCMGeometricFunctors_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace VCMGeometricFunctors 
  {


    /**
     * Description of template class 'VCMNormalVectorFunctor' <p> \brief
     * Aim: A functor Surfel -> Quantity that returns the outer normal
     * vector at given point.
     *
     * @tparam TVCMOnDigitalSurface any concrete type of VoronoiCovarianceMeasureOnDigitalSurface
     */
    template <typename TVCMOnDigitalSurface>
    struct VCMNormalVectorFunctor {
      typedef TVCMOnDigitalSurface VCMOnDigitalSurface;
      typedef typename VCMOnDigitalSurface::KSpace KSpace;
      typedef typename VCMOnDigitalSurface::Surfel Surfel;
      typedef typename VCMOnDigitalSurface::VectorN RealVector;
      typedef typename RealVector::Component Scalar;
      typedef Surfel Argument;
      typedef RealVector Quantity;
      
      /**
       * Constructor. A VCM may also be attached at construction.
       *
       * @param aVCMOnSurface the VCM on surface that stores all the
       * information. The alias can be secured if some counted
       * pointer is handed.
       */
      VCMNormalVectorFunctor( ConstAlias<VCMOnDigitalSurface> aVCMOnDigitalSurface = 0 ) 
        : myVCMOnDigitalSurface( aVCMOnDigitalSurface ) {}
      
      /**
       * Attach a VCM on a digital surface.
       *
       * @param aVCMOnDigitalSurface on surface that stores all the
       * information. The alias can be secured if some counted
       * pointer is handed.
       */
      void attach( ConstAlias<VCMOnDigitalSurface> aVCMOnDigitalSurface )
      {
        myVCMOnDigitalSurface = aVCMOnDigitalSurface;
      }

      /**
         Map operator Surfel -> RealVector giving the normal vector estimated by the VCM object.
         @param s any surfel of the shape.
         @return the normal at point p (as the normalized gradient).
      */
      Quantity operator()( const Surfel & s ) const
      {
        typedef typename VCMOnDigitalSurface::Surfel2Normals Surfel2Normals;
        ASSERT( myVCMOnDigitalSurface != 0 );
        typename Surfel2Normals::const_iterator itSN = myVCMOnDigitalSurface->mapSurfel2Normals().find( s );
        ASSERT( itSN != myVCMOnDigitalSurface->mapSurfel2Normals().end() );
        return - itSN->second.vcmNormal;
      }

    private:
      /// The shape of interest.
      CountedConstPtrOrConstPtr<VCMOnDigitalSurface> myVCMOnDigitalSurface;
    };
    
  } // namespace VCMGeometricFunctors 
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VCMGeometricFunctors_h

#undef VCMGeometricFunctors_RECURSES
#endif // else defined(VCMGeometricFunctors_RECURSES)
