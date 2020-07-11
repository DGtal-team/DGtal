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
 * @file SurfaceMeshHelper.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/07/11
 *
 * Header file for module SurfaceMeshHelper.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SurfaceMeshHelper_RECURSES)
#error Recursive header files inclusion detected in SurfaceMeshHelper.h
#else // defined(SurfaceMeshHelper_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SurfaceMeshHelper_RECURSES

#if !defined SurfaceMeshHelper_h
/** Prevents repeated inclusion of headers. */
#define SurfaceMeshHelper_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/SurfaceMesh.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class SurfaceMeshHelper
  /**
     Description of template class 'SurfaceMeshHelper' <p> \brief Aim:
     An helper class for building classical meshes.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct SurfaceMeshHelper
  {
    typedef TRealPoint                              RealPoint;
    typedef TRealVector                             RealVector;
    typedef SurfaceMeshHelper< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );

    typedef DGtal::SurfaceMesh< RealPoint, RealVector > SurfaceMesh;
    typedef typename RealVector::Component          Scalar;
    typedef std::vector<Scalar>                     Scalars;
    typedef std::vector<RealVector>                 RealVectors;
    typedef typename SurfaceMesh::Size           Size;
    typedef typename SurfaceMesh::Index          Index;
    typedef typename SurfaceMesh::Vertices       Vertices;
    typedef typename SurfaceMesh::Faces          Faces;

    enum class Normals { NO_NORMALS, VERTEX_NORMALS, FACE_NORMALS };

    static
    SurfaceMesh makeSphere( Scalar radius, RealPoint center,
                            Size m, Size n, Normals normals );
    
    static
    Scalars sphereMeanCurvatures( Scalar radius, Size m, Size n );
    
    static
    Scalars sphereGaussianCurvatures( Scalar radius, Size m, Size n );

    static
    Scalars sphereFirstPrincipalCurvatures( Scalar radius, Size m, Size n );
    static
    Scalars sphereSecondPrincipalCurvatures( Scalar radius, Size m, Size n );
    static
    RealVectors sphereFirstPrincipalDirections( Scalar radius, Size m, Size n );
    static
    RealVectors sphereSecondPrincipalDirections( Scalar radius, Size m, Size n );

    static
    SurfaceMesh makeLantern( Scalar radius, Scalar height, RealPoint center,
                             Size m, Size n, Normals normals );
    static
    Scalars lanternMeanCurvatures( Scalar radius, Size m, Size n );
    static
    Scalars lanternGaussianCurvatures( Scalar radius, Size m, Size n );
    static
    Scalars lanternFirstPrincipalCurvatures( Scalar radius, Size m, Size n );
    static
    Scalars lanternSecondPrincipalCurvatures( Scalar radius, Size m, Size n );
    static
    RealVectors lanternFirstPrincipalDirections( Scalar radius, Size m, Size n );
    static
    RealVectors lanternSecondPrincipalDirections( Scalar radius, Size m, Size n );

    static
    SurfaceMesh makeTorus( Scalar big_radius, Scalar small_radius, RealPoint center,
                           Size m, Size n, int twist, Normals normals );

    static
    Scalars torusMeanCurvatures( Scalar big_radius, Scalar small_radius, 
				 Size m, Size n, int twist );
    static
    Scalars torusGaussianCurvatures( Scalar big_radius, Scalar small_radius, 
				     Size m, Size n, int twist );
    static
    Scalars torusFirstPrincipalCurvatures( Scalar big_radius, Scalar small_radius, 
					   Size m, Size n, int twist );
    static
    Scalars torusSecondPrincipalCurvatures( Scalar big_radius, Scalar small_radius, 
					    Size m, Size n, int twist );
    static
    RealVectors torusFirstPrincipalDirections( Scalar big_radius, Scalar small_radius, 
					       Size m, Size n, int twist );
    static
    RealVectors torusSecondPrincipalDirections( Scalar big_radius, Scalar small_radius, 
						Size m, Size n, int twist );
    
  };
  
} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "SurfaceMeshHelper.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SurfaceMeshHelper_h

#undef SurfaceMeshHelper_RECURSES
#endif // else defined(SurfaceMeshHelper_RECURSES)

