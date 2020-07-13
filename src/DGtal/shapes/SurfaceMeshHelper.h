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

    /// Enum type for specifying if the mesh should have after
    /// construction: NO_NORMALS no normals, VERTEX_NORMALS the
    /// normals at each vertex, FACE_NORMALS the normals at each face.
    enum class Normals { NO_NORMALS, VERTEX_NORMALS, FACE_NORMALS };

    //---------------------------------------------------------------------------
  public:
    /// @name Sphere services
    /// @{

    /// Builds a surface mesh representing a sphere with \a m latitudes
    /// (poles excluded) and \a n longitudes.
    ///
    /// @param[in] radius the radius of the sphere.
    /// @param[in] center the center of the sphere.
    /// @param[in] m the number of latitudes (poles excepted), minimum is 1.
    /// @param[in] m the number of longitudes, minimum is 3.
    ///
    /// @param[in] normals specifies built-time normals of the output
    /// mesh: NO_NORMALS no normals, VERTEX_NORMALS the normals at
    /// each vertex, FACE_NORMALS the normals at each face.
    ///
    /// @return the corresponding surface mesh.
    static
    SurfaceMesh
    makeSphere( Scalar radius, RealPoint center,
                Size m, Size n, Normals normals );
    
    static
    Scalars
    sphereMeanCurvatures( Scalar radius, Size m, Size n );
    
    static
    Scalars
    sphereGaussianCurvatures( Scalar radius, Size m, Size n );

    static
    Scalars
    sphereFirstPrincipalCurvatures( Scalar radius, Size m, Size n );

    static
    Scalars
    sphereSecondPrincipalCurvatures( Scalar radius, Size m, Size n );

    static
    RealVectors
    sphereFirstPrincipalDirections( Scalar radius, Size m, Size n );

    static
    RealVectors
    sphereSecondPrincipalDirections( Scalar radius, Size m, Size n );

    /// @}
    
    //---------------------------------------------------------------------------
  public:
    /// @name Schwarz lantern services
    /// @{

    /// Builds a surface mesh representing a Schwarz lantern with \a m
    /// latitudes and \a n longitudes.
    ///
    /// @param[in] radius the radius of the lantern.
    /// @param[in] height the height of the lantern.
    /// @param[in] center the center of the lantern.
    /// @param[in] m the number of latitudes, minimum is 2.
    /// @param[in] m the number of longitudes, minimum is 3.
    ///
    /// @param[in] normals specifies built-time normals of the output
    /// mesh: NO_NORMALS no normals, VERTEX_NORMALS the normals at
    /// each vertex, FACE_NORMALS the normals at each face.
    ///
    /// @return the corresponding surface mesh.
    static
    SurfaceMesh
    makeLantern( Scalar radius, Scalar height, RealPoint center,
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

    /// @}
    
    //---------------------------------------------------------------------------
  public:
    /// @name Torus services
    /// @{
    
    /// Builds a surface mesh representing a torus with \a m
    /// latitudes and \a n longitudes.
    ///
    /// @param[in] big_radius the big radius of the torus.
    /// @param[in] small_radius the small radius of the torus.
    /// @param[in] center the center of the torus.
    /// @param[in] m the number of latitudes, minimum is 3.
    /// @param[in] m the number of longitudes, minimum is 3.
    ///
    /// @param[in] twist the integer shift when visiting a whole
    /// parallel: 0 is a natural torus parameterization, +n or -n
    /// makes the visitor arriving on another parallel after one turn.
    ///
    /// @param[in] normals specifies built-time normals of the output
    /// mesh: NO_NORMALS no normals, VERTEX_NORMALS the normals at
    /// each vertex, FACE_NORMALS the normals at each face.
    ///
    /// @return the corresponding surface mesh.
    static
    SurfaceMesh
    makeTorus( Scalar big_radius, Scalar small_radius, RealPoint center,
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
    /// @}
    
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

