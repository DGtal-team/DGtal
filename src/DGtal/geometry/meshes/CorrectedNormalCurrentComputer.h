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
 * @file CorrectedNormalCurrentComputer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/18
 *
 * Header file for module CorrectedNormalCurrentComputer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CorrectedNormalCurrentComputer_RECURSES)
#error Recursive header files inclusion detected in CorrectedNormalCurrentComputer.h
#else // defined(CorrectedNormalCurrentComputer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CorrectedNormalCurrentComputer_RECURSES

#if !defined CorrectedNormalCurrentComputer_h
/** Prevents repeated inclusion of headers. */
#define CorrectedNormalCurrentComputer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/math/linalg/EigenDecomposition.h"
#include "DGtal/geometry/meshes/SurfaceMeshMeasure.h"
#include "DGtal/geometry/meshes/CorrectedNormalCurrentFormula.h"
#include "DGtal/shapes/SurfaceMesh.h"


namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class CorrectedNormalCurrentComputer
  /**
     Description of template class 'CorrectedNormalCurrentComputer'
     <p> \brief Aim: Utility class to compute curvature measures
     induced by (1) a corrected normal current defined by a surface
     mesh with prescribed normals and (2) the standard
     Lipschitz-Killing invariant forms of area and curvatures.

     @note By default it tries to compute interpolated corrected
     curvature measures, if the mesh has a normal at each vertex,
     otherwise it computes constant corrected curvature measures.

     @tparam TRealPoint an arbitrary model of RealPoint.
     @tparam TRealVector an arbitrary model of RealVector.
   */
  template < typename TRealPoint, typename TRealVector >
  struct CorrectedNormalCurrentComputer
  {
    typedef TRealPoint                                   RealPoint;
    typedef TRealVector                                  RealVector;
    typedef CorrectedNormalCurrentComputer< RealPoint, RealVector > Self;
    static const Dimension dimension = RealPoint::dimension;
    BOOST_STATIC_ASSERT( ( dimension == 3 ) );
    typedef DGtal::SurfaceMesh< RealPoint, RealVector >  SurfaceMesh;
    typedef typename RealVector::Component               Scalar;
    typedef SimpleMatrix< Scalar, dimension, dimension > RealTensor;
    typedef CorrectedNormalCurrentFormula< RealPoint, RealVector > Formula;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, Scalar >     ScalarMeasure;
    typedef SurfaceMeshMeasure< RealPoint, RealVector, RealTensor > TensorMeasure;
    typedef std::vector< Scalar >                        Scalars;
    typedef std::vector< RealPoint >                     RealPoints;
    typedef std::vector< RealVector >                    RealVectors;
    typedef std::vector< RealTensor >                    RealTensors;
    typedef typename SurfaceMesh::Size                   Size;
    typedef typename SurfaceMesh::Index                  Index;
    typedef typename SurfaceMesh::Vertex                 Vertex;
    typedef typename SurfaceMesh::Face                   Face;


    /// Constructor from mesh.
    ///
    /// @param aMesh any simplified mesh that is referenced in this
    /// object.
    ///
    /// @param unit_u when set to 'true' tells the computer to
    /// normalize corrected normals in curvature formulae. For
    /// instance, for the Gaussian curvature, it induces exact
    /// Gaussian curvature computation with spherical triangles.
    CorrectedNormalCurrentComputer( ConstAlias< SurfaceMesh > aMesh,
                                    bool unit_u = false );

    /// @return the \f$ \mu_0 \f$ corrected curvature measure,
    /// i.e. the area measure.
    ScalarMeasure computeMu0() const;
    /// @return the \f$ \mu_1 \f$ corrected curvature measure,
    /// i.e. twice the mean curvature measure.
    ScalarMeasure computeMu1() const;
    /// @return the \f$ \mu_2 \f$ corrected curvature measure,
    /// i.e. the Gaussian curvature measure.
    ScalarMeasure computeMu2() const;
    /// @return the \f$ \mu^{X,Y} \f$ corrected curvature measure,
    /// i.e. the anisotropic tensor curvature measure.
    TensorMeasure computeMuXY() const;

    //-------------------------------------------------------------------------
  public:
    /// @name Formulas for estimating curvatures from measures
    /// @{

    /// @param mu0 the mu0 measure (i.e. area) of some set
    /// @param mu1 the mu1 measure (i.e. twice the mean curvature measure) of the same set
    /// @return the estimated mean curvature on this set.
    static
    Scalar meanCurvature( Scalar mu0, Scalar mu1 )
    {
      return ( mu0 != 0.0 ) ? mu1 / ( 2.0 * mu0 ) : 0.0;
    }

    /// @param mu0 the mu0 measure (i.e. area) of some set
    /// @param mu2 the mu2 measure (i.e. the Gaussian curvature measure) of the same set
    /// @return the estimated Gaussian curvature on this set.
    static
    Scalar GaussianCurvature( Scalar mu0, Scalar mu2 )
    {
      return ( mu0 != 0.0 ) ? mu2 / mu0 : 0.0;
    }

    /// @param mu0 the mu0 measure (i.e. area) of some set
    /// @param muXY the anisotropic muXY measure (i.e. the second fundamental form measure) of the same set
    /// @param N the normal vector at the location of the set
    ///
    /// @return a tuple (K1,K2,D1,D2) where K1 and K2 are two
    /// principal curvatures (K1<=K2) and D1 and D2 are their
    /// associated principal directions.
    static
    std::tuple< Scalar, Scalar, RealVector, RealVector >
    principalCurvatures( Scalar mu0, RealTensor muXY, const RealVector& N )
    {
      muXY += muXY.transpose();
      muXY *= 0.5;
      const double   coef_N = 1000.0 * mu0;
      // Trick to force orthogonality to normal vector.
      // (see @cite lachaud2020interpolated, section 2)
      for ( int j = 0; j < 3; j++ )
        for ( int k = 0; k < 3; k++ )
          muXY( j, k ) += coef_N * N[ j ] * N[ k ];
      RealTensor V;
      RealVector L;
      EigenDecomposition< 3, double>::getEigenDecomposition( muXY, V, L );
      return std::make_tuple( ( mu0 != 0.0 ) ? -L[ 1 ] / mu0 : 0.0,
                              ( mu0 != 0.0 ) ? -L[ 0 ] / mu0 : 0.0,
                              V.column( 1 ),
                              V.column( 0 ) );
    }
    /// @}

    // ------------------------- Public Data ------------------------------
  public:

    // ------------------------- Protected Data ------------------------------
  protected:

    /// A reference to the mesh over which computations are done.
    const SurfaceMesh& myMesh;
    /// Tells if we should use a normalized unit vector or not for
    /// interpolated curvature measures.
    bool myUnitU;

    // ------------------------- Private Data --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  protected:

    /// @return the \f$ \mu_0 \f$ corrected curvature measure,
    /// i.e. the area measure, when corrected normals are constant per
    /// face.
    /// @pre `! myMesh.faceNormals().empty()`
    ScalarMeasure computeMu0ConstantU() const;
    /// @return the \f$ \mu_1 \f$ corrected curvature measure,
    /// i.e. twice the mean curvature measure, when corrected normals
    /// are constant per face.
    /// @pre `! myMesh.faceNormals().empty()`
    ScalarMeasure computeMu1ConstantU() const;
    /// @return the \f$ \mu_2 \f$ corrected curvature measure,
    /// i.e. the Gaussian curvature measure, when corrected normals
    /// are constant per face.
    /// @pre `! myMesh.faceNormals().empty()`
    ScalarMeasure computeMu2ConstantU() const;
    /// @return the \f$ \mu^{X,Y} \f$ corrected curvature measure,
    /// i.e. the anisotropic tensor curvature measure, when corrected
    /// normals are constant per face.
    /// @pre `! myMesh.faceNormals().empty()`
    TensorMeasure computeMuXYConstantU() const;

    /// @return the \f$ \mu_0 \f$ corrected curvature measure,
    /// i.e. the area measure, when corrected normals are interpolated per
    /// face.
    /// @pre `! myMesh.vertexNormals().empty()`
    ScalarMeasure computeMu0InterpolatedU() const;
    /// @return the \f$ \mu_1 \f$ corrected curvature measure,
    /// i.e. twice the mean curvature measure, when corrected normals
    /// are interpolated per face.
    /// @pre `! myMesh.vertexNormals().empty()`
    ScalarMeasure computeMu1InterpolatedU() const;
    /// @return the \f$ \mu_2 \f$ corrected curvature measure,
    /// i.e. the Gaussian curvature measure, when corrected normals
    /// are interpolated per face.
    /// @pre `! myMesh.vertexNormals().empty()`
    ScalarMeasure computeMu2InterpolatedU() const;
    /// @return the \f$ \mu^{X,Y} \f$ corrected curvature measure,
    /// i.e. the anisotropic tensor curvature measure, when corrected
    /// normals are interpolated per face.
    /// @pre `! myMesh.vertexNormals().empty()`
    TensorMeasure computeMuXYInterpolatedU() const;


  }; // end of class CorrectedNormalCurrentComputer

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "CorrectedNormalCurrentComputer.ih"
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CorrectedNormalCurrentComputer_h

#undef CorrectedNormalCurrentComputer_RECURSES
#endif // else defined(CorrectedNormalCurrentComputer_RECURSES)
