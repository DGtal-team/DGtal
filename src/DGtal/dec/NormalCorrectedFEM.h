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
 * @file
 * @author
 *
 * @date 2024/06/21
 *
 * Header file for module SurfaceDEC.h
 *
 * This file is part of the DGtal library.
 */

#if !defined NormalCorrectedFEM_h
/** Prevents repeated inclusion of headers. */
#define NormalCorrectedFEM_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include <iostream>
#include <sstream>
#include <string>

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // template class CorrectedFEM
  /**
     Description of template class 'NormalCorrectedFEM' <p> \brief Aim:
     Provides methods for building a stiffness and mass matrix to solve
     Poisson problems using the Finite Element Method using a
     corrected normal field.

     Works with triangular and quadrangular surfaces.

     Requires face normals on attached surface mesh.

   * @tparam TLinearAlgebraBackend linear algebra backend used (i.e.
   EigenSparseLinearAlgebraBackend).
     @tparam TRealPoint an arbitrary model of 3D RealPoint.
     @tparam TRealVector an arbitrary model of 3D RealVector.
  */
  template <typename TLinearAlgebraBackend, typename TRealPoint,
            typename TRealVector>
  struct NormalCorrectedFEM
  {
    typedef TLinearAlgebraBackend LinearAlgebraBackend;
    typedef TRealPoint RealPoint;
    typedef TRealVector RealVector;
    typedef typename LinearAlgebraBackend::SparseMatrix LinearOperator;
    typedef typename LinearAlgebraBackend::SparseMatrix::StorageIndex StorageIndex;
    typedef typename LinearAlgebraBackend::DenseMatrix DenseMatrix;
    typedef typename LinearAlgebraBackend::DenseVector DenseVector;
    typedef SurfaceMesh<RealPoint, RealVector> Mesh;
    typedef typename Mesh::Vertex Vertex;
    typedef typename Mesh::Edge Edge;
    typedef typename Mesh::Face Face;
    typedef typename Mesh::Face Corner;
    typedef typename Mesh::Size Size;

    /// A triplet (row, col, value), useful for initializaing sparse matrices.
    typedef typename LinearAlgebraBackend::Triplet Triplet;
    /// A range of triplets (row,col,value).
    typedef std::vector<Triplet> Triplets;

    private:
    // ------------------------- Initialization services
    // ----------------------------

    void buildTriangleFaceMass( Triplets & triplets, Triplets & lumpedTriplets,
                                Face f ) const
    {
      const auto vtcs         = myMesh->incidentVertices( f );
      const auto v0           = vtcs[ 0 ];
      const auto v1           = vtcs[ 1 ];
      const auto v2           = vtcs[ 2 ];
      const auto xij          = myMesh->position( v1 ) - myMesh->position( v0 );
      const auto xik          = myMesh->position( v2 ) - myMesh->position( v0 );
      const auto n            = xij.crossProduct( xik );
      const auto nn           = n.getNormalized();
      const auto face_n       = myMesh->faceNormal( f );
      const double z          = face_n.dot( nn );
      const double doubleArea = n.norm();
      triplets.push_back( { (StorageIndex)v0, (StorageIndex)v0, 1. / 12. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v0, (StorageIndex)v1, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v0, (StorageIndex)v2, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v1, (StorageIndex)v0, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v1, (StorageIndex)v1, 1. / 12. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v1, (StorageIndex)v2, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v2, (StorageIndex)v0, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v2, (StorageIndex)v1, 1. / 24. * z * doubleArea } );
      triplets.push_back( { (StorageIndex)v2, (StorageIndex)v2, 1. / 12. * z * doubleArea } );
      lumpedTriplets.push_back( { (StorageIndex)v0, (StorageIndex)v0, 1. / 6. * z * doubleArea } );
      lumpedTriplets.push_back( { (StorageIndex)v1, (StorageIndex)v1, 1. / 6. * z * doubleArea } );
      lumpedTriplets.push_back( { (StorageIndex)v2, (StorageIndex)v2, 1. / 6. * z * doubleArea } );
    }

    void buildQuadrangleFaceMass( Triplets & triplets,
                                  Triplets & lumpedTriplets, Face f ) const
    {
      const auto vtcs   = myMesh->incidentVertices( f );
      const auto v0     = vtcs[ 0 ];
      const auto v1     = vtcs[ 1 ];
      const auto v2     = vtcs[ 2 ];
      const auto v3     = vtcs[ 3 ];
      const auto xij    = myMesh->position( v1 ) - myMesh->position( v0 );
      const auto xil    = myMesh->position( v3 ) - myMesh->position( v0 );
      const auto nn     = xij.crossProduct( xil ).getNormalized();
      const auto face_n = myMesh->faceNormal( f );
      const double z    = nn.dot( face_n );
      if ( z < 0. )
      {
        trace.warning() << "Bad corrected normals at face " << f << std::endl;
      }

      double area     = xij.crossProduct( xil ).norm();
      int v[ 2 ][ 2 ] = { { (int)v0, (int)v1 }, { (int)v3, (int)v2 } };
      for ( int i = 0; i < 2; i++ )
      {
        for ( int j = 0; j < 2; j++ )
        {
          for ( int i2 = 0; i2 < 2; i2++ )
          {
            for ( int j2 = 0; j2 < 2; j2++ )
            {
              int v_i = v[ i ][ j ];
              int v_j = v[ i2 ][ j2 ];
              if ( i == i2 && j == j2 )
              {
                triplets.push_back( { (StorageIndex)v_i, (StorageIndex)v_j, area * z * 1. / 9. } );
                lumpedTriplets.push_back( { (StorageIndex)v_i, (StorageIndex)v_j, 1. / 4. * z * area } );
              }
              else if ( i == i2 && j == 1 - j2 )
              {
                triplets.push_back( { (StorageIndex)v_i, (StorageIndex)v_j, area * z * 1. / 18. } );
              }
              else if ( j == j2 && i == 1 - i2 )
              {
                triplets.push_back( { (StorageIndex)v_i, (StorageIndex)v_j, area * z * 1. / 18. } );
              }
              else
              {
                triplets.push_back( { (StorageIndex)v_i, (StorageIndex)v_j, area * z * 1. / 36. } );
              }
            }
          }
        }
      }
    }

    DenseMatrix buildTriangleFaceStiffness( Face f ) const
    {
      DenseMatrix res     = DenseMatrix( 3, 3 );
      const auto vtcs     = myMesh->incidentVertices( f );
      const auto v0       = vtcs[ 0 ];
      const auto v1       = vtcs[ 1 ];
      const auto v2       = vtcs[ 2 ];
      const auto xij      = myMesh->position( v1 ) - myMesh->position( v0 );
      const auto xik      = myMesh->position( v2 ) - myMesh->position( v0 );
      const auto n        = xij.crossProduct( xik );
      const auto nn       = n.getNormalized();
      const auto xil      = nn.crossProduct( xij ).getNormalized();
      const auto face_n   = myMesh->faceNormal( f );
      const double x      = face_n.dot( xij.getNormalized() );
      const double y      = face_n.dot( xil );
      const double z      = face_n.dot( nn );
      const double l1     = xij.norm();
      const double l2     = xik.norm();
      const double cos_t  = xij.dot( xik ) / ( l1 * l2 );
      const double sin_t  = n.norm() / ( l1 * l2 );
      const double x2     = x * x;
      const double y2     = y * y;
      const double z2     = z * z;
      const double l12    = l1 * l1;
      const double l22    = l2 * l2;
      const double cos_t2 = cos_t * cos_t;
      const double sin_t2 = sin_t * sin_t;
      res( 0, 0 ) =
      1. / 2 *
      ( 2 * ( l1 * l2 - cos_t * l22 ) * sin_t * x * y - 2 * cos_t * l1 * l2 +
        l22 * sin_t2 * z2 - l22 * sin_t2 +
        ( 2 * cos_t * l1 * l2 + 2 * l22 * sin_t2 - l12 - l22 ) * x2 + l12 +
        l22 ) /
      ( l1 * l2 * sin_t * z );
      res( 1, 0 ) =
      1. / 2 *
      ( ( 2 * cos_t * l2 - l1 ) * sin_t * x * y + l2 * sin_t2 * y2 +
        cos_t * l1 - ( cos_t * l1 + l2 * sin_t2 - l2 ) * x2 - l2 ) /
      ( l1 * sin_t * z );
      res( 2, 0 ) =
      -1. / 2 *
      ( l2 * sin_t * x * y - cos_t * l2 + ( cos_t * l2 - l1 ) * x2 + l1 ) /
      ( l2 * sin_t * z );
      res( 0, 1 ) =
      1. / 2 *
      ( ( 2 * cos_t * l2 - l1 ) * sin_t * x * y + l2 * sin_t2 * y2 +
        cos_t * l1 - ( cos_t * l1 + l2 * sin_t2 - l2 ) * x2 - l2 ) /
      ( l1 * sin_t * z );
      res( 1, 1 ) = -1. / 2 *
                    ( 2 * cos_t * l2 * sin_t * x * y + cos_t2 * l2 * x2 -
                      ( cos_t2 - 1 ) * l2 * y2 - l2 ) /
                    ( l1 * sin_t * z );
      res( 2, 1 ) =
      1. / 2 * ( sin_t * x * y + cos_t * x2 - cos_t ) / ( sin_t * z );
      res( 0, 2 ) =
      -1. / 2 *
      ( l2 * sin_t * x * y - cos_t * l2 + ( cos_t * l2 - l1 ) * x2 + l1 ) /
      ( l2 * sin_t * z );
      res( 1, 2 ) =
      1. / 2 * ( sin_t * x * y + cos_t * x2 - cos_t ) / ( sin_t * z );
      res( 2, 2 ) = -1. / 2 * ( l1 * x2 - l1 ) / ( l2 * sin_t * z );
      return res;
    }

    DenseMatrix buildQuadrangleFaceStiffness( Face f ) const
    {
      DenseMatrix res   = DenseMatrix( 4, 4 );
      const auto vtcs   = myMesh->incidentVertices( f );
      const auto v0     = vtcs[ 0 ];
      const auto v1     = vtcs[ 1 ];
      const auto v2     = vtcs[ 2 ];
      const auto v3     = vtcs[ 3 ];
      const auto xij    = myMesh->position( v1 ) - myMesh->position( v0 );
      const auto xil    = myMesh->position( v3 ) - myMesh->position( v0 );
      const auto nn     = xij.crossProduct( xil ).getNormalized();
      const auto face_n = myMesh->faceNormal( f );
      const double x    = face_n.dot( xij.getNormalized() );
      const double y    = face_n.dot( xil.getNormalized() );
      const double z    = face_n.dot( nn );
      if ( z < 0. )
      {
        trace.warning() << "Bad corrected normals at face " << f << std::endl;
      }
      const double lap  = 0.5 * ( x * y / z );
      const double sqx  = 1. - x * x;
      const double sqy  = 1. - y * y;
      const double self = lap + 1. / 3. * ( sqx + sqy ) / z;
      const double op1  = -1. / 3. * sqy / z + 1. / 6. * sqx / z;
      const double op2  = -1. / 3. * sqx / z + 1. / 6. * sqy / z;
      const double op   = -lap - 1. / 6. * ( sqx + sqy ) / z;
      res( 0, 0 )       = self;
      res( 0, 1 )       = op1;
      res( 0, 2 )       = op;
      res( 0, 3 )       = op2;

      res( 1, 0 ) = op1;
      res( 1, 1 ) = self - 2. * lap;
      res( 1, 2 ) = op2;
      res( 1, 3 ) = op + 2. * lap;

      res( 2, 0 ) = op;
      res( 2, 1 ) = op2;
      res( 2, 2 ) = self;
      res( 2, 3 ) = op1;

      res( 3, 0 ) = op2;
      res( 3, 1 ) = op + 2. * lap;
      res( 3, 2 ) = op1;
      res( 3, 3 ) = self - 2. * lap;
      return res;
    }

    LinearOperator buildMass()
    {
      return true;
    }

    bool init( const Mesh * pMesh )
    {
      myMesh = pMesh;
      if ( order != 1 )
      {
        trace.error() << "[NormalCorrectedFEM::init]"
                      << " order should be 1" << std::endl;
        return false;
      }
      if ( myMesh->faceNormals().empty() )
      {
        trace.error() << "[NormalCorrectedFEM::init]"
                      << " face normals should be provided." << std::endl;
        return false;
      }
      return true;
    }

    LinearOperator buildL0() const
    {
      LinearOperator L( myMesh->nbVertices(), myMesh->nbVertices() );
      Triplets triplets;
      DenseMatrix values;
      for ( Face f = 0; f < myMesh->nbFaces(); f++ )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        if ( vtcs.size() == 3 )
          values = buildTriangleFaceStiffness( f );
        else if ( vtcs.size() == 4 )
          values = buildQuadrangleFaceStiffness( f );
        else
        {
          trace.error() << "[NormalCorrectedFEM::init]"
                        << " faces should be triangle or quadrangles"
                        << std::endl;

          return L;
        }
        for ( int i = 0; i < values.rows(); i++ )
          for ( int j = 0; j < values.cols(); j++ )
            triplets.push_back( { (StorageIndex)vtcs[ i ], (StorageIndex)vtcs[ j ], values( i, j ) } );
      }
      L.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return -L;
    }

    LinearOperator buildM0() const
    {
      LinearOperator M( myMesh->nbVertices(), myMesh->nbVertices() );

      Triplets triplets;
      Triplets lumpedTriplets;
      for ( Face f = 0; f < myMesh->nbFaces(); f++ )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        if ( vtcs.size() == 3 )
          buildTriangleFaceMass( triplets, lumpedTriplets, f );
        else if ( vtcs.size() == 4 )
          buildQuadrangleFaceMass( triplets, lumpedTriplets, f );
        else
        {
          trace.error() << "[NormalCorrectedFEM::buildM0]"
                        << " faces should be triangles or quadrangles"
                        << std::endl;
          return M;
        }
      }
      M.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return M;
    }

    LinearOperator buildLumpedM0() const
    {
      LinearOperator lumpedM( myMesh->nbVertices(), myMesh->nbVertices() );

      Triplets triplets;
      Triplets lumpedTriplets;
      for ( Face f = 0; f < myMesh->nbFaces(); f++ )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        if ( vtcs.size() == 3 )
          buildTriangleFaceMass( triplets, lumpedTriplets, f );
        else if ( vtcs.size() == 4 )
          buildQuadrangleFaceMass( triplets, lumpedTriplets, f );
        else
        {
          trace.error() << "[NormalCorrectedFEM::buildLumpedM0]"
                        << " faces should be triangles or quadrangles"
                        << std::endl;
          return lumpedM;
        }
      }
      lumpedM.setFromTriplets( lumpedTriplets.cbegin(), lumpedTriplets.cend() );
      return lumpedM;
    }

    private:
    const Mesh * myMesh;
    int order;

    public:
    /// @name Initialization services
    /// @{

    /// Default constructor. The object is invalid.
    NormalCorrectedFEM() : myMesh( nullptr )
    {
    }

    /// Constructor from surface mesh \a smesh.
    /// @param smesh any surface mesh
    /// @param order the order (default 1)
    NormalCorrectedFEM( ConstAlias<Mesh> smesh, int order = 1 )
      : myMesh( nullptr ), order( order )
    {
      init( &smesh );
    }

    /// @}
    /// @name Standard services
    /// @{

    /// Return the stiffness matrix degree x degree of the face.
    /// @param f a face
    /// @return the degree x degree stiffness matrix
    DenseMatrix localL0( Face f ) const
    {
      const auto vtcs = myMesh->incidentVertices( f );
      if ( vtcs.size() == 3 )
        return -buildTriangleFaceStiffness( f );
      else if ( vtcs.size() == 4 )
        return -buildQuadrangleFaceStiffness( f );
      else
      {
        trace.error() << "[NormalCorrectedFEM::buildLocalL0]"
                      << " faces should be triangles or quadrangles"
                      << std::endl;
        return DenseMatrix();
      }
    }

    /// Return the global stiffness matrix n_v x n_v.
    /// @return the n_v x n_v stiffness matrix
    ///
    /// @note The sign convention for the divergence and the Laplacian
    /// operator is opposite to the one of @cite degoes2020discrete .
    /// This is to match the usual mathematical
    /// convention that the Laplacian (and the Laplacian-Beltrami) has
    /// negative eigenvalues (and is the sum of second derivatives in
    /// the cartesian grid). It also follows the formal adjointness of
    /// exterior derivative and opposite of divergence as relation \f$
    /// \langle \mathrm{d} u, v \rangle = - \langle u, \mathrm{div} v
    /// \rangle \f$. See also
    /// https://en.wikipedia.org/wiki/Laplace–Beltrami_operator
    LinearOperator L0() const
    {
      return buildL0();
    }

    /// Return the global mass matrix n_v x n_v.
    /// This matrix is not diagonal. To obtain a (less precise)
    /// diagonal version use lumpedM0()
    /// @return the n_v x n_v mass matrix
    LinearOperator M0() const
    {
      return buildM0();
    }

    /// Return the global lumped mass matrix n_v x n_v.
    /// This matrix is diagonal. To obtain a (more precise)
    /// non diagonal version use M0()
    /// @return the n_v x n_v mass matrix
    LinearOperator lumpedM0() const
    {
      return buildLumpedM0();
    }
    /// @}
  };
} // namespace DGtal

#endif
