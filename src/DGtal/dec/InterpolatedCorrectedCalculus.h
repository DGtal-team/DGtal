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

 
#if !defined InterpolatedCorrectedCalculus_h
/** Prevents repeated inclusion of headers. */
#define InterpolatedCorrectedCalculus_h

#include "DGtal/base/Common.h"
#include "DGtal/dec/SurfaceDEC.h"
#include "DGtal/shapes/SurfaceMesh.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class InterpolatedCorrectedCalculus
  /**
     Description of template class 'InterpolatedCorrectedCalculus' <p>
     \brief Implement differential operators on digital surfaces with
     a vertex normal field.

     Works digital surfaces.

     Requires vertex normals on attached surface mesh.

     Implements SurfaceDEC methods. Use them to access the corresponding
   operators.

   * @tparam TLinearAlgebraBackend linear algebra backend used (i.e.
   EigenSparseLinearAlgebraBackend).
     @tparam TRealPoint an arbitrary model of 3D RealPoint.
     @tparam TRealVector an arbitrary model of 3D RealVector.
  */
  template <typename TLinearAlgebraBackend, typename TRealPoint,
            typename TRealVector>
  class InterpolatedCorrectedCalculus
    : public SurfaceDEC<InterpolatedCorrectedCalculus<TLinearAlgebraBackend,
                                                      TRealPoint, TRealVector>,
                        TLinearAlgebraBackend>
  {
    friend class SurfaceDEC<InterpolatedCorrectedCalculus,
                            TLinearAlgebraBackend>;

    public:
    typedef TLinearAlgebraBackend LinearAlgebraBackend;
    typedef TRealPoint RealPoint;
    typedef TRealVector RealVector;
    typedef typename LinearAlgebraBackend::DenseVector::Index Index;
    typedef typename LinearAlgebraBackend::DenseVector::Scalar Scalar;
    typedef typename LinearAlgebraBackend::SparseMatrix LinearOperator;
    typedef
    typename LinearAlgebraBackend::SparseMatrix::StorageIndex StorageIndex;
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
    const Mesh * myMesh;
    bool use2ndOrder = false;
    double lambda    = 1.;

    DenseMatrix quadrangleSharpLocalMatrix( Face f ) const
    {
      DenseMatrix Basis = DenseMatrix::Zero( 3, 3 );
      DenseMatrix Gavg  = DenseMatrix::Zero( 3, 4 );
      DenseMatrix Gedge = DenseMatrix::Zero( 3, 4 );
      DenseMatrix S     = DenseMatrix::Zero( 3, 4 );
      const auto vtcs   = myMesh->incidentVertices( f );
      const auto i      = vtcs[ 0 ];
      const auto j      = vtcs[ 1 ];
      const auto k      = vtcs[ 2 ];
      const auto l      = vtcs[ 3 ];
      const auto dxij   = myMesh->position( j ) - myMesh->position( i );
      const auto dxjk   = myMesh->position( k ) - myMesh->position( j );
      const auto xij    = dxij.getNormalized();
      const auto xjk    = dxjk.getNormalized();

      const RealVector n = xij.crossProduct( xjk );
      const Dimension x  = fabs( xij[ 0 ] ) > fabs( xij[ 1 ] )
                           ? ( fabs( xij[ 0 ] ) > fabs( xij[ 2 ] ) ? 0 : 2 )
                           : ( fabs( xij[ 1 ] ) > fabs( xij[ 2 ] ) ? 1 : 2 );
      const Dimension y  = fabs( xjk[ 0 ] ) > fabs( xjk[ 1 ] )
                           ? ( fabs( xjk[ 0 ] ) > fabs( xjk[ 2 ] ) ? 0 : 2 )
                           : ( fabs( xjk[ 1 ] ) > fabs( xjk[ 2 ] ) ? 1 : 2 );
      const Dimension z  = fabs( n[ 0 ] ) > fabs( n[ 1 ] )
                           ? ( fabs( n[ 0 ] ) > fabs( n[ 2 ] ) ? 0 : 2 )
                           : ( fabs( n[ 1 ] ) > fabs( n[ 2 ] ) ? 1 : 2 );
      if ( ( x == y ) || ( x == z ) || ( y == z ) || ( x + y + z > 3 ) )
        trace.warning() << "Bad basis: " << x << y << z << std::endl;
      const Scalar sign_x = xij[ x ] > 0.0 ? 1.0 : -1.0;
      const Scalar sign_y = xjk[ y ] > 0.0 ? 1.0 : -1.0;
      const Scalar sign_z = n[ z ] > 0.0 ? 1.0 : -1.0;

      if ( use2ndOrder == false )
      {
        const auto ui = myMesh->vertexNormal( i );
        const auto uj = myMesh->vertexNormal( j );
        const auto uk = myMesh->vertexNormal( k );
        const auto ul = myMesh->vertexNormal( l );
        const auto u0 = 0.5 * ( ui + uj );
        const auto u1 = 0.5 * ( uj + uk );
        const auto u2 = 0.5 * ( uk + ul );
        const auto u3 = 0.5 * ( ul + ui );
        const auto u  = 0.5 * ( u0 + u2 );
        // We put u into the "natural" frame of the surfel
        const auto ux         = xij.dot( u );
        const auto uy         = xjk.dot( u );
        const auto uz         = n.dot( u );
        Gavg.coeffRef( 0, 0 ) = uz;
        Gavg.coeffRef( 0, 2 ) = -uz;
        Gavg.coeffRef( 1, 1 ) = uz;
        Gavg.coeffRef( 1, 3 ) = -uz;
        Gavg.coeffRef( 2, 0 ) = -ux;
        Gavg.coeffRef( 2, 1 ) = -uy;
        Gavg.coeffRef( 2, 2 ) = ux;
        Gavg.coeffRef( 2, 3 ) = uy;
        Gavg /= 3.0;
        Gedge.coeffRef( 0, 0 ) = n.dot( u0 );    //  u0[z];
        Gedge.coeffRef( 0, 2 ) = -n.dot( u2 );   // -u2[z];
        Gedge.coeffRef( 1, 1 ) = n.dot( u1 );    //  u1[z];
        Gedge.coeffRef( 1, 3 ) = -n.dot( u3 );   // -u3[z];
        Gedge.coeffRef( 2, 0 ) = -xij.dot( u0 ); // -u0[x];
        Gedge.coeffRef( 2, 1 ) = -xjk.dot( u1 ); // -u1[y];
        Gedge.coeffRef( 2, 2 ) = xij.dot( u2 );  //  u2[x];
        Gedge.coeffRef( 2, 3 ) = xjk.dot( u3 );  //  u3[y];
        Gedge /= 6.0;
        // We compute the change of basis to come back to the canonic frame of
        // R3.
        S = Gavg + Gedge;
      }
      else
      { // use2ndOrder
        const auto u00  = myMesh->vertexNormal( i );
        const auto u10  = myMesh->vertexNormal( j );
        const auto u11  = myMesh->vertexNormal( k );
        const auto u01  = myMesh->vertexNormal( l );
        const auto ux00 = xij.dot( u00 );
        const auto ux10 = xij.dot( u10 );
        const auto ux11 = xij.dot( u11 );
        const auto ux01 = xij.dot( u01 );
        const auto uy00 = xjk.dot( u00 );
        const auto uy10 = xjk.dot( u10 );
        const auto uy11 = xjk.dot( u11 );
        const auto uy01 = xjk.dot( u01 );
        const auto uz00 = n.dot( u00 );
        const auto uz10 = n.dot( u10 );
        const auto uz11 = n.dot( u11 );
        const auto uz01 = n.dot( u01 );
        const auto a    = 1.0 / ( u00 + u10 ).norm();
        const auto b    = 1.0 / ( u10 + u11 ).norm();
        const auto c    = 1.0 / ( u11 + u01 ).norm();
        const auto d    = 1.0 / ( u01 + u00 ).norm();
        const auto e    = 1.0 / ( u00 + u10 + u11 + u01 ).norm();
        S.coeffRef( 0, 0 ) =
        ( 4 * a + 2 * d + 8 * e + 1 ) * uz00 + 2 * ( d + 4 * e ) * uz01 +
        ( 4 * a + 2 * b + 8 * e + 1 ) * uz10 + 2 * ( b + 4 * e ) * uz11;
        S.coeffRef( 0, 2 ) =
        -2 * ( d + 4 * e ) * uz00 - ( 4 * c + 2 * d + 8 * e + 1 ) * uz01 -
        2 * ( b + 4 * e ) * uz10 - ( 2 * b + 4 * c + 8 * e + 1 ) * uz11;
        S.coeffRef( 1, 1 ) = 2 * ( a + 4 * e ) * uz00 +
                             2 * ( c + 4 * e ) * uz01 +
                             ( 2 * a + 4 * b + 8 * e + 1 ) * uz10 +
                             ( 4 * b + 2 * c + 8 * e + 1 ) * uz11;
        S.coeffRef( 1, 3 ) = -( 2 * a + 4 * d + 8 * e + 1 ) * uz00 -
                             ( 2 * c + 4 * d + 8 * e + 1 ) * uz01 -
                             2 * ( a + 4 * e ) * uz10 -
                             2 * ( c + 4 * e ) * uz11;
        S.coeffRef( 2, 0 ) =
        -( 4 * a + 2 * d + 8 * e + 1 ) * ux00 - 2 * ( d + 4 * e ) * ux01 -
        ( 4 * a + 2 * b + 8 * e + 1 ) * ux10 - 2 * ( b + 4 * e ) * ux11;
        S.coeffRef( 2, 1 ) = -2 * ( a + 4 * e ) * uy00 -
                             2 * ( c + 4 * e ) * uy01 -
                             ( 2 * a + 4 * b + 8 * e + 1 ) * uy10 -
                             ( 4 * b + 2 * c + 8 * e + 1 ) * uy11;
        S.coeffRef( 2, 2 ) =
        2 * ( d + 4 * e ) * ux00 + ( 4 * c + 2 * d + 8 * e + 1 ) * ux01 +
        2 * ( b + 4 * e ) * ux10 + ( 2 * b + 4 * c + 8 * e + 1 ) * ux11;
        S.coeffRef( 2, 3 ) = ( 2 * a + 4 * d + 8 * e + 1 ) * uy00 +
                             ( 2 * c + 4 * d + 8 * e + 1 ) * uy01 +
                             2 * ( a + 4 * e ) * uy10 +
                             2 * ( c + 4 * e ) * uy11;
        S /= 36.0;
      }
      const Scalar h         = fabs( dxij[ x ] );
      Basis.coeffRef( 0, x ) = sign_x * h;
      Basis.coeffRef( 1, y ) = sign_y * h;
      Basis.coeffRef( 2, z ) = sign_z * h;
      const auto a_f         = quadrangleArea( f );
      return ( Basis.transpose() / a_f ) * S;
    }

    DenseMatrix quadrangleFlatLocalMatrix( Face f ) const
    {
      DenseMatrix Basis = DenseMatrix::Zero( 3, 3 );
      DenseMatrix V     = DenseMatrix::Zero( 4, 3 );
      const auto vtcs   = myMesh->incidentVertices( f );
      if ( vtcs.size() != 4 )
      {
        trace.error()
        << "[SurfaceMeshDEC::quadrangleCorrectedDigitalFlatLocalMatrix]"
        << " restricted to quadrangle faces." << std::endl;
        return V;
      }
      const auto i    = vtcs[ 0 ];
      const auto j    = vtcs[ 1 ];
      const auto k    = vtcs[ 2 ];
      const auto l    = vtcs[ 3 ];
      const auto dxij = myMesh->position( j ) - myMesh->position( i );
      const auto dxjk = myMesh->position( k ) - myMesh->position( j );
      const auto xij  = dxij.getNormalized();
      const auto xjk  = dxjk.getNormalized();

      const RealVector n = xij.crossProduct( xjk );
      const Dimension x  = fabs( xij[ 0 ] ) > fabs( xij[ 1 ] )
                           ? ( fabs( xij[ 0 ] ) > fabs( xij[ 2 ] ) ? 0 : 2 )
                           : ( fabs( xij[ 1 ] ) > fabs( xij[ 2 ] ) ? 1 : 2 );
      const Dimension y  = fabs( xjk[ 0 ] ) > fabs( xjk[ 1 ] )
                           ? ( fabs( xjk[ 0 ] ) > fabs( xjk[ 2 ] ) ? 0 : 2 )
                           : ( fabs( xjk[ 1 ] ) > fabs( xjk[ 2 ] ) ? 1 : 2 );
      const Dimension z  = fabs( n[ 0 ] ) > fabs( n[ 1 ] )
                           ? ( fabs( n[ 0 ] ) > fabs( n[ 2 ] ) ? 0 : 2 )
                           : ( fabs( n[ 1 ] ) > fabs( n[ 2 ] ) ? 1 : 2 );
      if ( ( x == y ) || ( x == z ) || ( y == z ) || ( x + y + z > 3 ) )
        trace.warning() << "Bad basis: " << x << y << z << std::endl;
      const Scalar sign_x = xij[ x ] > 0.0 ? 1.0 : -1.0;
      const Scalar sign_y = xjk[ y ] > 0.0 ? 1.0 : -1.0;
      const Scalar sign_z = n[ z ] > 0.0 ? 1.0 : -1.0;
      const auto ui       = myMesh->vertexNormal( i );
      const auto uj       = myMesh->vertexNormal( j );
      const auto uk       = myMesh->vertexNormal( k );
      const auto ul       = myMesh->vertexNormal( l );
      // We put all the u into the "natural" frame of the surfel
      const auto uix = xij.dot( ui );
      const auto uiy = xjk.dot( ui );
      const auto uiz = n.dot( ui );
      const auto ujx = xij.dot( uj );
      const auto ujy = xjk.dot( uj );
      const auto ujz = n.dot( uj );
      const auto ukx = xij.dot( uk );
      const auto uky = xjk.dot( uk );
      const auto ukz = n.dot( uk );
      const auto ulx = xij.dot( ul );
      const auto uly = xjk.dot( ul );
      const auto ulz = n.dot( ul );
      if ( !use2ndOrder )
      {
        V.coeffRef( 0, 0 ) =
        6.0 - 2.0 * ( 2 * uix + ujx ) * uix - 2.0 * ( uix + 2 * ujx ) * ujx;
        V.coeffRef( 0, 1 ) =
        -2.0 * ( 2 * uix + ujx ) * uiy - 2.0 * ( uix + 2 * ujx ) * ujy;
        V.coeffRef( 0, 2 ) =
        -2.0 * ( 2 * uix + ujx ) * uiz - 2.0 * ( uix + 2 * ujx ) * ujz;
        V.coeffRef( 1, 0 ) =
        -2.0 * ( 2 * ujy + uky ) * ujx - 2.0 * ( ujy + 2 * uky ) * ukx;
        V.coeffRef( 1, 1 ) =
        6.0 - 2.0 * ( 2 * ujy + uky ) * ujy - 2.0 * ( ujy + 2 * uky ) * uky;
        V.coeffRef( 1, 2 ) =
        -2.0 * ( 2 * ujy + uky ) * ujz - 2.0 * ( ujy + 2 * uky ) * ukz;
        V.coeffRef( 2, 0 ) =
        -6.0 + 2.0 * ( 2 * ukx + ulx ) * ukx + 2.0 * ( ukx + 2 * ulx ) * ulx;
        V.coeffRef( 2, 1 ) =
        2.0 * ( 2 * ukx + ulx ) * uky + 2.0 * ( ukx + 2 * ulx ) * uly;
        V.coeffRef( 2, 2 ) =
        2.0 * ( 2 * ukx + ulx ) * ukz + 2.0 * ( ukx + 2 * ulx ) * ulz;
        V.coeffRef( 3, 0 ) =
        2.0 * ( 2 * uly + uiy ) * ulx + 2.0 * ( uly + 2 * uiy ) * uix;
        V.coeffRef( 3, 1 ) =
        -6.0 + 2.0 * ( 2 * uly + uiy ) * uly + 2.0 * ( uly + 2 * uiy ) * uiy;
        V.coeffRef( 3, 2 ) =
        2.0 * ( 2 * uly + uiy ) * ulz + 2.0 * ( uly + 2 * uiy ) * uiz;
        V /= 6.0;
      }
      else
      {
        const auto a   = 1.0 / ( ui + uj ).norm();
        const auto b   = 1.0 / ( uj + uk ).norm();
        const auto c   = 1.0 / ( uk + ul ).norm();
        const auto d   = 1.0 / ( ul + ui ).norm();
        const auto e   = 1.0 / ( ui + uj + uk + ul ).norm();
        const auto ap  = 4 * a * a + a + 1;
        const auto app = 16 * a * a + 4 * a - 1;
        const auto bp  = 4 * b * b + b + 1;
        const auto bpp = 16 * b * b + 4 * b - 1;
        const auto cp  = 4 * c * c + c + 1;
        const auto cpp = 16 * c * c + 4 * c - 1;
        const auto dp  = 4 * d * d + d + 1;
        const auto dpp = 16 * d * d + 4 * d - 1;
        V.coeffRef( 0, 0 ) =
        -4 * ap * uix * uix - 2 * app * uix * ujx - 4 * ap * ujx * ujx + 30.0;
        V.coeffRef( 0, 1 ) = -( 4 * ap * uix + app * ujx ) * uiy -
                             ( app * uix + 4 * ap * ujx ) * ujy;
        V.coeffRef( 0, 2 ) = -( 4 * ap * uix + app * ujx ) * uiz -
                             ( app * uix + 4 * ap * ujx ) * ujz;
        V.coeffRef( 1, 0 ) = -( 4 * bp * ujx + bpp * ukx ) * ujy -
                             ( bpp * ujx + 4 * bp * ukx ) * uky;
        V.coeffRef( 1, 1 ) =
        -4 * bp * ujy * ujy - 2 * bpp * ujy * uky - 4 * bp * uky * uky + 30.0;
        V.coeffRef( 1, 2 ) = -( 4 * bp * ujy + bpp * uky ) * ujz -
                             ( bpp * ujy + 4 * bp * uky ) * ukz;
        V.coeffRef( 2, 0 ) =
        4 * cp * ulx * ulx + 2 * cpp * ulx * ukx + 4 * cp * ukx * ukx - 30.0;
        V.coeffRef( 2, 1 ) =
        ( 4 * cp * ulx + cpp * ukx ) * uly + ( cpp * ulx + 4 * cp * ukx ) * uky;
        V.coeffRef( 2, 2 ) =
        ( 4 * cp * ulx + cpp * ukx ) * ulz + ( cpp * ulx + 4 * cp * ukx ) * ukz;
        V.coeffRef( 3, 0 ) =
        ( 4 * dp * uix + dpp * ulx ) * uiy + ( dpp * uix + 4 * dp * ulx ) * uly;
        V.coeffRef( 3, 1 ) =
        4 * dp * uiy * uiy + 2 * dpp * uiy * uly + 4 * dp * uly * uly - 30.0;
        V.coeffRef( 3, 2 ) =
        ( 4 * dp * uiy + dpp * uly ) * uiz + ( dpp * uiy + 4 * dp * uly ) * ulz;
        V /= 30.0;
      }

      // We compute the change of basis to come back to the canonic frame of R3.
      const Scalar h         = fabs( dxij[ x ] );
      Basis.coeffRef( 0, x ) = sign_x * h;
      Basis.coeffRef( 1, y ) = sign_y * h;
      Basis.coeffRef( 2, z ) = sign_z * h;
      return V * Basis;
    }

    //-----------------------------------------------------------------------------
    DenseMatrix quadrangleInnerProduct0( Face f ) const
    {
      const auto vtcs          = myMesh->incidentVertices( f );
      const auto i             = vtcs[ 0 ];
      const auto j             = vtcs[ 1 ];
      const auto k             = vtcs[ 2 ];
      const auto l             = vtcs[ 3 ];
      const auto xij           = myMesh->position( j ) - myMesh->position( i );
      const auto xil           = myMesh->position( l ) - myMesh->position( i );
      const auto xlk           = myMesh->position( k ) - myMesh->position( l );
      const auto xjk           = myMesh->position( k ) - myMesh->position( j );
      const RealVector nn[ 4 ] = { /* ni */ xij.crossProduct( xil ),
                                   /* nj */ xij.crossProduct( xjk ),
                                   /* nk */ xlk.crossProduct( xjk ),
                                   /* nl */ xlk.crossProduct( xil ) };
      const Scalar factor      = 1.0 / 3600.0;
      const Vertex id[ 4 ]     = { i, j, k, l };
      DenseMatrix M            = DenseMatrix::Zero( 4, 4 );
      if ( !use2ndOrder )
      {
        // All coefs are computed from polynomial of the form:
        // s^a (1-s)^a t^b (1-t)^b
        // Indices are 0,1,2,3 == i,j,k,l
        const int pa[ 4 ]         = { 0, 1, 1, 0 };
        const int pb[ 4 ]         = { 0, 0, 1, 1 };
        const double co[ 5 ][ 5 ] = { { 144.0, 36.0, 24.0, 36.0, 144.0 },
                                      { 36.0, 9.0, 6.0, 9.0, 36.0 },
                                      { 24.0, 6.0, 4.0, 6.0, 24.0 },
                                      { 36.0, 9.0, 6.0, 9.0, 36.0 },
                                      { 144.0, 36.0, 24.0, 36.0, 144.0 } };
        for ( int g = 0; g < 4; ++g )   // left 0-form at vertex
          for ( int h = 0; h < 4; ++h ) // right 0-form at vertex
          { // Computing coefficients for ( id[ g ], id[ h ] )
            for ( int u = 0; u < 4; ++u )   // corrected normal at vertex
              for ( int n = 0; n < 4; ++n ) // naive normal at vertex
              {
                const int a = pa[ g ] + pa[ h ] + pa[ u ] + pa[ n ];
                const int b = pb[ g ] + pb[ h ] + pb[ u ] + pb[ n ];
                const double coeff =
                factor * co[ a ][ b ] *
                myMesh->vertexNormal( id[ u ] ).dot( nn[ n ] );
                M( g, h ) += coeff;
              }
          }
      }
      else
      { // use2ndOrder
        // # 1ere ligne de M0 (avec kle facteur 3600*)
        const auto ui = myMesh->vertexNormal( i );
        const auto uj = myMesh->vertexNormal( j );
        const auto uk = myMesh->vertexNormal( k );
        const auto ul = myMesh->vertexNormal( l );
        // formula is theoretically valid for constant geometric normals nn
        const auto uiz     = nn[ 0 ].dot( myMesh->vertexNormal( i ) );
        const auto ujz     = nn[ 1 ].dot( myMesh->vertexNormal( j ) );
        const auto ukz     = nn[ 2 ].dot( myMesh->vertexNormal( k ) );
        const auto ulz     = nn[ 3 ].dot( myMesh->vertexNormal( l ) );
        const auto a       = 1.0 / ( ui + uj ).norm();
        const auto b       = 1.0 / ( uj + uk ).norm();
        const auto c       = 1.0 / ( uk + ul ).norm();
        const auto d       = 1.0 / ( ul + ui ).norm();
        const auto e       = 1.0 / ( ui + uj + uk + ul ).norm();
        M.coeffRef( 0, 0 ) = ( 108 * a + 108 * d + 144 * e + 81 ) * uiz +
                             ( 108 * a - 12 * b + 144 * e - 9 ) * ujz +
                             ( -12 * b - 12 * c + 144 * e + 1 ) * ukz +
                             ( -12 * c + 108 * d + 144 * e - 9 ) * ulz;
        M.coeffRef( 0, 1 ) = ( 72 * a + 12 * d + 96 * e + 9 ) * uiz +
                             ( 72 * a + 12 * b + 96 * e + 9 ) * ujz +
                             ( 12 * b - 8 * c + 96 * e - 1 ) * ukz +
                             ( -8 * c + 12 * d + 96 * e - 1 ) * ulz;
        M.coeffRef( 0, 2 ) = ( 8 * a + 8 * d + 64 * e + 1 ) * uiz +
                             ( 8 * a + 8 * b + 64 * e + 1 ) * ujz +
                             ( 8 * b + 8 * c + 64 * e + 1 ) * ukz +
                             ( 8 * c + 8 * d + 64 * e + 1 ) * ulz;
        M.coeffRef( 0, 3 ) = ( 12 * a + 72 * d + 96 * e + 9 ) * uiz +
                             ( 12 * a - 8 * b + 96 * e - 1 ) * ujz +
                             ( -8 * b + 12 * c + 96 * e - 1 ) * ukz +
                             ( 12 * c + 72 * d + 96 * e + 9 ) * ulz;
        M.coeffRef( 1, 0 ) = M( 0, 1 );
        M.coeffRef( 1, 1 ) = ( 108 * a - 12 * d + 144 * e - 9 ) * uiz +
                             ( 108 * a + 108 * b + 144 * e + 81 ) * ujz +
                             ( 108 * b - 12 * c + 144 * e - 9 ) * ukz +
                             ( -12 * c - 12 * d + 144 * e + 1 ) * ulz;
        M.coeffRef( 1, 2 ) = ( 12 * a - 8 * d + 96 * e - 1 ) * uiz +
                             ( 12 * a + 72 * b + 96 * e + 9 ) * ujz +
                             ( 72 * b + 12 * c + 96 * e + 9 ) * ukz +
                             ( 12 * c - 8 * d + 96 * e - 1 ) * ulz;
        M.coeffRef( 1, 3 ) = ( 8 * a + 8 * d + 64 * e + 1 ) * uiz +
                             ( 8 * a + 8 * b + 64 * e + 1 ) * ujz +
                             ( 8 * b + 8 * c + 64 * e + 1 ) * ukz +
                             ( 8 * c + 8 * d + 64 * e + 1 ) * ulz;
        M.coeffRef( 2, 0 ) = M( 0, 2 );
        M.coeffRef( 2, 1 ) = M( 1, 2 );
        M.coeffRef( 2, 2 ) = ( -12 * a - 12 * d + 144 * e + 1 ) * uiz +
                             ( -12 * a + 108 * b + 144 * e - 9 ) * ujz +
                             ( 108 * b + 108 * c + 144 * e + 81 ) * ukz +
                             ( 108 * c - 12 * d + 144 * e - 9 ) * ulz;
        M.coeffRef( 2, 3 ) = ( -8 * a + 12 * d + 96 * e - 1 ) * uiz +
                             ( -8 * a + 12 * b + 96 * e - 1 ) * ujz +
                             ( 12 * b + 72 * c + 96 * e + 9 ) * ukz +
                             ( 72 * c + 12 * d + 96 * e + 9 ) * ulz;
        M.coeffRef( 3, 0 ) = M( 0, 3 );
        M.coeffRef( 3, 1 ) = M( 1, 3 );
        M.coeffRef( 3, 2 ) = M( 2, 3 );
        M.coeffRef( 3, 3 ) = ( -12 * a + 108 * d + 144 * e - 9 ) * uiz +
                             ( -12 * a - 12 * b + 144 * e + 1 ) * ujz +
                             ( -12 * b + 108 * c + 144 * e - 9 ) * ukz +
                             ( 108 * c + 108 * d + 144 * e + 81 ) * ulz;
        M *= factor;
      }
      return M;
    }

    Scalar quadrangleArea( Face f ) const
    {
      const auto vtcs = myMesh->incidentVertices( f );
      ASSERT( vtcs.size() == 4 );
      const auto i        = vtcs[ 0 ];
      const auto j        = vtcs[ 1 ];
      const auto k        = vtcs[ 2 ];
      const auto l        = vtcs[ 3 ];
      const auto xij      = myMesh->position( j ) - myMesh->position( i );
      const auto xil      = myMesh->position( l ) - myMesh->position( i );
      const auto xlk      = myMesh->position( k ) - myMesh->position( l );
      const auto xjk      = myMesh->position( k ) - myMesh->position( j );
      const RealVector ni = xij.crossProduct( xil );
      const RealVector nj = xij.crossProduct( xjk );
      const RealVector nk = xlk.crossProduct( xjk );
      const RealVector nl = xlk.crossProduct( xil );
      const auto & ui     = myMesh->vertexNormal( i );
      const auto & uj     = myMesh->vertexNormal( j );
      const auto & uk     = myMesh->vertexNormal( k );
      const auto & ul     = myMesh->vertexNormal( l );
      const Scalar factor = 1.0 / 36.0;
      if ( !use2ndOrder )
        return factor * ( ni.dot( 4 * ui + 2 * uj + uk + 2 * ul ) +
                          nj.dot( 2 * ui + 4 * uj + 2 * uk + ul ) +
                          nk.dot( ui + 2 * uj + 4 * uk + 2 * ul ) +
                          nl.dot( 2 * ui + uj + 2 * uk + 4 * ul ) );
      else
      {
        // Works only for surfels ! i.e. xij==xlk and xjk==xil
        // If we assume f00=f10=f01=f11=1, the corrected area of a surfel is,
        // after *36 factor A = (4*a+4*d+16*e+1) < u00 | n >
        //   + (4*a+4*b+16*e+1) < u10 | n >
        //   + (4*b+4*c+16*e+1) < u11 | n >
        //   + (4*c+4*d+16*e+1) < u01 | n >
        const auto a = 1.0 / ( ui + uj ).norm();
        const auto b = 1.0 / ( uj + uk ).norm();
        const auto c = 1.0 / ( uk + ul ).norm();
        const auto d = 1.0 / ( ul + ui ).norm();
        const auto e = 1.0 / ( ui + uj + uk + ul ).norm();
        return factor * ( ( 4 * a + 4 * d + 16 * e + 1 ) * ni.dot( ui ) +
                          ( 4 * a + 4 * b + 16 * e + 1 ) * nj.dot( uj ) +
                          ( 4 * b + 4 * c + 16 * e + 1 ) * nk.dot( uk ) +
                          ( 4 * c + 4 * d + 16 * e + 1 ) * nl.dot( ul ) );
      }
    }

    public:
    /// @name Initialization services
    /// @{

    /// Default constructor. The object is invalid.
    InterpolatedCorrectedCalculus() : myMesh( nullptr )
    {
    }

    /// Constructor from surface mesh \a smesh.
    /// @param smesh any surface mesh
    /// @param use2ndOrder wether to use 2nd order method for interpolating
    /// normals (slightly more precise)
    /// @param lambda value used for M1 positive definitness following @cite
    /// degoes2020discrete
    InterpolatedCorrectedCalculus( const ConstAlias<Mesh> smesh,
                                   bool use2ndOrder = false,
                                   double lambda    = 0.1 )
      : myMesh( &smesh ), use2ndOrder( use2ndOrder ), lambda( lambda )
    {
      if ( myMesh->vertexNormals().empty() )
      {
        trace.error() << "[InterpolatedCorrectedCalculus::init]"
                      << " vertex normals should be provided." << std::endl;
        return;
      }
    }
    ///@}

    private:
    DenseMatrix buildLocalM0( Index f ) const
    {
      if ( myMesh->incidentVertices( f ).size() == 4 )
        return quadrangleInnerProduct0( f );
      else
      {
        trace.error() << "[InterpolatedCorrectedCalculus::buildLocalM0]"
                      << " face should be quadrangles" << std::endl;
        return DenseMatrix();
      }
    }

    LinearOperator buildM0() const
    {
      Triplets triplets;
      LinearOperator myM0 =
      LinearOperator( myMesh->nbVertices(), myMesh->nbVertices() );
      DenseMatrix values;
      for ( Face f = 0; f < myMesh->nbFaces(); ++f )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        const auto nv   = vtcs.size();
        if ( nv == 4 )
          values = quadrangleInnerProduct0( f );
        else
        {
          trace.error() << "[InterpolatedCorrectedCalculus::init]"
                        << " faces should be quadrangles" << std::endl;
          return myM0;
        }
        for ( int i = 0; i < values.rows(); i++ )
          for ( int j = 0; j < values.cols(); j++ )
            triplets.push_back( { (StorageIndex)vtcs[ i ],
                                  (StorageIndex)vtcs[ j ], values( i, j ) } );
      }
      myM0.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return myM0;
    }

    LinearOperator buildLumpedM0() const
    {
      return buildM0();
    }

    DenseMatrix buildLocalM1( Index f ) const
    {
      const auto a_f    = quadrangleArea( f );
      DenseMatrix CA    = DenseMatrix::Identity( 3, 3 ) * a_f;
      DenseMatrix sharp = buildLocalSharp( f );
      DenseMatrix Id    = DenseMatrix::Identity( 4, 4 );
      DenseMatrix P     = Id - buildLocalFlat( f ) * sharp;
      // Compute inner product for 1-forms
      DenseMatrix myM1 =
      sharp.transpose() * CA * sharp + lambda * P.transpose() * P;
      return myM1;
    }

    LinearOperator buildM1() const
    {

      LinearOperator CA( 3 * myMesh->nbFaces(), 3 * myMesh->nbFaces() );
      Triplets triplets;
      for ( Face f = 0; f < myMesh->nbFaces(); ++f )
      {
        const auto a_f = quadrangleArea( f );
        if ( a_f <= 1e-8 )
          trace.warning() << "Bad corrected area " << f << " " << a_f
                          << std::endl;
        triplets.push_back( { (StorageIndex)3 * f, (StorageIndex)3 * f, a_f } );
        triplets.push_back(
        { (StorageIndex)3 * f + 1, (StorageIndex)3 * f + 1, a_f } );
        triplets.push_back(
        { (StorageIndex)3 * f + 2, (StorageIndex)3 * f + 2, a_f } );
      }
      CA.setFromTriplets( triplets.cbegin(), triplets.cend() );
      // Compute projection operator
      LinearOperator mySharp = buildSharp();
      LinearOperator Id1( myMesh->nbEdges(), myMesh->nbEdges() );
      Id1.setIdentity();
      LinearOperator P = Id1 - buildFlat() * mySharp;
      // Compute inner product for 1-forms
      LinearOperator myM1 =
      mySharp.transpose() * CA * mySharp + lambda * P.transpose() * P;
      return myM1;
    }

    DenseMatrix buildLocalM2( Index f ) const
    {
      const auto nv = myMesh->incidentVertices( f ).size();
      DenseMatrix res;
      if ( nv == 4 )
        res = DenseMatrix::Identity( 1, 1 ) / quadrangleArea( f );
      else
      {
        trace.error() << "[InterpolatedCorrectedCalculus::buildLocalM2]"
                      << " face should be quadrangles" << std::endl;
      }
      return res;
    }

    LinearOperator buildM2() const
    {
      Triplets triplets;
      LinearOperator myM2 =
      LinearOperator( myMesh->nbFaces(), myMesh->nbFaces() );
      for ( Face f = 0; f < myMesh->nbFaces(); ++f )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        double area     = 0.;
        if ( vtcs.size() == 4 )
          area = quadrangleArea( f );
        else
        {
          trace.error() << "[InterpolatedCorrectedCalculus::buildM2]"
                        << " faces should be quadrangular." << std::endl;
        }
        triplets.push_back( { (StorageIndex)f, (StorageIndex)f, 1.0 / area } );
      }
      myM2.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return myM2;
    }

    DenseMatrix buildLocalSharp( Index f ) const
    {
      if ( myMesh->incidentVertices( f ).size() != 4 )
      {
        trace.error() << "[InterpolatedCorrectedCalculus::buildLocalSharp]"
                      << " restricted to quadrangle faces." << std::endl;
        return DenseMatrix();
      }
      return quadrangleSharpLocalMatrix( f );
    }

    LinearOperator buildSharp() const
    {
      Triplets triplets;
      LinearOperator mySharp =
      LinearOperator( 3 * myMesh->nbFaces(), myMesh->nbEdges() );
      for ( Face f = 0; f < myMesh->nbFaces(); ++f )
      {
        // std::cout << "Face " << f << std::endl;
        const auto vtcs = myMesh->incidentVertices( f );
        const auto nc   = vtcs.size();
        if ( nc != 4 )
        {
          trace.error() << "[InterpolatedCorrectedCalculus::buildSharp]"
                        << " restricted to quadrangle faces." << std::endl;
          return mySharp;
        }
        Edge edge_idcs[ 4 ];
        Scalar edge_sign[ 4 ];
        for ( Size i = 0; i < nc; i++ )
        {
          edge_idcs[ i ] =
          myMesh->makeEdge( vtcs[ i ], vtcs[ ( i + 1 ) % nc ] );
          edge_sign[ i ] = vtcs[ i ] < vtcs[ ( i + 1 ) % nc ] ? 1.0 : -1.0;
        }
        const auto G_f = buildLocalSharp( f );
        for ( Dimension x = 0; x < 3; ++x )
          for ( Dimension n = 0; n < nc; ++n )
            triplets.push_back( { (StorageIndex)3 * f + x,
                                  (StorageIndex)edge_idcs[ n ],
                                  edge_sign[ n ] * G_f( x, n ) } );
      }
      mySharp.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return mySharp;
    }

    DenseMatrix buildLocalFlat( Index f ) const
    {
      if ( myMesh->incidentVertices( f ).size() != 4 )
      {
        trace.error() << "[InterpolatedCorrectedCalculus::buildLocalFlat]"
                      << " restricted to quadrangle faces." << std::endl;
        return DenseMatrix();
      }
      return quadrangleFlatLocalMatrix( f );
    }

    LinearOperator buildFlat() const
    {
      Triplets triplets;
      LinearOperator myFlat =
      LinearOperator( myMesh->nbEdges(), 3 * myMesh->nbFaces() );
      for ( Face f = 0; f < myMesh->nbFaces(); ++f )
      {
        const auto vtcs = myMesh->incidentVertices( f );
        const auto nc   = vtcs.size();
        if ( nc != 4 )
        {
          trace.error() << "[InterpolatedCorrectedCalculus::buildFlat]"
                        << " restricted to quadrangle faces." << std::endl;
          return myFlat;
        }
        Edge edge_idcs[ 4 ];
        Scalar edge_sign[ 4 ];
        for ( Size i = 0; i < nc; i++ )
        {
          edge_idcs[ i ] =
          myMesh->makeEdge( vtcs[ i ], vtcs[ ( i + 1 ) % nc ] );
          edge_sign[ i ] = vtcs[ i ] < vtcs[ ( i + 1 ) % nc ] ? 1.0 : -1.0;
        }
        const auto V_f = buildLocalFlat( f );
        for ( Dimension x = 0; x < 3; ++x )
          for ( Dimension n = 0; n < nc; ++n )
          {
            const double nbFaces = myMesh->edgeFaces( edge_idcs[ n ] ).size();
            triplets.push_back( { (StorageIndex)edge_idcs[ n ],
                                  (StorageIndex)3 * f + x,
                                  edge_sign[ n ] * V_f( n, x ) / nbFaces } );
          }
      }
      myFlat.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return myFlat;
    }

    DenseMatrix buildLocalD0( Index f ) const
    {
      const auto nc   = myMesh->incidentVertices( f ).size();
      DenseMatrix res = DenseMatrix::Zero( nc, nc );
      for ( int i = 0; i < nc; i++ )
      {
        res( ( i - 1 + nc ) % nc, i ) = 1;
        res( i, i )                   = -1;
      }
      return res;
    }

    LinearOperator buildD0() const
    {
      Triplets triplets;
      // trace.beginBlock( "Init derivative operator D0" );
      Edge e = 0;
      for ( auto && vtcs : myMesh->allEdgeVertices() )
      {
        triplets.push_back( { (StorageIndex)e, (StorageIndex)vtcs.first, -1 } );
        triplets.push_back( { (StorageIndex)e, (StorageIndex)vtcs.second, 1 } );
        e++;
      }
      LinearOperator myD0 =
      LinearOperator( myMesh->nbEdges(), myMesh->nbVertices() );
      myD0.setFromTriplets( triplets.cbegin(), triplets.cend() );
      return myD0;
    }

    DenseMatrix buildLocalL0( Index f ) const
    {
      DenseMatrix D0 = buildLocalD0( f );
      DenseMatrix M1 = buildLocalM1( f );
      return -1.0 * D0.transpose() * M1 * D0;
    }

    LinearOperator buildL0() const
    {
      LinearOperator myD0 = buildD0();
      LinearOperator myM1 = buildM1();
      return -1.0 * myD0.transpose() * myM1 * myD0;
    }
  };

} // namespace DGtal

#endif
