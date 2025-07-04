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
 */
/**
 * @file tangency-reconstruction.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2022/07/15
 *
 * This file is part of the DGtal library.
 */
#include <iostream>
#include <string>
#include <DGtal/base/Common.h>
#include <DGtal/helpers/StdDefs.h>
#include <DGtal/helpers/Shortcuts.h>
#include <DGtal/helpers/ShortcutsGeometry.h>
#include <DGtal/shapes/SurfaceMesh.h>
#include <DGtal/geometry/volumes/TangencyComputer.h>
#include <DGtal/geometry/volumes/ConvexityHelper.h>
#include <DGtal/geometry/tools/QuickHull.h>
#include <DGtal/geometry/surfaces/estimation/PlaneProbingTetrahedronEstimator.h>

#include <polyscope/pick.h>
#include <polyscope/polyscope.h>
#include <polyscope/surface_mesh.h>
#include <polyscope/point_cloud.h>


using namespace DGtal;
using namespace Z3i;

// Using standard 3D digital space.
typedef Shortcuts<Z3i::KSpace>         SH3;
typedef ShortcutsGeometry<Z3i::KSpace> SHG3;
// The following typedefs are useful
typedef SurfaceMesh< RealPoint, RealVector >  SurfMesh;
typedef ConvexityHelper<3>                    CvxHelper;
typedef SurfMesh::Face                        Face;
typedef SurfMesh::Vertex                      Vertex;
typedef DGtal::ConvexHullIntegralKernel< 3 >  Kernel3D;
typedef DGtal::QuickHull< Kernel3D >          QuickHull3D;
typedef std::size_t Size;
//typedef Z3i::KSpace::SCell SCell;
//Polyscope global
polyscope::SurfaceMesh *psMesh;
polyscope::SurfaceMesh *psDualMesh;
polyscope::SurfaceMesh *psOuterDualMesh;
polyscope::SurfaceMesh *psTriangle;
polyscope::SurfaceMesh *psDelaunay;
polyscope::PointCloud*  psCloud;
polyscope::PointCloud*  psCloudRemaining;
polyscope::PointCloud*  psCloudCvx;
polyscope::PointCloud*  psCloudProc;
polyscope::PointCloud*  psOuterCloudProc;
polyscope::PointCloud*  psCloudInnerDelaunay;

SurfMesh surfmesh;
SurfMesh dual_surfmesh;
SurfMesh outer_dual_surfmesh;
float gridstep   = 1.0;
int   vertex_idx = -1;
float Time = 0.0;
int   nb_cones = 10;
bool remove_empty_cells = false;
bool local_tangency     = false;

Size current = 0;
Size outer_current = 0;
std::vector< Size >      order;
std::vector< Size >      outer_order;
std::vector< Point >     digital_points;
std::vector< Point >     outer_digital_points;
std::set< Point >        useless_points;
std::vector< double >    intercepts;
std::vector< double >    outer_intercepts;
std::vector<std::vector<SH3::SurfaceMesh::Vertex>> dual_faces;
std::vector<RealPoint>   dual_positions;
std::vector<std::vector<SH3::SurfaceMesh::Vertex>> outer_dual_faces;
std::vector<RealPoint>   outer_dual_positions;
std::vector<SCell> surfels;
std::map<Point,Size> surfel2idx;

KSpace K;
DGtal::DigitalConvexity< KSpace > dconv;
DGtal::TangencyComputer< KSpace > TC;
DGtal::TangencyComputer< KSpace > outer_TC;
DGtal::LatticeSetByIntervals< Space > LS;
DGtal::LatticeSetByIntervals< Space > outer_LS;

typedef DGtal::TangencyComputer< KSpace >::Index Index;
typedef std::vector< Index >  Indices;
typedef double                Scalar;
typedef std::vector< Scalar > Scalars;

int pickPoint()
{
  if ( order.size() != digital_points.size() )
    {
      auto rng = std::default_random_engine {};
      order.resize( digital_points.size() );
      for ( Size i = 0; i < order.size(); i++ ) order[ i ] = i;
      std::shuffle( order.begin(), order.end(), rng);
      current = 0;
    }
  if ( current == order.size() ) current = 0;
  return static_cast<int>(order[ current++ ]);
}
int pickOuterPoint()
{
  if ( outer_order.size() != outer_digital_points.size() )
    {
      auto rng = std::default_random_engine {};
      outer_order.resize( outer_digital_points.size() );
      for ( Size i = 0; i < outer_order.size(); i++ ) outer_order[ i ] = i;
      std::shuffle( outer_order.begin(), outer_order.end(), rng);
      outer_current = 0;
    }
  if ( outer_current == outer_order.size() ) outer_current = 0;
  return static_cast<int>(outer_order[ outer_current++ ]);
}

// ----------------------------------------------------------------------
// utilities pointel
Point pointelRealPoint2Point( RealPoint p )
{
  RealPoint sp = RealPoint( round( p[ 0 ] / gridstep + 0.5 ),
                            round( p[ 1 ] / gridstep + 0.5 ),
                            round( p[ 2 ] / gridstep + 0.5 ) );
  return Point( sp[ 0 ], sp[ 1 ], sp[ 2 ] );
}
RealPoint pointelPoint2RealPoint( Point q )
{
  return RealPoint( gridstep * ( q[ 0 ] - 0.5 ),
                    gridstep * ( q[ 1 ] - 0.5 ),
                    gridstep * ( q[ 2 ] - 0.5 ) );
}
void embedPointels( const std::vector< Point >& vq, std::vector< RealPoint >& vp )
{
  vp.resize( vq.size() );
  for ( auto i = 0; i < vp.size(); ++i )
    vp[ i ] = pointelPoint2RealPoint( vq[ i ] );
}
void digitizePointels( const std::vector< RealPoint >& vp, std::vector< Point >& vq )
{
  vq.resize( vp.size() );
  for ( auto i = 0; i < vq.size(); ++i )
    vq[ i ] = pointelRealPoint2Point( vp[ i ] );
}

// ----------------------------------------------------------------------
// utilities voxel
Point voxelRealPoint2Point( RealPoint p )
{
  RealPoint sp = RealPoint( round( p[ 0 ] / gridstep ),
                            round( p[ 1 ] / gridstep ),
                            round( p[ 2 ] / gridstep ) );
  return Point( sp[ 0 ], sp[ 1 ], sp[ 2 ] );
}
RealPoint voxelPoint2RealPoint( Point q )
{
  return RealPoint( gridstep * ( q[ 0 ] ),
                    gridstep * ( q[ 1 ] ),
                    gridstep * ( q[ 2 ] ) );
}
void embedVoxels( const std::vector< Point >& vq, std::vector< RealPoint >& vp )
{
  vp.resize( vq.size() );
  for ( auto i = 0; i < vp.size(); ++i )
    vp[ i ] = voxelPoint2RealPoint( vq[ i ] );
}
void digitizeVoxels( const std::vector< RealPoint >& vp, std::vector< Point >& vq )
{
  vq.resize( vp.size() );
  for ( auto i = 0; i < vq.size(); ++i )
    vq[ i ] = voxelRealPoint2Point( vp[ i ] );
}

// @return the indices of all the points of X different from p that are cotangent to p.
Indices tangentCone( const Point& p )
{
  return TC.getCotangentPoints( p );
}

Scalars distances( const Point& p, const Indices& idx )
{
  Scalars D( idx.size() );
  for ( Index i = 0; i < idx.size(); i++ )
    D[ i ] = ( TC.point( i ) - p ).norm();
  return D;
}

///////////////////////////////////////////////////////////////////////////////
std::vector< Point >
findCorners( const std::unordered_set< Point >& S,
             const std::vector< Vector >& In,
             const std::vector< Vector >& Out )
{
  std::vector< Point > C;
  for ( auto&& p : S )
    {
      bool corner = true;
      for ( auto&& n : In )
        if ( ! S.count( p+n ) ) { corner = false; break; }
      if ( ! corner ) continue;
      for ( auto&& n : Out )
        if ( S.count( p+n ) ) { corner = false; break; }
      if ( corner ) C.push_back( p );
    }
  return C;
}

void computeQuadrant( int q,
                      std::vector< Vector >& In,
                      std::vector< Vector >& Out )
{
  In.clear();
  Out.clear();
  In.push_back( Vector( q & 0x1 ? 1 : -1, 0, 0 ) );
  In.push_back( Vector( 0, q & 0x2 ? 1 : -1, 0 ) );
  In.push_back( Vector( 0, 0, q & 0x4 ? 1 : -1 ) );
  Out.push_back( Vector( q & 0x1 ? 1 : -1, q & 0x2 ? 1 : -1, q & 0x4 ? 1 : -1 ) );
  Vector D = ( In[ 1 ] - In[ 0 ] ).crossProduct( In[ 2 ] - In[ 0 ] );
  if ( D.dot( Out[ 0 ] ) < 0.0 ) std::swap( In[ 1 ], In[ 2 ] );
  In.push_back( In[ 0 ]+In[ 1 ] );
  In.push_back( In[ 0 ]+In[ 2 ] );
  In.push_back( In[ 1 ]+In[ 2 ] );
}

struct UnorderedPointSetPredicate
{
  typedef DGtal::Z3i::Point Point;
  const std::unordered_set< Point >* myS;
  explicit UnorderedPointSetPredicate( const std::unordered_set< Point >& S )
    : myS( &S ) {}
  bool operator()( const Point& p ) const
  { return myS->count( p ) != 0; }
};

void computePlanes()
{
  // - mode specifies the candidate set, it is one of { ProbingMode::H, ProbingMode::R, ProbingMode::R1 }.
  using Estimator
    = DGtal::PlaneProbingTetrahedronEstimator< UnorderedPointSetPredicate,
                                               ProbingMode::R >;
  trace.beginBlock( "Compute planes" );
  std::vector< RealPoint > positions;
  std::vector< Point >     vertices;
  std::vector< std::vector<SH3::SurfaceMesh::Vertex> > faces;
  std::vector< Vector >  In;
  std::vector< Vector > Out;
  std::unordered_set< Point > S( digital_points.cbegin(), digital_points.cend() );
  UnorderedPointSetPredicate predS( S );
  Index i = 0;
  for ( int q = 0; q < 8; q++ ) // for each quadrant
    {
      computeQuadrant( q, In, Out );
      std::vector< Point > corners = findCorners( S, In, Out );
      std::cout << "Found " << corners.size() << " in Q" << q << std::endl;
      std::array<Point, 3> m = { In[ 0 ], In[ 1 ], In[ 2 ] };
      for ( auto&& p : corners )
        {
          Estimator estimator( p, m, predS );
          // if ( estimator.hexagonState()
          //      != Estimator::Neighborhood::HexagonState::Planar)
          //   continue;
          std::vector<SH3::SurfaceMesh::Vertex> triangle { i, i+1, i+2 };
          auto v = estimator.vertices();
          faces.push_back( triangle );
          vertices.push_back( v[ 0 ] );
          vertices.push_back( v[ 1 ] );
          vertices.push_back( v[ 2 ] );
          while (estimator.advance().first) {
            auto state = estimator.hexagonState();
            if (state == Estimator::Neighborhood::HexagonState::Planar) {
              auto v = estimator.vertices();
              if ( S.count( v[ 0 ] ) && S.count( v[ 1 ] ) && S.count( v[ 2 ] ) )
                {
                  std::vector< Point > X { v[ 0 ], v[ 1 ], v[ 2 ] };
                  auto P = dconv.makePolytope( X );
                  if ( dconv.isFullySubconvex( P, TC.cellCover() ) )
                      // // && TC.arePointsCotangent( v[ 0 ], v[ 1 ], v[ 2 ] ) ) 
                    {
                      vertices[ i   ] = v[ 0 ];
                      vertices[ i+1 ] = v[ 1 ];
                      vertices[ i+2 ] = v[ 2 ];
                    }
                }
            }          
          }
          i += 3;
          // }
        }
    }
  Time = trace.endBlock();
  embedPointels( vertices, positions );
  psTriangle = polyscope::registerSurfaceMesh("Triangle", positions, faces);
}

void displayMidReconstruction()
{
  std::vector< RealPoint > mid_dual_positions( dual_positions.size() );
  for ( auto i = 0; i < surfels.size(); i++ )
    {
      auto int_vox = K.interiorVoxel( surfels[ i ] );
      auto ext_vox = K.exteriorVoxel( surfels[ i ] );
      auto int_p   = voxelPoint2RealPoint( int_vox ); //K.uCoords( int_vox ) );
      auto ext_p   = voxelPoint2RealPoint( ext_vox ); //K.uCoords( ext_vox ) );
      const double     s = intercepts[ i ] + 0.001; 
      const RealPoint  q = ( 1.0 - s ) * int_p + s * ext_p;
      const double    ds = outer_intercepts[ i ] + 0.001; 
      const RealPoint dq = ( 1.0 - ds ) * ext_p + ds * int_p;
      mid_dual_positions[ i ] = 0.5*(q+dq);
    }
  psDualMesh = polyscope::registerSurfaceMesh("Mid surface", mid_dual_positions, dual_faces);
}

void displayReconstruction()
{
  for ( auto i = 0; i < surfels.size(); i++ )
    {
      auto int_vox = K.interiorVoxel( surfels[ i ] );
      auto ext_vox = K.exteriorVoxel( surfels[ i ] );
      auto int_p   = voxelPoint2RealPoint( int_vox ); //K.uCoords( int_vox ) );
      auto ext_p   = voxelPoint2RealPoint( ext_vox ); //K.uCoords( ext_vox ) );
      const double    s = intercepts[ i ] + 0.001; 
      const RealPoint q = ( 1.0 - s ) * int_p + s * ext_p;
      dual_positions[ i ] = q;
    }
  psDualMesh = polyscope::registerSurfaceMesh("Reconstruction surface", dual_positions, dual_faces);
  std::vector< Point > X;
  for ( Size i = 0; i < current; i++ )
    X.push_back( digital_points[ order[ i ] ] );
  std::vector< RealPoint > emb_X;
  embedVoxels( X, emb_X );
  psCloudProc = polyscope::registerPointCloud( "Processed points", emb_X );
  psCloudProc->setPointRadius( gridstep / 300.0 );

}

void displayRemainingPoints()
{
  std::vector< Point > X;
  std::vector< RealPoint > emb_X;
  for ( auto i = current; i < digital_points.size(); i++ )
    {
      Point p = digital_points[ order[ i ] ];
      if ( ! useless_points.count( p ) )
        X.push_back( p );
    }
  embedVoxels( X, emb_X );
  psCloudProc = polyscope::registerPointCloud( "Remaining points", emb_X );
  psCloudProc->setPointRadius( gridstep / 200.0 );
}

void displayOuterReconstruction()
{
  for ( auto i = 0; i < surfels.size(); i++ )
    {
      auto int_vox = K.interiorVoxel( surfels[ i ] );
      auto ext_vox = K.exteriorVoxel( surfels[ i ] );
      auto int_p   = voxelPoint2RealPoint( int_vox ); //K.uCoords( int_vox ) );
      auto ext_p   = voxelPoint2RealPoint( ext_vox ); //K.uCoords( ext_vox ) );
      const double    s = outer_intercepts[ i ] + 0.001; 
      const RealPoint q = ( 1.0 - s ) * ext_p + s * int_p;
      outer_dual_positions[ i ] = q;
    }
  psOuterDualMesh = polyscope::registerSurfaceMesh("Reconstruction outer surface", outer_dual_positions, outer_dual_faces);
  std::vector< Point > X;
  for ( Size i = 0; i < outer_current; i++ )
    X.push_back( outer_digital_points[ outer_order[ i ] ] );
  std::vector< RealPoint > emb_X;
  embedVoxels( X, emb_X );
  psOuterCloudProc = polyscope::registerPointCloud( "Processed outer points", emb_X );
  psOuterCloudProc->setPointRadius( gridstep / 300.0 );

}

void updateReconstructionFromCells( const std::vector< Point >& X,
                                    const std::vector< Point >& cells )
{
  // Compute plane
  const Vector N = ( X[ 1 ] - X[ 0 ] ).crossProduct( X[ 2 ] - X[ 0 ] );
  const auto   a = N.dot( X[ 0 ] );
  // Extract 1-cells which are dual to surfels
  for ( auto&& kp : cells )
    {
      // Look for dimension 1 cells.
      const Cell c = K.uCell( kp );
      if ( K.uDim( c ) != 1 ) continue;
      // Compute dual surfel
      const Dimension t = *K.uDirs( c );
      const Cell p0 = K.uIncident( c, t, false );
      const Cell p1 = K.uIncident( c, t, true );
      const Point dual_kp = K.uCoords( p0 ) + K.uCoords( p1 ) + Point::diagonal(1);
      const auto it = surfel2idx.find( dual_kp );
      if ( it == surfel2idx.cend() ) continue;
      // Compute and update intercept
      const Size  idx     = it->second;
      const SCell surfel  = surfels[ idx ];
      const Point int_vox = K.interiorVoxel( surfel );
      const Point ext_vox = K.exteriorVoxel( surfel );
      const auto  int_val = N.dot( int_vox );
      const auto  ext_val = N.dot( ext_vox );
      // std::cout << " int_val=" << int_val << " a=" << a << " ext_val=" << ext_val;
      if ( ( int_val <= a && ext_val <= a ) || ( int_val >= a && ext_val >= a ) )
        {
          if ( ( int_val < a && ext_val < a ) || ( int_val > a && ext_val > a ) )
            trace.warning() << "Bad intersection" << std::endl;
          continue;
        }
      const double s     = (double)( a - int_val ) / (double) (ext_val - int_val );
      const double old_s = intercepts[ idx ];
      // if ( old_s < s ) std::cout  << " s=" << old_s << " -> " << s << std::endl;
      intercepts[ idx ] = std::max( old_s, s );
    }
}

void updateOuterReconstructionFromCells( const std::vector< Point >& X,
                                         const std::vector< Point >& cells )
{
  // Compute plane
  const Vector N = ( X[ 1 ] - X[ 0 ] ).crossProduct( X[ 2 ] - X[ 0 ] );
  const auto   a = N.dot( X[ 0 ] );
  // Extract 1-cells which are dual to surfels
  for ( auto&& kp : cells )
    {
      // Look for dimension 1 cells.
      const Cell c = K.uCell( kp );
      if ( K.uDim( c ) != 1 ) continue;
      // Compute dual surfel
      const Dimension t = *K.uDirs( c );
      const Cell p0 = K.uIncident( c, t, false );
      const Cell p1 = K.uIncident( c, t, true );
      const Point dual_kp = K.uCoords( p0 ) + K.uCoords( p1 ) + Point::diagonal(1);
      const auto it = surfel2idx.find( dual_kp );
      if ( it == surfel2idx.cend() ) continue;
      // Compute and update intercept
      const Size  idx     = it->second;
      const SCell surfel  = surfels[ idx ];
      const Point int_vox = K.interiorVoxel( surfel );
      const Point ext_vox = K.exteriorVoxel( surfel );
      const auto  int_val = N.dot( int_vox );
      const auto  ext_val = N.dot( ext_vox );
      if ( ( int_val <= a && ext_val <= a ) || ( int_val >= a && ext_val >= a ) )
        {
          if ( ( int_val < a && ext_val < a ) || ( int_val > a && ext_val > a ) )
            trace.warning() << "Bad intersection" << std::endl;
          continue;
        }
      const double s = (double)( a - ext_val ) / (double) (int_val - ext_val );
      outer_intercepts[ idx ] = std::max( outer_intercepts[ idx ], s );
    }
}

void updateReconstructionFromCells( Point x, Point y,
                                    const std::vector< Point >& cells )
{
  // Compute plane
  const Vector     U = y - x;
  if ( U == Vector::zero ) return;
  const double     l = U.norm();
  const RealVector u = U.getNormalized();
  const RealPoint  p( x[ 0 ], x[ 1 ], x[ 2 ] );
  // Extract 1-cells which are dual to surfels
  for ( auto&& kp : cells )
    {
      // Look for dimension 1 cells.
      const Cell c = K.uCell( kp );
      if ( K.uDim( c ) != 1 ) continue;
      // Compute dual surfel
      const Dimension r = *K.uDirs( c );
      const Cell p0 = K.uIncident( c, r, false );
      const Cell p1 = K.uIncident( c, r, true );
      const Point dual_kp = K.uCoords( p0 ) + K.uCoords( p1 ) + Point::diagonal(1);
      const auto it = surfel2idx.find( dual_kp );
      if ( it == surfel2idx.cend() ) continue;
      // Compute and update intercept
      const Size  idx     = it->second;
      const SCell surfel  = surfels[ idx ];
      const Point int_vox = K.interiorVoxel( surfel );
      const Point ext_vox = K.exteriorVoxel( surfel );
      const Vector V      = ext_vox - int_vox;
      const RealVector v  = V.getNormalized();
      const RealPoint  q( int_vox[ 0 ], int_vox[ 1 ], int_vox[ 2 ] );
      const auto   uv     = u.dot( v );
      if ( uv == 0 ) continue;
      // Solving system to get closest points.
      const auto   c1     = ( q - p ).dot( u );
      const auto   c2     = ( p - q ).dot( v );
      const double d      =  1.0-uv*uv;
      const double s      = ( c1 + uv * c2 ) / d; // on [xy]
      const double t      = ( c2 + uv * c1 ) / d; // on linel
      if ( ( s < 0.0 ) || ( s > l ) ) continue;
      // if ( ( t < 0.0 ) || ( t > 1.0 ) ) continue;
      intercepts[ idx ] = std::max( intercepts[ idx ], std::min( t, 1.0 ) );
    }
}

void updateReconstructionFromTangentConeLines( int vertex_idx )
{
  if ( digital_points.empty() ) return;
  if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  // trace.beginBlock( "Compute tangent cone" );
  auto local_X_idx = TC.getCotangentPoints( p );
  std::vector< Point > local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( TC.point( idx ) );
  for ( auto&& q : local_X )
    {
      std::vector< Point > X { p, q };
      const auto line_cells = dconv.StarCvxH( X, LS.axis() );
      updateReconstructionFromCells( p, q, line_cells.toPointRange() );
    }
}

void updateReconstructionFromTangentConeTriangles( int vertex_idx )
{
  typedef QuickHull3D::IndexRange IndexRange;
  if ( digital_points.empty() ) return;
  if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  // trace.beginBlock( "Compute tangent cone" );
  auto local_X_idx = TC.getCotangentPoints( p );
  local_X_idx.push_back( vertex_idx );
  std::vector< Point > local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( TC.point( idx ) );
  QuickHull3D hull;
  hull.setInput( local_X, false );
  hull.computeConvexHull();
  std::vector< Point > positions;
  hull.getVertexPositions( positions );
  
  std::vector< IndexRange > facet_vertices;
  bool ok = hull.getFacetVertices( facet_vertices );
  if ( ! ok ) trace.error() << "Bad facet computation" << std::endl;
  // Update from all cones
  std::set< std::pair< Point, Point > > edges;
  for ( auto&& facet : facet_vertices )
    {
      const auto nb = facet.size();
      for ( auto i = 0; i < nb; i++ )
        edges.insert( std::make_pair( positions[ facet[ i ] ],
                                      positions[ facet[ (i+1)%nb ] ] ) );
    }
  for ( auto&& e : edges )
    {
      if ( e.second < e.first ) continue;
      std::vector< Point > X { p, e.first, e.second };
      const auto triangle_cells = dconv.StarCvxH( X, LS.axis() );
      if ( LS.includes( triangle_cells ) ) // tangent to shape
        updateReconstructionFromCells( X, triangle_cells.toPointRange() );
    }

  // Miss features
  // // Update from all facets
  // for ( auto&& facet : facet_vertices )
  //   {
  //     const auto nb = facet.size();
  //     for ( auto i = 0; i < nb; i++ )
  //       for ( auto j = i+1; j < nb; j++ )
  //         for ( auto k = j+1; k < nb; k++ )
  //           {
  //             std::vector< Point > X
  //               { positions[ facet[ i ] ],
  //                 positions[ facet[ j ] ],
  //                 positions[ facet[ k ] ] };
  //             const auto triangle_cells = dconv.StarCvxH( X, LS.axis() );
  //             if ( LS.includes( triangle_cells ) ) // tangent to shape
  //               updateReconstructionFromCells( X, triangle_cells.toPointRange() );
  //           }
  //   }

  // Too costly
  // // Update from all possible triplets
  // const auto nb = positions.size();
  // for ( auto i = 0; i < nb; i++ )
  //   for ( auto j = i+1; j < nb; j++ )
  //     for ( auto k = j+1; k < nb; k++ )
  //       {
  //         std::vector< Point > X { positions[ i ], positions[ j ], positions[ k ] };
  //         const auto triangle_cells = dconv.StarCvxH( X, LS.axis() );
  //         if ( LS.includes( triangle_cells ) ) // tangent to shape
  //           updateReconstructionFromCells( X, triangle_cells.toPointRange() );
  //       }
  // Time = trace.endBlock();
}


///////////////////////////////////////////////////////////////////////////////
void computeTangentCone( int vertex_idx)
{
  if ( digital_points.empty() ) return;
  // if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  // trace.beginBlock( "Compute tangent cone" );
  auto local_X_idx = TC.getCotangentPoints( p );
  std::cout << "#cone=" << local_X_idx.size() << std::endl;
  local_X_idx.push_back( vertex_idx );
  std::vector< Point > local_X;
  std::vector< RealPoint > emb_local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( TC.point( idx ) );
  std::vector< double > values( local_X.size(), 0.0 );
  values.back() = 1.0;
  embedVoxels( local_X, emb_local_X );
  psCloud = polyscope::registerPointCloud( "Tangent cone", emb_local_X );
  psCloud->setPointRadius( gridstep / 300.0 );
  psCloud->addScalarQuantity( "Classification", values );
  QuickHull3D hull;
  hull.setInput( local_X, false );
  hull.computeConvexHull();
  std::vector< Point > positions;
  std::vector< RealPoint > emb_positions;
  hull.getVertexPositions( positions );
  // Time = trace.endBlock();
  embedVoxels( positions, emb_positions );
  psCloudCvx = polyscope::registerPointCloud( "Tangent cone vertices", emb_positions );
  psCloudCvx->setPointRadius( gridstep / 200.0  );

  updateReconstructionFromTangentConeTriangles( vertex_idx );
  displayReconstruction();
}

///////////////////////////////////////////////////////////////////////////////
void  updateReconstructionFromLocalTangentDelaunayComplex( int vertex_idx)
{
  if ( digital_points.empty() ) return;
  // if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  auto local_X_idx = TC.getCotangentPoints( p );
  local_X_idx.push_back( vertex_idx );
  std::vector< Point > local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( TC.point( idx ) );
  DGtal::LatticeSetByIntervals< Space > local_LS;
  if ( local_tangency ) 
    local_LS = DGtal::LatticeSetByIntervals< Space >( local_X.cbegin(), local_X.cend(), 0 ).starOfPoints();

  typedef ConvexCellComplex< Point >::Index       Index;
  ConvexCellComplex< Point > dcomplex;
  bool ok = CvxHelper::computeDelaunayCellComplex( dcomplex, local_X, false );
  if ( ! ok )
    trace.error() << "Input set of points is not full dimensional." << std::endl;
  
  dcomplex.requireFaceGeometry();
  // Filter cells
  std::vector< bool > is_cell_tangent( dcomplex.nbCells(), false );
  if ( remove_empty_cells )
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
        auto Y = dcomplex.cellVertexPositions( c );
        auto P = dconv.makePolytope( Y );
        if ( P.countUpTo( (Integer)Y.size()+1 ) >= (Integer)Y.size()+1 ) continue;
        is_cell_tangent[ c ] =
          local_tangency
          ? dconv.isFullySubconvex( P, local_LS )
          : dconv.isFullySubconvex( P, LS );
      }
  else
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
      	auto Y = dcomplex.cellVertexPositions( c );
        is_cell_tangent[ c ] =
          local_tangency
          ? dconv.isFullySubconvex( Y, local_LS )
          : dconv.isFullySubconvex( Y, LS );
      }
  // Get faces
  std::vector< std::vector<SH3::SurfaceMesh::Vertex> > del_faces;
  std::vector< RealPoint >   del_positions;
  std::set<Index> boundary_or_ext_points;
  for ( Index f = 0; f < dcomplex.nbFaces(); ++f )
    {
      auto f0 = std::make_pair( f, false );
      auto c0 = dcomplex.faceCell( f0 );
      auto f1 = dcomplex.opposite( f0 );
      auto c1 = dcomplex.faceCell( f1 );
      if ( dcomplex.isInfinite( c0 ) )
        {
          std::swap( f0, f1 );
          std::swap( c0, c1 );
        }
      if ( ! is_cell_tangent[ c0 ] )
        {
          auto V = dcomplex.cellVertices( c0 );
          boundary_or_ext_points.insert( V.cbegin(), V.cend() );
        }
      if ( ! dcomplex.isInfinite( c1 ) && ! is_cell_tangent[ c1 ] )
        {
          auto V = dcomplex.cellVertices( c1 );
          boundary_or_ext_points.insert( V.cbegin(), V.cend() );
        }
      bool bdry =
        ( is_cell_tangent[ c0 ] && ( dcomplex.isInfinite( c1 )
                                     || ( ! is_cell_tangent[ c1 ] ) ) )
        ||
        ( ! is_cell_tangent[ c0 ] && ( ! dcomplex.isInfinite( c1 )
                                       && ( is_cell_tangent[ c1 ] ) ) );
      if ( ! bdry ) continue;
      std::vector< Point > X = dcomplex.faceVertexPositions( f0 );
      const auto triangle_cells = dconv.StarCvxH( X, LS.axis() );
      //if ( LS.includes( triangle_cells ) ) // tangent to shape
      updateReconstructionFromCells( X, triangle_cells.toPointRange() );
    }
  useless_points.insert( p );
  for ( Index v = 0; v < dcomplex.nbVertices(); v++ )
    {
      auto q = dcomplex.position( v );
      if ( ! boundary_or_ext_points.count( v ) )
        useless_points.insert( q );
    }
}


///////////////////////////////////////////////////////////////////////////////
void updateOuterReconstructionFromLocalTangentDelaunayComplex( int vertex_idx)
{
  if ( outer_digital_points.empty() ) return;
  // if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = outer_digital_points[ vertex_idx ];
  auto local_X_idx = outer_TC.getCotangentPoints( p );
  local_X_idx.push_back( vertex_idx );
  std::vector< Point > local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( outer_TC.point( idx ) );

  typedef ConvexCellComplex< Point >::Index       Index;
  ConvexCellComplex< Point > dcomplex;
  bool ok = CvxHelper::computeDelaunayCellComplex( dcomplex, local_X, false );
  if ( ! ok )
    trace.error() << "Input set of points is not full dimensional." << std::endl;
  
  dcomplex.requireFaceGeometry();
  // Filter cells
  std::vector< bool > is_cell_tangent( dcomplex.nbCells(), false );
  if ( remove_empty_cells )
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
        auto Y = dcomplex.cellVertexPositions( c );
        auto P = dconv.makePolytope( Y );
        if ( P.countUpTo( (Integer)Y.size()+1 ) >= (Integer)Y.size()+1 ) continue;
        is_cell_tangent[ c ] = dconv.isFullySubconvex( P, outer_LS );
      }
  else
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
      	auto Y = dcomplex.cellVertexPositions( c );
        is_cell_tangent[ c ] = dconv.isFullySubconvex( Y, outer_LS );
      }
  // Get faces
  std::vector< std::vector<SH3::SurfaceMesh::Vertex> > del_faces;
  for ( Index f = 0; f < dcomplex.nbFaces(); ++f )
    {
      auto f0 = std::make_pair( f, false );
      auto c0 = dcomplex.faceCell( f0 );
      auto f1 = dcomplex.opposite( f0 );
      auto c1 = dcomplex.faceCell( f1 );
      if ( dcomplex.isInfinite( c0 ) )
        {
          std::swap( f0, f1 );
          std::swap( c0, c1 );
        }
      bool bdry =
        ( is_cell_tangent[ c0 ] && ( dcomplex.isInfinite( c1 )
                                     || ( ! is_cell_tangent[ c1 ] ) ) )
        ||
        ( ! is_cell_tangent[ c0 ] && ( ! dcomplex.isInfinite( c1 )
                                       && ( is_cell_tangent[ c1 ] ) ) );
      if ( ! bdry ) continue;
      std::vector< Point > X = dcomplex.faceVertexPositions( f0 );
      const auto triangle_cells = dconv.StarCvxH( X, outer_LS.axis() );
      //if ( LS.includes( triangle_cells ) ) // tangent to shape
      updateOuterReconstructionFromCells( X, triangle_cells.toPointRange() );
    }
}

///////////////////////////////////////////////////////////////////////////////
void computeLocalTangentDelaunayComplex( int vertex_idx)
{
  if ( digital_points.empty() ) return;
  // if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  trace.beginBlock( "Compute tangent cone" );
  auto local_X_idx = TC.getCotangentPoints( p );
  std::cout << "#cone=" << local_X_idx.size() << std::endl;
  local_X_idx.push_back( vertex_idx );
  std::vector< Point > local_X;
  for ( auto idx : local_X_idx )
    local_X.push_back( TC.point( idx ) );
  DGtal::LatticeSetByIntervals< Space > local_LS;
  if ( local_tangency ) 
    local_LS = DGtal::LatticeSetByIntervals< Space >( local_X.cbegin(), local_X.cend(), 0 ).starOfPoints();

  typedef ConvexCellComplex< Point >::Index       Index;
  ConvexCellComplex< Point > dcomplex;
  bool ok = CvxHelper::computeDelaunayCellComplex( dcomplex, local_X, false );
  if ( ! ok )
    trace.error() << "Input set of points is not full dimensional." << std::endl;
  dcomplex.requireFaceGeometry();
  
  // Filter cells
  std::vector< bool > is_cell_tangent( dcomplex.nbCells(), false );
  if ( remove_empty_cells )
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
        auto Y = dcomplex.cellVertexPositions( c );
        auto P = dconv.makePolytope( Y );
        if ( P.countUpTo( (Integer)Y.size()+1 ) >= (Integer)Y.size()+1 ) continue;
        is_cell_tangent[ c ] =
          local_tangency
          ? dconv.isFullySubconvex( P, local_LS )
          : dconv.isFullySubconvex( P, LS );
      }
  else
    for ( Index c = 0; c < dcomplex.nbCells(); ++c )
      {
      	auto Y = dcomplex.cellVertexPositions( c );
        is_cell_tangent[ c ] =
          local_tangency
          ? dconv.isFullySubconvex( Y, local_LS )
          : dconv.isFullySubconvex( Y, LS );
      }
  // Get faces
  std::vector< std::vector<SH3::SurfaceMesh::Vertex> > del_faces;
  std::vector< RealPoint >   del_positions;
  std::vector< RealPoint >   del_inner_points;
  std::set<Index> boundary_or_ext_points;
  for ( Index f = 0; f < dcomplex.nbFaces(); ++f )
    {
      auto f0 = std::make_pair( f, false );
      auto c0 = dcomplex.faceCell( f0 );
      auto f1 = dcomplex.opposite( f0 );
      auto c1 = dcomplex.faceCell( f1 );
      if ( dcomplex.isInfinite( c0 ) )
        {
          std::swap( f0, f1 );
          std::swap( c0, c1 );
        }
      if ( ! is_cell_tangent[ c0 ] )
        {
          auto V = dcomplex.cellVertices( c0 );
          boundary_or_ext_points.insert( V.cbegin(), V.cend() );
        }
      if ( ! dcomplex.isInfinite( c1 ) && ! is_cell_tangent[ c1 ] )
        {
          auto V = dcomplex.cellVertices( c1 );
          boundary_or_ext_points.insert( V.cbegin(), V.cend() );
        }
      bool bdry =
        ( is_cell_tangent[ c0 ] && ( dcomplex.isInfinite( c1 )
                                     || ( ! is_cell_tangent[ c1 ] ) ) )
        ||
        ( ! is_cell_tangent[ c0 ] && ( ! dcomplex.isInfinite( c1 )
                                       && ( is_cell_tangent[ c1 ] ) ) );
      if ( bdry ) del_faces.push_back( dcomplex.faceVertices( f0 ) );
    }
  for ( Index v = 0; v < dcomplex.nbVertices(); v++ )
    {
      auto p = dcomplex.position( v );
      del_positions.push_back( voxelPoint2RealPoint( p ) );
      if ( ! boundary_or_ext_points.count( v ) )
        del_inner_points.push_back( voxelPoint2RealPoint( p ) );
    }
  Time = trace.endBlock();
  
  psCloudInnerDelaunay = polyscope::registerPointCloud( "Inner Delaunay points",
                                                        del_inner_points );
  psCloudInnerDelaunay->setPointRadius( gridstep / 200.0 );
  
  psDelaunay = polyscope::registerSurfaceMesh("Delaunay surface",
                                              del_positions, del_faces);
  updateReconstructionFromLocalTangentDelaunayComplex( vertex_idx );
  displayReconstruction();
}

///////////////////////////////////////////////////////////////////////////////
void computeGlobalTangentDelaunayComplex()
{
  if ( digital_points.empty() ) return;
  // if ( vertex_idx < 0 || vertex_idx >= digital_points.size() ) return;
  const auto p = digital_points[ vertex_idx ];
  trace.beginBlock( "Compute global tangent Delaunay complex" );

  typedef ConvexCellComplex< Point >::Index       Index;
  ConvexCellComplex< Point > dcomplex;
  bool ok = CvxHelper::computeDelaunayCellComplex( dcomplex, digital_points, false );
  if ( ! ok )
    trace.error() << "Input set of points is not full dimensional." << std::endl;

  // Reorder vertices of faces
  dcomplex.requireFaceGeometry();
  
  // Filter cells
  std::vector< bool > is_cell_tangent( dcomplex.nbCells(), false );
  for ( Index c = 0; c < dcomplex.nbCells(); ++c )
    {
      auto Y = dcomplex.cellVertexPositions( c );
      auto P = dconv.makePolytope( Y );
      if ( P.countUpTo( (Integer)Y.size()+1 ) >= (Integer)Y.size()+1 ) continue;
      is_cell_tangent[ c ] = dconv.isFullySubconvex( P, LS );
    }
    
  // Get faces
  std::vector< std::vector<SH3::SurfaceMesh::Vertex> > del_faces;
  std::vector< RealPoint >   del_positions;
  std::vector< RealPoint >   del_inner_points;
  std::set<Index> useful_points;
  std::vector< std::pair< Index, bool > > all_bdry_faces;
  for ( Index f = 0; f < dcomplex.nbFaces(); ++f )
    {
      auto f0 = std::make_pair( f, false );
      auto c0 = dcomplex.faceCell( f0 );
      auto f1 = dcomplex.opposite( f0 );
      auto c1 = dcomplex.faceCell( f1 );
      if ( dcomplex.isInfinite( c0 ) )
        {
          std::swap( c0, c1 ); std::swap( f0, f1 );
        }
      const bool inf_c1 = dcomplex.isInfinite( c1 );
      const bool bdry_f0 = is_cell_tangent[ c0 ] &&
        ( inf_c1 || ! is_cell_tangent[ c1 ] );
      const bool bdry_f1 = ( ! is_cell_tangent[ c0 ] )
        && ( ( ! inf_c1 ) && is_cell_tangent[ c1 ] );

      if ( bdry_f0 )
        {
          const auto V = dcomplex.faceVertices( f0 );
          del_faces.push_back( V );
          useful_points.insert( V.cbegin(), V.cend() );
          all_bdry_faces.push_back( f0 );
        }
      else if ( bdry_f1 )
        {
          const auto V = dcomplex.faceVertices( f1 );
          del_faces.push_back( V );
          useful_points.insert( V.cbegin(), V.cend() );
          all_bdry_faces.push_back( f1 );
        }
      else if ( ( ! ( is_cell_tangent[ c0 ] || inf_c1 || is_cell_tangent[ c0 ] )
                  || ( ! is_cell_tangent[ c0 ] && inf_c1 ) )
                && dconv.isFullySubconvex( dcomplex.faceVertexPositions( f0 ), LS ) )
        {
          const auto V = dcomplex.faceVertices( f0 );
          del_faces.push_back( V );
          useful_points.insert( V.cbegin(), V.cend() );
          //del_faces.push_back( dcomplex.faceVertices( f1 ) );
          all_bdry_faces.push_back( f0 );
        }
    }
  for ( Index v = 0; v < dcomplex.nbVertices(); v++ )
    {
      auto p = dcomplex.position( v );
      del_positions.push_back( voxelPoint2RealPoint( p ) );
      if ( ! useful_points.count( v ) )
        {
          del_inner_points.push_back( voxelPoint2RealPoint( p ) );
          useless_points.insert( p );
        }
    }
  Time = trace.endBlock();

  trace.beginBlock( "Update reconstruction" );
  const auto axis = LS.axis();
  for ( auto f : all_bdry_faces )
    {
      std::vector< Point > X = dcomplex.faceVertexPositions( f );
      const auto cells = dconv.StarCvxH( X, axis );
      // std::cout << "(" << f.first << "," << f.second << ") "
      //           << " #X=" << X.size() << " #cells=" << cells.size() << std::endl;
      updateReconstructionFromCells( X, cells.toPointRange() );
    }
  trace.endBlock();
      
  psCloudInnerDelaunay = polyscope::registerPointCloud( "Inner Delaunay points",
                                                        del_inner_points );
  psCloudInnerDelaunay->setPointRadius( gridstep / 200.0 );

  psDelaunay = polyscope::registerSurfaceMesh("Delaunay surface",
                                              del_positions, del_faces);
  displayReconstruction();
}

void myCallback()
{
  // Select a vertex with the mouse
  if (polyscope::haveSelection()) {
    bool goodSelection = false;
    auto selection = polyscope::getSelection();
    auto selectedSurface = static_cast<polyscope::SurfaceMesh*>(selection.structure);
    const auto idx = selection.localIndex;

    // Only authorize selection on the input surface and the reconstruction
    auto surf = polyscope::getSurfaceMesh("Input surface");
    goodSelection = goodSelection || (selectedSurface == surf);
    const auto nv = selectedSurface->nVertices(); 
    // Validate that it its a face index
    if ( goodSelection )
      {
        if ( idx < nv )
          {
            std::ostringstream otext;
            otext << "Selected vertex = " << idx;
            ImGui::Text( "%s", otext.str().c_str() );
            vertex_idx = (Integer)idx;
          }
        else vertex_idx = -1;
      }
  }
  if (ImGui::Button("Compute global tangent Delaunay complex"))
    {
      computeGlobalTangentDelaunayComplex();
    }
  if (ImGui::Button("Compute tangent cone"))
    {
      computeTangentCone( pickPoint() );
      current--;
    }
  if (ImGui::Button("Compute local Delaunay complex"))
    {
      computeLocalTangentDelaunayComplex( pickPoint() );
      current--;
    }
  ImGui::Checkbox( "Remove empty cells", &remove_empty_cells );
  ImGui::Checkbox( "Check local tangency", &local_tangency );
  if (ImGui::Button("Compute planes"))
    computePlanes();
  if (ImGui::Button("Compute reconstructions from triangles"))
    { // todo
      trace.beginBlock( "Compute reconstruction" );
      for ( int i = 0; i < nb_cones; ++i )
        {
          trace.progressBar( (double) i, (double) nb_cones );
          updateReconstructionFromTangentConeTriangles( pickPoint() );
        }
      displayReconstruction();
      Time = trace.endBlock();
    }
  if (ImGui::Button("Compute reconstruction from local Delaunay cplx"))
    { // todo
      trace.beginBlock( "Compute reconstruction" );
      for ( int i = 0; i < nb_cones; ++i )
        {
          trace.progressBar( (double) i, (double) nb_cones );
          auto j = pickPoint();
          if ( ! useless_points.count( digital_points[ j ] ) )
            updateReconstructionFromLocalTangentDelaunayComplex( j );
          if ( current == 0 ) break;
        }
      displayReconstruction();
      Time = trace.endBlock();
    }
  if (ImGui::Button("Compute outer reconstruction from local Delaunay cplx"))
    { // todo
      trace.beginBlock( "Compute outer reconstruction" );
      for ( int i = 0; i < nb_cones; ++i )
        {
          trace.progressBar( (double) i, (double) nb_cones );
          updateOuterReconstructionFromLocalTangentDelaunayComplex( pickOuterPoint() );
        }
      displayOuterReconstruction();
      Time = trace.endBlock();
    }
  if (ImGui::Button("Compute reconstructions from lines"))
    { // todo
      trace.beginBlock( "Compute reconstruction" );
      for ( int i = 0; i < nb_cones; ++i )
        {
          trace.progressBar( (double) i, (double) nb_cones );
          updateReconstructionFromTangentConeLines( pickPoint() );
        }
      displayReconstruction();
      Time = trace.endBlock();
    }
  ImGui::SliderInt("Nb cones for reconstruction", &nb_cones, 1, 100);
  ImGui::Text( "Computation time = %f ms", Time );
  ImGui::Text( "#X = %ld, #P = %ld, #U = %ld",
               digital_points.size(), current, useless_points.size() );
  if (ImGui::Button("Mid reconstructions"))
    displayMidReconstruction();
  if (ImGui::Button("Remaining points"))
    displayRemainingPoints();
}


int main( int argc, char* argv[] )
{
  auto params = SH3::defaultParameters() | SHG3::defaultParameters() |  SHG3::parametersGeometryEstimation();
  params("surfaceComponents", "All");
  if ( argc <= 1 ) return 0;
  bool input_polynomial = argc > 2;
  std::string filename   = std::string( argv[ 1 ] );
  std::string polynomial = std::string( argv[ 1 ] );
  const double h         = argc >= 3 ? atof( argv[ 2 ] ) : 1.0;
  const double bounds    = argc >= 4 ? atof( argv[ 3 ] ) : 10.0;
  
  CountedPtr<SH3::BinaryImage> binary_image ( nullptr );
  if ( input_polynomial )
    {
      params( "polynomial", polynomial );
      params( "gridstep",  h );
      params( "minAABB",  -bounds );
      params( "maxAABB",   bounds );
      params( "offset",    1.0 );
      params( "closed",    1   );
      auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
      auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
      K            = SH3::getKSpace( params );
      binary_image = SH3::makeBinaryImage(digitized_shape,
					  SH3::Domain(K.lowerBound(),K.upperBound()),
					  params );
    }
  else 
    {
      //params( "thresholdMin", vm[ "min" ].as<int>() );
      // params( "thresholdMax", vm[ "max" ].as<int>() );
      binary_image = SH3::makeBinaryImage( filename, params );
      K            = SH3::getKSpace( binary_image );
    }
  
  // auto binary_image    = SH3::makeBinaryImage(filename, params );
  // K                    = SH3::getKSpace( binary_image, params );
  auto surface         = SH3::makeDigitalSurface( binary_image, K, params );
  surfels              = SH3::getSurfelRange( surface, params );
  // compute map surfel -> idx
  for ( Size i = 0; i < surfels.size(); i++ )
    surfel2idx[ K.sKCoords( surfels[ i ] ) ] = i;
  // compute inner points
  std::set< Point > I;
  for ( auto s : surfels )  I.insert( K.interiorVoxel( s ) );
  digital_points = std::vector<Point>( I.cbegin(), I.cend() );
  // compute outer points
  std::set< Point > O;
  for ( auto s : surfels )  O.insert( K.exteriorVoxel( s ) );
  outer_digital_points = std::vector<Point>( O.cbegin(), O.cend() );
  // initializa intercepts
  intercepts       = std::vector< double >( surfels.size(), 0.0 );
  outer_intercepts = std::vector< double >( surfels.size(), 0.0 );
  auto primalSurface   = SH3::makePrimalSurfaceMesh(surface);
  SH3::Surfel2Index s2i;
  auto dualSurface     = SH3::makeDualPolygonalSurface( s2i, surface );  
  //Need to convert the faces
  std::vector<std::vector<SH3::SurfaceMesh::Vertex>> faces;
  std::vector<RealPoint> positions;
  for(auto face= 0 ; face < primalSurface->nbFaces(); ++face)
    faces.push_back(primalSurface->incidentVertices( face ));
  
  //Recasting to vector of vertices
  positions = primalSurface->positions();
  
  surfmesh = SurfMesh(positions.begin(),
                      positions.end(),
                      faces.begin(),
                      faces.end());
  for(auto face= 0 ; face < dualSurface->nbFaces(); ++face)
    dual_faces.push_back( dualSurface->verticesAroundFace( face ));
    
    //Recasting to vector of vertices
  for ( auto vtx = 0; vtx < dualSurface->nbVertices(); ++vtx )
    dual_positions.push_back( dualSurface->position( vtx ) );
    
  dual_surfmesh = SurfMesh(dual_positions.begin(),
                           dual_positions.end(),
                           dual_faces.begin(),
                           dual_faces.end());
  outer_dual_faces     = dual_faces;
  outer_dual_positions = dual_positions;
  outer_dual_surfmesh = SurfMesh(outer_dual_positions.begin(),
                                 outer_dual_positions.end(),
                                 outer_dual_faces.begin(),
                                 outer_dual_faces.end());
  std::cout << surfmesh << std::endl;
  // Make digital surface
  // digitizePointels( positions, digital_points );
  trace.info() << "Inner points has " << digital_points.size() << " points." << std::endl;
  trace.info() << "Outer points has " << outer_digital_points.size() << " points." << std::endl;
  dconv = DGtal::DigitalConvexity< KSpace >( K );
  TC    = DGtal::TangencyComputer< KSpace >( K );
  TC.init( digital_points.cbegin(), digital_points.cend() );
  outer_TC    = DGtal::TangencyComputer< KSpace >( K );
  outer_TC.init( outer_digital_points.cbegin(), outer_digital_points.cend() );
  trace.info() << "#cell_cover = " << TC.cellCover().nbCells() << std::endl;
  trace.info() << "#outer_cell_cover = " << outer_TC.cellCover().nbCells() << std::endl;
  LS    = DGtal::LatticeSetByIntervals< Space >
    ( digital_points.cbegin(), digital_points.cend(), 0 ).starOfPoints();
  trace.info() << "#lattice_cover = " << LS.size() << std::endl;
  outer_LS    = DGtal::LatticeSetByIntervals< Space >
    ( outer_digital_points.cbegin(), outer_digital_points.cend(), 0 ).starOfPoints();
  trace.info() << "#outer_lattice_cover = " << outer_LS.size() << std::endl;
  
  // Initialize polyscope
  polyscope::init();

  psMesh = polyscope::registerSurfaceMesh("Input surface", positions, faces);
  displayReconstruction();
  displayOuterReconstruction();
  // psDualMesh = polyscope::registerSurfaceMesh("Input dual surface", dual_positions, dual_faces);

  
  // Set the callback function
  polyscope::state::userCallback = myCallback;
  polyscope::show();
  return EXIT_SUCCESS;
}
