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

/**
 * @file testConvexityHelper.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class ConvexityHelper.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/volumes/ConvexityHelper.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConvexityHelper in 2D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "ConvexityHelper< 2 > unit tests",
          "[convexity_helper][lattice_polytope][2d]" )
{
  typedef ConvexityHelper< 2 >    Helper;
  typedef Helper::Point           Point;
  typedef ConvexCellComplex< Point > CvxCellComplex;
  GIVEN( "Given a star { (0,0), (-4,-1), (-3,5), (7,3), (5, -2) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(-4,-1), Point(-3,5), Point(7,3), Point(5, -2) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 4 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 4 == 4 );
      }
      THEN( "The polytope is Minkowski summable" ) {
        REQUIRE( P.canBeSummed() );
      }
      THEN( "The polytope contains the input points" ) {
        REQUIRE( P.isInside( V[ 0 ] ) );
        REQUIRE( P.isInside( V[ 1 ] ) );
        REQUIRE( P.isInside( V[ 2 ] ) );
        REQUIRE( P.isInside( V[ 3 ] ) );
        REQUIRE( P.isInside( V[ 4 ] ) );
      }
      THEN( "The polytope contains 58 points" ) {
        REQUIRE( P.count() == 58 );
      }
      THEN( "The interior of the polytope contains 53 points" ) {
        REQUIRE( P.countInterior() == 53 );
      }
      THEN( "The boundary of the polytope contains 5 points" ) {
        REQUIRE( P.countBoundary() == 5 );
      }
    }
  }
  GIVEN( "Given a square with an additional outside vertex " ) {
    std::vector<Point> V
      = { Point(-10,-10), Point(10,-10), Point(-10,10), Point(10,10),
      Point(0,18) };
    WHEN( "Computing its Delaunay cell complex" ){
      CvxCellComplex complex;
      bool ok = Helper::computeDelaunayCellComplex( complex, V, false );
      CAPTURE( complex );
      THEN( "The complex has 2 cells, 6 faces, 5 vertices" ) {
        REQUIRE( ok );
        REQUIRE( complex.nbCells() == 2 );
        REQUIRE( complex.nbFaces() == 6 );
        REQUIRE( complex.nbVertices() == 5 );
      }
      THEN( "The faces of cells are finite" ) {
        bool ok_finite = true;
        for ( auto c = 0; c < complex.nbCells(); ++c ) {
          const auto faces = complex.cellFaces( c );
          for ( auto f : faces )
            ok_finite = ok_finite && ! complex.isInfinite( complex.faceCell( f ) );
        }
        REQUIRE( ok_finite );
      }
      THEN( "The opposite of faces of cells are infinite except two" ) {
        int  nb_finite   = 0;
        for ( auto c = 0; c < complex.nbCells(); ++c ) {
          const auto faces = complex.cellFaces( c );
          for ( auto f : faces ) {
            const auto opp_f = complex.opposite( f );
            nb_finite += complex.isInfinite( complex.faceCell( opp_f ) ) ? 0 : 1;
          }
        }
        REQUIRE( nb_finite == 2 );
      }
    }
  }
  GIVEN( "Given a degenerated polytope { (0,0), (3,-1), (9,-3), (-6,2) } " ) {
    std::vector<Point> V
      = { Point(0,0), Point(3,-1), Point(9,-3), Point(-6,2) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 2 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 4 == 2 );
      }
      THEN( "The polytope contains 6 points" ) {
        REQUIRE( P.count() == 6 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a segment defined by two points" ) {
        REQUIRE( X.size() == 2 );
        REQUIRE( X[ 0 ] == Point(-6, 2) );
        REQUIRE( X[ 1 ] == Point( 9,-3) );
      }
    }
  }
  GIVEN( "Given a degenerated simplex { (4,0), (7,2), (-5,-6) } " ) {
    std::vector<Point> V
      = { Point(4,0), Point(7,2), Point(-5,-6) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 2 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 4 == 2 );
      }
      THEN( "The polytope contains 5 points" ) {
        REQUIRE( P.count() == 5 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a segment defined by two points" ) {
        REQUIRE( X.size() == 2 );
        REQUIRE( X[ 0 ] == Point(-5,-6) );
        REQUIRE( X[ 1 ] == Point( 7, 2) );
      }
    }
  }
}

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConvexityHelper in 3D.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "ConvexityHelper< 3 > unit tests",
          "[convexity_helper][3d]" )
{
  typedef ConvexityHelper< 3 >    Helper;
  typedef Helper::Point           Point;
  typedef Helper::RealPoint       RealPoint;
  typedef Helper::RealVector      RealVector;
  typedef SurfaceMesh< RealPoint, RealVector > SMesh;
  typedef PolygonalSurface< Point > LatticePolySurf;
  typedef ConvexCellComplex< Point > CvxCellComplex;
  GIVEN( "Given an octahedron star { (0,0,0), (-2,0,0), (2,0,0), (0,-2,0), (0,2,0), (0,0,-2), (0,0,2) } " ) {
    std::vector<Point> V
      = { Point(0,0,0), Point(-2,0,0), Point(2,0,0), Point(0,-2,0), Point(0,2,0),
      Point(0,0,-2), Point(0,0,2) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 8 non trivial facets plus 12 edge constraints" ) {
        REQUIRE( P.nbHalfSpaces() - 6 == 20 );
      }
      THEN( "The polytope is Minkowski summable" ) {
        REQUIRE( P.canBeSummed() );
      }
      THEN( "The polytope contains the input points" ) {
        REQUIRE( P.isInside( V[ 0 ] ) );
        REQUIRE( P.isInside( V[ 1 ] ) );
        REQUIRE( P.isInside( V[ 2 ] ) );
        REQUIRE( P.isInside( V[ 3 ] ) );
        REQUIRE( P.isInside( V[ 4 ] ) );
        REQUIRE( P.isInside( V[ 5 ] ) );
        REQUIRE( P.isInside( V[ 6 ] ) );
      }
      THEN( "The polytope contains 25 points" ) {
        REQUIRE( P.count() == 25 );
      }
      THEN( "The interior of the polytope contains 7 points" ) {
        REQUIRE( P.countInterior() == 7 );
      }
      THEN( "The boundary of the polytope contains 18 points" ) {
        REQUIRE( P.countBoundary() == 18 );
      }
    }
    WHEN( "Computing the boundary of its convex hull as a SurfaceMesh" ){
      SMesh smesh;
      bool ok = Helper::computeConvexHullBoundary( smesh, V, false );
      CAPTURE( smesh );
      THEN( "The surface mesh is valid and has 6 vertices, 12 edges and 8 faces" ) {
        REQUIRE( ok );
        REQUIRE( smesh.nbVertices() == 6 );
        REQUIRE( smesh.nbEdges() == 12 );
        REQUIRE( smesh.nbFaces() == 8 );
      }
      THEN( "The surface mesh has the topology of a sphere" ) {
        REQUIRE( smesh.Euler() == 2 );
        REQUIRE( smesh.computeManifoldBoundaryEdges().size() == 0 );
        REQUIRE( smesh.computeNonManifoldEdges().size() == 0 );
      }
    }
    WHEN( "Computing the boundary of its convex hull as a lattice PolygonalSurface" ){
      LatticePolySurf lpsurf;
      bool ok = Helper::computeConvexHullBoundary( lpsurf, V, false );
      CAPTURE( lpsurf );
      THEN( "The polygonal surface is valid and has 6 vertices, 12 edges and 8 faces" ) {
        REQUIRE( ok );
        REQUIRE( lpsurf.nbVertices() == 6 );
        REQUIRE( lpsurf.nbEdges() == 12 );
        REQUIRE( lpsurf.nbFaces() == 8 );
        REQUIRE( lpsurf.nbArcs() == 24 );
      }
      THEN( "The polygonal surface has the topology of a sphere and no boundary" ) {
        REQUIRE( lpsurf.Euler() == 2 );
        REQUIRE( lpsurf.allBoundaryArcs().size() == 0 );
        REQUIRE( lpsurf.allBoundaryVertices().size() == 0 );
      }
    }
    WHEN( "Computing its convex hull as a ConvexCellComplex" ){
      CvxCellComplex complex;
      bool ok = Helper::computeConvexHullCellComplex( complex, V, false );
      CAPTURE( complex );
      THEN( "The convex cell complex is valid and has 6 vertices, 8 faces and 1 finite cell" ) {
        REQUIRE( ok );
        REQUIRE( complex.nbVertices() == 6 );
        REQUIRE( complex.nbFaces() == 8 );
        REQUIRE( complex.nbCells() == 1 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      const auto X = Helper::computeConvexHullVertices( V, false );
      CAPTURE( X );
      THEN( "The polytope has 6 vertices" )  {
        REQUIRE( X.size() == 6 );
      }
    }
  }
  GIVEN( "Given a cube with an additional outside vertex " ) {
    std::vector<Point> V
      = { Point(-10,-10,-10), Point(10,-10,-10), Point(-10,10,-10), Point(10,10,-10),
      Point(-10,-10,10), Point(10,-10,10), Point(-10,10,10), Point(10,10,10),
      Point(0,0,18) };
    WHEN( "Computing its Delaunay cell complex" ){
      CvxCellComplex complex;
      bool ok = Helper::computeDelaunayCellComplex( complex, V, false );
      CAPTURE( complex );
      THEN( "The complex has 2 cells, 10 faces, 9 vertices" ) {
        REQUIRE( ok );
        REQUIRE( complex.nbCells() == 2 );
        REQUIRE( complex.nbFaces() == 10 );
        REQUIRE( complex.nbVertices() == 9 );
      }
      THEN( "The faces of cells are finite" ) {
        bool ok_finite = true;
        for ( auto c = 0; c < complex.nbCells(); ++c ) {
          const auto faces = complex.cellFaces( c );
          for ( auto f : faces )
            ok_finite = ok_finite && ! complex.isInfinite( complex.faceCell( f ) );
        }
        REQUIRE( ok_finite );
      }
      THEN( "The opposite of faces of cells are infinite except two" ) {
        int  nb_finite   = 0;
        for ( auto c = 0; c < complex.nbCells(); ++c ) {
          const auto faces = complex.cellFaces( c );
          for ( auto f : faces ) {
            const auto opp_f = complex.opposite( f );
            nb_finite += complex.isInfinite( complex.faceCell( opp_f ) ) ? 0 : 1;
          }
        }
        REQUIRE( nb_finite == 2 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      const auto X = Helper::computeConvexHullVertices( V, false );
      CAPTURE( X );
      THEN( "The polytope has 9 vertices" )  {
        REQUIRE( X.size() == 9 );
      }
    }
  }
  GIVEN( "Given a degenerated 1d polytope { (0,0,1), (3,-1,2), (9,-3,4), (-6,2,-1) } " ) {
    std::vector<Point> V
      = { Point(0,0,1), Point(3,-1,2), Point(9,-3,4), Point(-6,2,-1) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 6 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 6 == 6 );
      }
      THEN( "The polytope contains 6 points" ) {
        REQUIRE( P.count() == 6 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a segment defined by two points" ) {
        REQUIRE( X.size() == 2 );
        REQUIRE( X[ 0 ] == Point(-6, 2,-1) );
        REQUIRE( X[ 1 ] == Point( 9,-3, 4) );
      }
    }    
  }
  GIVEN( "Given a degenerated 1d simplex { (1,0,-1), Point(4,-1,-2), Point(10,-3,-4) } " ) {
    std::vector<Point> V
      = { Point(1,0,-1), Point(4,-1,-2), Point(10,-3,-4) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has 6 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 6 == 6 );
      }
      THEN( "The polytope contains 4 points" ) {
        REQUIRE( P.count() == 4 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a segment defined by two points" ) {
        REQUIRE( X.size() == 2 );
        REQUIRE( X[ 0 ] == Point( 1, 0,-1) );
        REQUIRE( X[ 1 ] == Point(10,-3,-4) );
      }
    }
  }
  GIVEN( "Given a degenerated 2d polytope { (2,1,0), (1,0,1), (1,2,1), (0,1,2), (0,3,2) } " ) {
    std::vector<Point> V
      = { Point(2,1,0), Point(1,0,1), Point(1,2,1), Point(0,1,2), Point(0,3,2) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has more than 6 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 6 == 6 );
      }
      THEN( "The polytope contains 7 points" ) {
        REQUIRE( P.count() == 7 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a quad" ) {
        REQUIRE( X.size() == 4 );
        REQUIRE( X[ 0 ] == Point( 0, 1, 2) );
        REQUIRE( X[ 1 ] == Point( 0, 3, 2) );
        REQUIRE( X[ 2 ] == Point( 1, 0, 1) );
        REQUIRE( X[ 3 ] == Point( 2, 1, 0) );
      }
    }    
  }
  GIVEN( "Given a degenerated 2d simplex { (2,1,0), (1,0,1), (1,5,1), (0,3,2) } " ) {
    std::vector<Point> V
      = { Point(2,1,0), Point(1,0,1), Point(1,5,1), Point(0,3,2) };
    WHEN( "Computing its lattice polytope" ){
      const auto P = Helper::computeLatticePolytope( V, false, true );
      CAPTURE( P );
      THEN( "The polytope is valid and has more than 6 non trivial facets" ) {
        REQUIRE( P.nbHalfSpaces() - 6 == 6 );
      }
      THEN( "The polytope contains 8 points" ) {
        REQUIRE( P.count() == 8 );
      }
      THEN( "The polytope contains no interior points" ) {
        REQUIRE( P.countInterior() == 0 );
      }
    }
    WHEN( "Computing the vertices of its convex hull" ){
      auto X = Helper::computeConvexHullVertices( V, false );
      std::sort( X.begin(), X.end() );
      CAPTURE( X );
      THEN( "The polytope is a quad" ) {
        REQUIRE( X.size() == 4 );
        REQUIRE( X[ 0 ] == Point( 0, 3, 2) );
        REQUIRE( X[ 1 ] == Point( 1, 0, 1) );
        REQUIRE( X[ 2 ] == Point( 1, 5, 1) );
        REQUIRE( X[ 3 ] == Point( 2, 1, 0) );
      }
    }
  }
} 
