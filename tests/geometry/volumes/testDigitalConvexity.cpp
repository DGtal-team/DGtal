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
 * @file testDigitalConvexity.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/01
 *
 * Functions for testing class CellGeometry.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/CellGeometry.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalConvexity.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "DigitalConvexity< Z2 > unit tests", "[digital_convexity][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -5, -5 ), Point( 10, 10 ) );

  GIVEN( "Given a fat simplex { (0,0), (4,-1), (2,5) } " ) {
    std::vector<Point> V = { Point(0,0), Point(4,-1), Point(2,5) };
    auto vertex_cover  = dconv.makeCellCover( V.begin(), V.end() );
    auto fat_simplex   = dconv.makeSimplex  ( V.begin(), V.end() );
    auto inside_pts    = dconv.insidePoints ( fat_simplex );
    auto simplex_cover = dconv.makeCellCover( fat_simplex );
    auto point_cover   = dconv.makeCellCover( inside_pts.begin(), inside_pts.end() );
    THEN( "The fat simplex is not degenerated." ) {
      REQUIRE( dconv.isSimplexFullDimensional( V.begin(), V.end() ) );
    }
    THEN( "Its vertex cover contains 3 0-cells, 12 1-cells, 12 2-cells" ) {
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 2 ) == 12 );
    }
    THEN( "Its vertex cover is a subset of its point cover" ) {
      REQUIRE( vertex_cover.subset( point_cover ) );
    }
    THEN( "Its point cover is a subset of its simplex cover" ) {
      REQUIRE( point_cover.subset( simplex_cover ) );
    }
    THEN( "Being fat, its simplex cover is equal to its point cover" ) {
      REQUIRE( simplex_cover.subset( point_cover ) );
    }
  }
  GIVEN( "Given a thin simplex { (0,0), (4,3), (7,5) } " ) {
    std::vector<Point> V = { Point(0,0), Point(4,3), Point(7,5) };
    auto vertex_cover  = dconv.makeCellCover( V.begin(), V.end() );
    auto thin_simplex  = dconv.makeSimplex  ( V.begin(), V.end() );
    auto inside_pts    = dconv.insidePoints ( thin_simplex );
    auto simplex_cover = dconv.makeCellCover( thin_simplex );
    auto point_cover   = dconv.makeCellCover( inside_pts.begin(), inside_pts.end() );
    THEN( "The thin simplex is not degenerated." ) {
      REQUIRE( dconv.isSimplexFullDimensional( V.begin(), V.end() ) );
    }
    THEN( "Its vertex cover contains 3 0-cells, 12 1-cells, 12 2-cells" ) {
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.cubicalComplex().nbCells( 2 ) == 12 );
    }
    THEN( "Its vertex cover is a subset of its point cover" ) {
      REQUIRE( vertex_cover.subset( point_cover ) );
    }
    THEN( "Its point cover is a subset of its simplex cover" ) {
      REQUIRE( point_cover.subset( simplex_cover ) );
    }
    THEN( "Being thin, its simplex cover is not equal to its point cover for 1<=dim<=2" ) {
      REQUIRE( ! simplex_cover.subset( point_cover ) );
      REQUIRE( simplex_cover.subset( point_cover, 0 ) );
      REQUIRE( ! simplex_cover.subset( point_cover, 1 ) );
      REQUIRE( ! simplex_cover.subset( point_cover, 2 ) );
    }
  }
}

SCENARIO( "DigitalConvexity< Z2 > fully convex triangles", "[convex_simplices][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( 0, 0 ), Point( 4, 4 ) );
  DConvexity dconv( Point( -1, -1 ), Point( 5, 5 ) );
  
  WHEN( "Computing all triangles in domain (0,0)-(4,4)." ) {
    unsigned int nb_notsimplex = 0;
    unsigned int nb_invalid    = 0;
    unsigned int nb_degenerated= 0;
    unsigned int nb_common     = 0;
    unsigned int nb_unitary    = 0;
    for ( auto a : domain ) 
      for ( auto b : domain ) 
	for ( auto c : domain )
	  {
	    nb_notsimplex   += ! dconv.isSimplexFullDimensional( { a, b, c } ) ? 1 : 0;
	    auto tri_type    = dconv.simplexType( { a, b, c } );
	    nb_degenerated  += tri_type == DConvexity::SimplexType::DEGENERATED ? 1 : 0;
	    nb_invalid      += tri_type == DConvexity::SimplexType::INVALID ? 1 : 0;
	    nb_unitary      += tri_type == DConvexity::SimplexType::UNITARY ? 1 : 0;
	    nb_common       += tri_type == DConvexity::SimplexType::COMMON  ? 1 : 0;
	  }
    THEN( "All 2737 invalid triangles are degenerated " ) {
      REQUIRE( nb_invalid == 0 );
      REQUIRE( nb_notsimplex == nb_degenerated );
      REQUIRE( nb_degenerated == 2737  );
    }
    THEN( "There are 12888 valid triangles" ) {
      REQUIRE( nb_unitary + nb_common == 12888 );
    }
    THEN( "There are fewer (1920) unitary triangles than common triangles (10968)" ) {
      REQUIRE( nb_unitary ==  1920 );
      REQUIRE( nb_common  == 10968 );
      REQUIRE( nb_unitary <  nb_common );
    }
    THEN( "The total number of triangles (unitary, common, degenerated) is (domain size)^3, i.e. 5^6" ) {
      REQUIRE( nb_unitary + nb_common + nb_degenerated == 5*5*5*5*5*5 );
    }
  }
  WHEN( "Computing all triangles in domain (0,0)-(4,4)." ) {
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb01_not2 = 0;
    for ( auto a : domain ) 
      for ( auto b : domain ) 
	for ( auto c : domain )
	  {
	    if ( ! ( ( a < b ) && ( b < c ) ) ) continue;
	    if ( ! dconv.isSimplexFullDimensional( { a, b, c } ) ) continue;
	    auto triangle = dconv.makeSimplex( { a, b, c } );
	    bool cvx0     = dconv.isKConvex( triangle, 0 );
	    bool cvx1     = dconv.isKConvex( triangle, 1 );
	    bool cvx2     = dconv.isKConvex( triangle, 2 );
	    nbsimplex += 1;
	    nb0       += cvx0 ? 1 : 0;
	    nb1       += cvx1 ? 1 : 0;
	    nb2       += cvx2 ? 1 : 0;
	    nb01_not2 += ( cvx0 && cvx1 && ! cvx2 ) ? 1 : 0;
	  }
    THEN( "All valid triangles are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "There are less 1-convex and 2-convex than 0-convex." ) {
      REQUIRE( nb1 < nb0 );
      REQUIRE( nb2 < nb0 );
    }
    THEN( "When the triangle is 0-convex and 1-convex, then it is 2-convex." ) {
      REQUIRE( nb1 <= nb2 );
      REQUIRE( nb01_not2 == 0 );
    }
  }
}
SCENARIO( "DigitalConvexity< Z3 > fully convex tetrahedra", "[convex_simplices][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( 0, 0, 0 ), Point( 3, 3, 3 ) );
  DConvexity dconv( Point( -1, -1, -1 ), Point( 4, 4, 4 ) );
  
  WHEN( "Computing all lexicographically ordered tetrahedra anchored at (0,0,0) in domain (0,0,0)-(3,3,3)." ) {
    unsigned int nb_notsimplex = 0;
    unsigned int nb_invalid    = 0;
    unsigned int nb_degenerated= 0;
    unsigned int nb_common     = 0;
    unsigned int nb_unitary    = 0;
    Point a(0,0,0);
    for ( auto b : domain ) 
      for ( auto c : domain )
	for ( auto d : domain )
	  {
	    if ( ! ( ( a < b ) && ( b < c ) && ( c < d ) ) ) continue;
	    nb_notsimplex   += ! dconv.isSimplexFullDimensional( {a,b,c,d} ) ? 1 : 0;
	    auto tri_type    = dconv.simplexType( { a, b, c, d } );
	    nb_degenerated  += tri_type == DConvexity::SimplexType::DEGENERATED ? 1 : 0;
	    nb_invalid      += tri_type == DConvexity::SimplexType::INVALID ? 1 : 0;
	    nb_unitary      += tri_type == DConvexity::SimplexType::UNITARY ? 1 : 0;
	    nb_common       += tri_type == DConvexity::SimplexType::COMMON  ? 1 : 0;
	  }
    THEN( "All 4228 invalid tetrahedra are degenerated " ) {
      REQUIRE( nb_invalid == 0 );
      REQUIRE( nb_notsimplex == nb_degenerated );
      REQUIRE( nb_degenerated == 4228  );
    }
    THEN( "There are 35483 valid tetrahedra" ) {
      REQUIRE( nb_unitary + nb_common == 35483 );
    }
    THEN( "There are fewer (2515) unitary triangles than common triangles (32968)" ) {
      REQUIRE( nb_unitary ==  2515 );
      REQUIRE( nb_common  == 32968 );
      REQUIRE( nb_unitary <  nb_common );
    }
    THEN( "The total number of triangles (unitary, common, degenerated) is 39711" ) {
      REQUIRE( nb_unitary + nb_common + nb_degenerated == 39711 );
    }
  }
  WHEN( "Computing many tetrahedra in domain (0,0,0)-(4,4,4)." ) {
    const unsigned int nb = 200; 
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb3      = 0;
    unsigned int nb012_not3 = 0;
    unsigned int nbf      = 0;
    unsigned int nb0123   = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
	Point a( rand() % 5, rand() % 5, rand() % 5 );
	Point b( rand() % 5, rand() % 5, rand() % 5 );
	Point c( rand() % 5, rand() % 5, rand() % 5 );
	Point d( rand() % 5, rand() % 5, rand() % 5 );
	if ( ! dconv.isSimplexFullDimensional( { a, b, c, d } ) ) continue;
	auto tetra = dconv.makeSimplex( { a, b, c, d } );
	bool cvx0     = dconv.isKConvex( tetra, 0 );
	bool cvx1     = dconv.isKConvex( tetra, 1 );
	bool cvx2     = dconv.isKConvex( tetra, 2 );
	bool cvx3     = dconv.isKConvex( tetra, 3 );
	bool cvxf     = dconv.isFullyConvex( tetra );
	nbsimplex += 1;
	nb0       += cvx0 ? 1 : 0;
	nb1       += cvx1 ? 1 : 0;
	nb2       += cvx2 ? 1 : 0;
	nb3       += cvx3 ? 1 : 0;
	nbf       += cvxf ? 1 : 0;
	nb0123    += ( cvx0 && cvx1 && cvx2 && cvx3 ) ? 1 : 0;
	nb012_not3+= ( cvx0 && cvx1 && cvx2 && ! cvx3 ) ? 1 : 0;
      }
    THEN( "All valid tetrahedra are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "There are less 1-convex, 2-convex and 3-convex than 0-convex." ) {
      REQUIRE( nb1 < nb0 );
      REQUIRE( nb2 < nb0 );
      REQUIRE( nb3 < nb0 );
    }
    THEN( "When the tetrahedron is 0-convex, 1-convex and 2-convex, then it is 3-convex." ) {
      REQUIRE( nb1 <= nb3 );
      REQUIRE( nb2 <= nb3 );
      REQUIRE( nb012_not3 == 0 );
      REQUIRE( nbf == nb0123 );
    }
  }
}

SCENARIO( "DigitalConvexity< Z3 > rational fully convex tetrahedra", "[convex_simplices][3d][rational]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -1, -1, -1 ), Point( 10, 10, 10 ) );
  WHEN( "Computing many tetrahedra in domain (0,0,0)-(4,4,4)." ) {
    const unsigned int nb = 400; 
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb3      = 0;
    unsigned int nb012_not3 = 0;
    unsigned int nbf      = 0;
    unsigned int nb0123   = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
	Point a( 2*(rand() % 10), rand() % 20, 2*(rand() % 10) );
	Point b( rand() % 20, 2*(rand() % 10), 2*(rand() % 10) );
	Point c( 2*(rand() % 10), 2*(rand() % 10), rand() % 20 );
	Point d( 2*(rand() % 10), 2*(rand() % 10), 2*(rand() % 10) );
	if ( ! dconv.isSimplexFullDimensional( { a, b, c, d } ) ) continue;
	auto tetra = dconv.makeRationalSimplex( { Point(2,2,2), a, b, c, d } );
	bool cvx0     = dconv.isKConvex( tetra, 0 );
	bool cvx1     = dconv.isKConvex( tetra, 1 );
	bool cvx2     = dconv.isKConvex( tetra, 2 );
	bool cvx3     = dconv.isKConvex( tetra, 3 );
	bool cvxf     = dconv.isFullyConvex( tetra );
	nbsimplex += 1;
	nb0       += cvx0 ? 1 : 0;
	nb1       += cvx1 ? 1 : 0;
	nb2       += cvx2 ? 1 : 0;
	nb3       += cvx3 ? 1 : 0;
	nbf       += cvxf ? 1 : 0;
	nb0123    += ( cvx0 && cvx1 && cvx2 && cvx3 ) ? 1 : 0;
	nb012_not3+= ( cvx0 && cvx1 && cvx2 && ! cvx3 ) ? 1 : 0;
      }
    THEN( "All valid tetrahedra are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "There are less 1-convex, 2-convex and 3-convex than 0-convex." ) {
      REQUIRE( nb1 < nb0 );
      REQUIRE( nb2 < nb0 );
      REQUIRE( nb3 < nb0 );
    }
    THEN( "When the tetrahedron is 0-convex, 1-convex and 2-convex, then it is 3-convex." ) {
      CAPTURE( nb0 );
      CAPTURE( nb1 );
      CAPTURE( nb2 );
      CAPTURE( nb3 );
      CAPTURE( nb012_not3 );
      CAPTURE( nbf );
      REQUIRE( nb2 <= nb3 );
      REQUIRE( nb012_not3 == 0 );
      REQUIRE( nbf == nb0123 );
    }
  }
}


SCENARIO( "DigitalConvexity< Z2 > rational fully convex tetrahedra", "[convex_simplices][2d][rational]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Integer                  Integer;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -1, -1 ), Point( 10, 10 ) );
  WHEN( "Computing many triangle in domain (0,0)-(9,9)." ) {
    const unsigned int nb = 400; 
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb01_not2 = 0;
    unsigned int nbf      = 0;
    unsigned int nb012    = 0;
    unsigned int nb1_notlat = 0;
    unsigned int nb2_notlat = 0;
    unsigned int nb3_notlat = 0;
    unsigned int nbf_notlat = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
	Point a( 2*(rand() % 10),    rand() % 20  );
	Point b(    rand() % 20 , 2*(rand() % 10) );
	Point c( 2*(rand() % 10), 2*(rand() % 10) );
	if ( ! dconv.isSimplexFullDimensional( { a, b, c } ) ) continue;
	auto tetra = dconv.makeRationalSimplex( { Point(2,2), a, b, c } );
	bool cvx0     = dconv.isKConvex( tetra, 0 );
	bool cvx1     = dconv.isKConvex( tetra, 1 );
	bool cvx2     = dconv.isKConvex( tetra, 2 );
	bool cvxf     = dconv.isFullyConvex( tetra );
	nbsimplex += 1;
	nb0       += cvx0 ? 1 : 0;
	nb1       += cvx1 ? 1 : 0;
	nb2       += cvx2 ? 1 : 0;
	nbf       += cvxf ? 1 : 0;
	nb012     += ( cvx0 && cvx1 && cvx2 ) ? 1 : 0;
	nb01_not2 += ( cvx0 && cvx1 && ! cvx2 ) ? 1 : 0;
      }
    THEN( "All valid tetrahedra are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "There are less 1-convex, 2-convex than 0-convex." ) {
      REQUIRE( nb1 < nb0 );
      REQUIRE( nb2 < nb0 );
    }
    THEN( "When the tetrahedron is 0-convex, and 1-convex, then it is 2-convex." ) {
      CAPTURE( nb0 );
      CAPTURE( nb1 );
      CAPTURE( nb2 );
      CAPTURE( nb01_not2 );
      CAPTURE( nbf );
      REQUIRE( nb1 <= nb2 );
      REQUIRE( nb01_not2 == 0 );
      REQUIRE( nbf == nb012 );
    }
  }
}

