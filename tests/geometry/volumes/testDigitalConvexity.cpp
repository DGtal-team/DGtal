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
 * Functions for testing class DigitalConvexity.
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
#include "DGtal/geometry/tools/AffineGeometry.h"
#include "DGtal/geometry/tools/AffineBasis.h"
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
      REQUIRE( vertex_cover.computeNbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.computeNbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.computeNbCells( 2 ) == 12 );
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
      REQUIRE( vertex_cover.computeNbCells( 0 ) == 3 );
      REQUIRE( vertex_cover.computeNbCells( 1 ) == 12 );
      REQUIRE( vertex_cover.computeNbCells( 2 ) == 12 );
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
    const unsigned int nb = 50;
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb3      = 0;
    unsigned int nb012_not3 = 0;
    unsigned int nbf      = 0;
    unsigned int nbfg     = 0;
    unsigned int nbffast  = 0;
    unsigned int nb0123   = 0;
    unsigned int nbdim3   = 0;
    unsigned int nbfull   = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
        Point a( rand() % 5, rand() % 5, rand() % 5 );
        Point b( rand() % 5, rand() % 5, rand() % 5 );
        Point c( rand() % 5, rand() % 5, rand() % 5 );
        Point d( rand() % 5, rand() % 5, rand() % 5 );
        if ( ! dconv.isSimplexFullDimensional( { a, b, c, d } ) ) continue;
        nbfull += 1;
        auto dim = functions::computeAffineDimension( std::vector<Point>{ a, b, c, d } );
        nbdim3 += ( dim == 3 ) ? 1 : 0;
        auto tetra = dconv.makeSimplex( { a, b, c, d } );
        std::vector< Point > X;
        tetra.getPoints( X );
        bool cvx0     = dconv.isKConvex( tetra, 0 );
        bool cvx1     = dconv.isKConvex( tetra, 1 );
        bool cvx2     = dconv.isKConvex( tetra, 2 );
        bool cvx3     = dconv.isKConvex( tetra, 3 );
        bool cvxf     = dconv.isFullyConvex( tetra );
        bool cvxfg    = dconv.isFullyConvex( X, false );
        bool cvxffast = dconv.isFullyConvexFast( X );
        if ( cvxf != cvxfg || cvxf != cvxffast) {
          bool cvxfc =  dconv.FC( X ).size() == X.size();
          std::cout << "[0123 " << cvx0 << cvx1 << cvx2 << cvx3 << "] "
                    << "[K " << cvxf << "] [M " << cvxfg
                    << "] [MF " << cvxffast << "] [FC " << cvxfc << "] "
                    << a << b << c << d << "\n";
          std::cout << "X=";
          for ( auto p : X ) std::cout << " " << p;
          std::cout << "\n";
        }
        nbsimplex += 1;
        nb0       += cvx0 ? 1 : 0;
        nb1       += cvx1 ? 1 : 0;
        nb2       += cvx2 ? 1 : 0;
        nb3       += cvx3 ? 1 : 0;
        nbf       += cvxf ? 1 : 0;
        nbfg      += cvxfg ? 1 : 0;
        nbffast   += cvxffast ? 1 : 0;
        nb0123    += ( cvx0 && cvx1 && cvx2 && cvx3 ) ? 1 : 0;
        nb012_not3+= ( cvx0 && cvx1 && cvx2 && ! cvx3 ) ? 1 : 0;
      }
    THEN( "All valid tetrahedra are 0-convex." ) {
      REQUIRE( nb0 == nbsimplex );
    }
    THEN( "All full dimensional simplices have affine dimension 3" ) {
      REQUIRE( nbfull == nbdim3 );
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
    THEN( "All methods for computing full convexity agree." ) {
      REQUIRE( nbf == nbfg );
      REQUIRE( nbf == nbffast );
    }
  }
}

SCENARIO( "DigitalConvexity< Z3 > rational fully convex tetrahedra", "[convex_simplices][3d][rational]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -10, -10, -10 ), Point( 100, 100, 100 ) );
  WHEN( "Computing many tetrahedra in domain (0,0,0)-(4,4,4)." ) {
    const unsigned int nb = 30;
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
      REQUIRE( nb1 <= nb0 );
      REQUIRE( nb2 <= nb0 );
      REQUIRE( nb3 <= nb0 );
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


SCENARIO( "DigitalConvexity< Z2 > rational fully convex triangles", "[convex_simplices][2d][rational]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -1, -1 ), Point( 30, 30 ) );
  WHEN( "Computing many triangle in domain (0,0)-(9,9)." ) {
    const unsigned int nb = 50;
    unsigned int nbsimplex= 0;
    unsigned int nb0      = 0;
    unsigned int nb1      = 0;
    unsigned int nb2      = 0;
    unsigned int nb01_not2 = 0;
    unsigned int nbf      = 0;
    unsigned int nb012    = 0;
    for ( unsigned int i = 0; i < nb; ++i )
      {
        Point a( 2*(rand() % 10),    rand() % 20  );
        Point b(    rand() % 20 , 2*(rand() % 10) );
        Point c( 2*(rand() % 10), 2*(rand() % 10) );
        if ( ! dconv.isSimplexFullDimensional( { a, b, c } ) ) continue;
        auto triangle = dconv.makeRationalSimplex( { Point(2,2), a, b, c } );
        bool cvx0     = dconv.isKConvex( triangle, 0 );
        bool cvx1     = dconv.isKConvex( triangle, 1 );
        bool cvx2     = dconv.isKConvex( triangle, 2 );
        bool cvxf     = dconv.isFullyConvex( triangle );
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
      REQUIRE( nb1 <= nb0 );
      REQUIRE( nb2 <= nb0 );
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

SCENARIO( "DigitalConvexity< Z3 > full subconvexity of segments and triangles", "[subconvexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -5, -5, -5 ), Point( 5, 5, 5 ) );
  DConvexity dconv( Point( -6, -6, -6 ), Point( 6, 6, 6 ) );

  WHEN( "Computing many tetrahedra" ) {
    const unsigned int nb   = 50;
    unsigned int nb_fulldim = 0;
    unsigned int nb_ok_seg  = 0;
    unsigned int nb_ok_tri  = 0;
    for ( unsigned int l = 0; l < nb; ++l )
      {
        const Point a { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point b { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point c { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point d { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const std::vector<Point> pts = { a, b, c, d };
        const bool fulldim = dconv.isSimplexFullDimensional(pts.cbegin(), pts.cend());
        nb_fulldim += fulldim ? 1 : 0;
        if ( ! fulldim ) continue;
        auto simplex = dconv.makeSimplex( pts.cbegin(), pts.cend() );
        auto cover = dconv.makeCellCover( simplex, 0, 3 );
        {
          unsigned int nb_subconvex = 0;
          unsigned int nb_total     = 0;
          for ( unsigned int i = 0; i < 4; i++ )
            for ( unsigned int j = i+1; j < 4; j++ )
              {
                auto segment = dconv.makeSimplex( { pts[ i ], pts[ j ] } );
                bool ok = dconv.isFullySubconvex( segment, cover );
                nb_subconvex += ok ? 1 : 0;
                nb_total     += 1;
                if ( ! ok ) {
                  trace.info() << "****** SEGMENT NOT SUBCONVEX ******" << std::endl; 
                  trace.info() << "splx v =" << a << b << c << d << std::endl;
                  trace.info() << "simplex=" << simplex << std::endl;
                  trace.info() << "seg v  =" << pts[ i ] << pts[ j ] << std::endl;
                  trace.info() << "segment=" << segment << std::endl;
                }
              }
          nb_ok_seg += ( nb_subconvex == nb_total ) ? 1 : 0;
        }
        {
          unsigned int nb_subconvex = 0;
          unsigned int nb_total     = 0;
          for ( unsigned int i = 0; i < 4; i++ )
            for ( unsigned int j = i+1; j < 4; j++ )
              for ( unsigned int k = j+1; k < 4; k++ )
                {
                  auto triangle = dconv.makeSimplex({ pts[ i ], pts[ j ], pts[ k ] });
                  bool ok = dconv.isFullySubconvex( triangle, cover );
                  nb_subconvex += ok ? 1 : 0;
                  nb_total     += 1;
                  if ( ! ok ) {
                    trace.info() << "****** TRIANGLE NOT SUBCONVEX ****" << std::endl; 
                    trace.info() << "splx v =" << a << b << c << d << std::endl;
                    trace.info() << "simplex=" << simplex << std::endl;
                    trace.info() << "tri v  =" << pts[ i ] << pts[ j ]
                                 << pts[ k ] << std::endl;
                    trace.info() << "triangle=" << triangle << std::endl;
                  }
                }
          nb_ok_tri += ( nb_subconvex == nb_total ) ? 1 : 0;
        }
      }
    THEN( "At least half the tetrahedra are full dimensional." ) {
      REQUIRE( nb_fulldim >= nb / 2 );
    }
    THEN( "All segments of a tetrahedron should be subconvex to it." ) {
      REQUIRE( nb_ok_seg == nb_fulldim );
    }
    THEN( "All triangles of a tetrahedron should be subconvex to it." ) {
      REQUIRE( nb_ok_tri == nb_fulldim );
    }
  }
}

SCENARIO( "DigitalConvexity< Z3 > full convexity of polyhedra", "[full_convexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -35, -35, -35 ), Point( 35, 35, 35 ) );
  DConvexity dconv( Point( -36, -36, -36 ), Point( 36, 36, 36 ) );

  const unsigned int nb = 10;
  unsigned int nbfg     = 0;
  unsigned int nbffast  = 0;
  unsigned int nbfenv   = 0;
  typedef std::vector< Point > PointRange;
  std::vector< PointRange > XX;
  for ( unsigned int i = 0; i < nb; ++i )
    {
      unsigned int k = 100;
      PointRange X( k );
      for ( unsigned int j = 0; j < k; ++ j )
        X[ j ] = Point( rand() % 10, rand() % 10, rand() % 10 );
      auto P = dconv.makePolytope( X );
      PointRange Y;
      P.getPoints( Y );
      XX.push_back( Y );
    }
  Clock c;
  c.startClock();
  for ( const auto& X : XX )
    {
      bool fcvx = dconv.isFullyConvex( X, false );
      nbfg += fcvx ? 1 : 0;
    }
  double t1 = c.stopClock();
  c.startClock();
  for ( const auto& X : XX )
    {
      bool fcvx = dconv.isFullyConvexFast( X );
      nbffast += fcvx ? 1 : 0;
    }
  double t2 = c.stopClock();
  c.startClock();
  for ( const auto& X : XX )
    {
      auto card = dconv.envelope( X ).size();
      bool fcvx = card == X.size();
      nbfenv += fcvx ? 1 : 0;
    }
  double t3 = c.stopClock();
  WHEN( "Computing many polytopes." ) {
    THEN( "All three methods agree on full convexity results" ) {
      CAPTURE( t1 );
      CAPTURE( t2 );
      CAPTURE( t3 );
      REQUIRE( nbfg == nbffast );
      REQUIRE( nbfg == nbfenv );
    }
  }
}


SCENARIO( "DigitalConvexity< Z4 > full convexity of polyhedra", "[full_convexity][4d]" )
{
  typedef KhalimskySpaceND<4,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -35, -35, -35, -35 ), Point( 35, 35, 35, 35 ) );
  DConvexity dconv( Point( -36, -36, -36, -36 ), Point( 36, 36, 36, 36 ) );

  const unsigned int nb = 4;
  unsigned int nbfg     = 0;
  unsigned int nbffast  = 0;
  unsigned int nbfenv  = 0;
  typedef std::vector< Point > PointRange;
  std::vector< PointRange > XX;
  for ( unsigned int i = 0; i < nb; ++i )
    {
      unsigned int k = 100;
      PointRange X( k );
      for ( unsigned int j = 0; j < k; ++ j )
        X[ j ] = Point( rand() % 8, rand() % 8, rand() % 8, rand() % 8 );
      auto P = dconv.makePolytope( X );
      PointRange Y;
      P.getPoints( Y );
      XX.push_back( Y );
    }
  Clock c;
  c.startClock();
  for ( const auto& X : XX )
    {
      bool fcvx = dconv.isFullyConvex( X, false );
      nbfg += fcvx ? 1 : 0;
    }
  double t1 = c.stopClock();
  c.startClock();
  for ( const auto& X : XX )
    {
      bool fcvx = dconv.isFullyConvexFast( X );
      nbffast += fcvx ? 1 : 0;
    }
  double t2 = c.stopClock();
  c.startClock();
  for ( const auto& X : XX )
    {
      auto card = dconv.envelope( X ).size();
      bool fcvx = card == X.size();
      nbfenv += fcvx ? 1 : 0;
    }
  double t3 = c.stopClock();
  WHEN( "Computing many polytopes." ) {
    THEN( "All three methods agree on full convexity results" ) {
      CAPTURE( t1 );
      CAPTURE( t2 );
      CAPTURE( t3 );
      REQUIRE( nbfg == nbffast );
      REQUIRE( nbfg == nbfenv );
    }
  }
}

SCENARIO( "DigitalConvexity< Z2 > sub-convexity of polyhedra", "[full_subconvexity][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -36, -36 ), Point( 36, 36 ) );
  unsigned int k = 6;
  std::vector< Point > X( k );
  X[ 0 ] = Point( 0,0 );
  X[ 1 ] = Point( 7,-2 );
  X[ 2 ] = Point( 3,6 );
  X[ 3 ] = Point( 5, 5 );
  X[ 4 ] = Point( 2, 3 );
  X[ 5 ] = Point( -1, 1 );
  auto  P = dconv.makePolytope( X, true );
  auto CG = dconv.makeCellCover( P, 0, 2 );
  auto  L = dconv.StarCvxH( X, 0 );
  REQUIRE( CG.nbCells() == L.size() );  
  for ( unsigned int i = 0; i < k; i++ )
    for ( unsigned int j = i+1; j < k; j++ )
      {
        std::vector< Point > Z { X[ i ], X[ j ] };
        const auto Q        = dconv.makePolytope( Z );
        bool tangent_old    = dconv.isFullySubconvex( Q, CG );
        bool tangent_new    = dconv.isFullySubconvex( Z, L );
        bool tangent_ab_old = dconv.isFullySubconvex( X[ i ], X[ j ], CG );
        bool tangent_ab_new = dconv.isFullySubconvex( X[ i ], X[ j ], L );
        REQUIRE( tangent_old == tangent_new );
        REQUIRE( tangent_ab_old == tangent_ab_new );
        REQUIRE( tangent_new == tangent_ab_new );
      }
}

SCENARIO( "DigitalConvexity< Z3 > sub-convexity of polyhedra", "[full_subconvexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -36, -36, -36 ), Point( 36, 36, 36 ) );
  std::vector< Point > X( 5 );
  X[ 0 ] = Point( 0,0,0 );
  X[ 1 ] = Point( 0,5,1 );
  X[ 2 ] = Point( 2,1,6 );
  X[ 3 ] = Point( 6,1,1 );
  X[ 4 ] = Point( -2,-2,-3 );
  auto  P = dconv.makePolytope( X, true );
  auto CG = dconv.makeCellCover( P, 0, 3 );
  auto  L = dconv.StarCvxH( X, 0 );
  std::vector< Point > Y;
  P.getPoints( Y );
  REQUIRE( CG.nbCells() == L.size() );
  unsigned int nb    = 0;
  unsigned int nb_ok = 0;
  unsigned int nb_tgt= 0;
  for ( int i = 0; i < 100; i++ )
    {
      Point a( rand() % 6, rand() % 6, rand() % 6 );
      Point b( rand() % 6, rand() % 6, rand() % 6 );
      //      Point b( rand() % 20 - 10, rand() % 20 - 10, rand() % 20 - 10 );
      bool tangent_ab_old = dconv.isFullySubconvex( a, b, CG );
      bool tangent_ab_new = dconv.isFullySubconvex( a, b, L );
      nb_tgt += tangent_ab_new ? 1 : 0;
      nb_ok  += ( tangent_ab_old == tangent_ab_new ) ? 1 : 0;
      nb     += 1;
    }
  REQUIRE( nb == nb_ok );
  REQUIRE( 0  <  nb_tgt );
  REQUIRE( nb_tgt < 100 );
}

SCENARIO( "DigitalConvexity< Z3 > envelope", "[envelope][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -36, -36, -36 ), Point( 36, 36, 36 ) );

  WHEN( "Computing the envelope Z of a digital set X with direct algorithm" ) {
    THEN( "Z contains X" ){
      for ( int k = 0; k < 5; k++ )
        {
          int n = 3 + ( rand() % 7 );
          std::set< Point > S;
          for ( int i = 0; i < n; i++ )
            S.insert( Point( rand() % 10, rand() % 10, rand() % 10 ) );
          std::vector< Point > X( S.cbegin(), S.cend() );
          CAPTURE( X );
          auto Z = dconv.envelope( X, DConvexity::EnvelopeAlgorithm::DIRECT );
          CAPTURE( Z );
          CAPTURE( dconv.depthLastEnvelope() );
          bool Z_includes_X = std::includes( Z.cbegin(), Z.cend(),
                                             X.cbegin(), X.cend() );
          REQUIRE( X.size() <= Z.size() );
          REQUIRE( Z_includes_X );
        }
      THEN( "Z is fully convex" ){
        for ( int k = 0; k < 5; k++ )
          {
            int n = 3 + ( rand() % 7 );
            std::set< Point > S;
            for ( int i = 0; i < n; i++ )
              S.insert( Point( rand() % 10, rand() % 10, rand() % 10 ) );
            std::vector< Point > X( S.cbegin(), S.cend() );
            auto Z = dconv.envelope( X );
            CAPTURE( X );
            CAPTURE( dconv.depthLastEnvelope() );
            REQUIRE( dconv.isFullyConvex( Z ) );
          }
      }
    }
  }
  WHEN( "Computing the envelope Z of a digital set X with LatticeSet algorithm" ) {
    THEN( "Z contains X" ){
      for ( int k = 0; k < 5; k++ )
        {
          int n = 3 + ( rand() % 7 );
          std::set< Point > S;
          for ( int i = 0; i < n; i++ )
            S.insert( Point( rand() % 10, rand() % 10, rand() % 10 ) );
          std::vector< Point > X( S.cbegin(), S.cend() );
          auto Z = dconv.envelope( X, DConvexity::EnvelopeAlgorithm::LATTICE_SET );
          CAPTURE( dconv.depthLastEnvelope() );
          bool Z_includes_X = std::includes( Z.cbegin(), Z.cend(),
                                             X.cbegin(), X.cend() );
          REQUIRE( X.size() <= Z.size() );
          REQUIRE( Z_includes_X );
        }
      THEN( "Z is fully convex" ){
        for ( int k = 0; k < 5; k++ )
          {
            int n = 3 + ( rand() % 7 );
            std::set< Point > S;
            for ( int i = 0; i < n; i++ )
              S.insert( Point( rand() % 10, rand() % 10, rand() % 10 ) );
            std::vector< Point > X( S.cbegin(), S.cend() );
            auto Z = dconv.envelope( X );
            CAPTURE( dconv.depthLastEnvelope() );
            REQUIRE( dconv.isFullyConvex( Z ) );
          }
      }
    }
  }
}

SCENARIO( "DigitalConvexity< Z2 > envelope", "[envelope][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -360, -360 ), Point( 360, 360 ) );
  
  WHEN( "Computing the envelope Z of two points" ) {
    THEN( "it requires at most one iteration" ){
      for ( int k = 0; k < 10; k++ )
        {
          std::vector< Point > X;
          X.push_back( Point( rand() % 100, rand() % 100 ) );
          X.push_back( Point( rand() % 100, rand() % 100 ) );
          std::sort( X.begin(), X.end() );
          auto Z = dconv.envelope( X );
          REQUIRE( dconv.depthLastEnvelope() <= 1 );
        }
    }
  }
}

SCENARIO( "DigitalConvexity< Z2 > relative envelope", "[rel_envelope][2d]" )
{
  typedef KhalimskySpaceND<2,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -360, -360 ), Point( 360, 360 ) );
  
  std::vector< Point > X { Point( -10, -7 ), Point( 10, 7 ) };
  std::vector< Point > Y { Point( -11, -6 ), Point( 9, 8 ) };
  std::sort( X.begin(), X.end() );
  std::sort( Y.begin(), Y.end() );
  X = dconv.envelope( X ); 
  Y = dconv.envelope( Y ); 
  REQUIRE( dconv.isFullyConvex( X ) );
  REQUIRE( dconv.isFullyConvex( Y ) );
  WHEN( "Computing the envelope of X relative to Y and Y relative to X" ) {
    auto FC_X_rel_Y = dconv.relativeEnvelope( X, Y );
    auto FC_Y_rel_X = dconv.relativeEnvelope( Y, X );
    THEN( "Both sets are fully convex" ){
      REQUIRE( dconv.isFullyConvex( FC_X_rel_Y ) );
      REQUIRE( dconv.isFullyConvex( FC_Y_rel_X ) );
    }
    THEN( "There are inclusion rules between sets" ){
      CAPTURE( FC_X_rel_Y );
      CAPTURE( FC_Y_rel_X );
      REQUIRE( std::includes( Y.cbegin(), Y.cend(),
                              FC_X_rel_Y.cbegin(), FC_X_rel_Y.cend() ) );
      REQUIRE( std::includes( X.cbegin(), X.cend(),
                              FC_Y_rel_X.cbegin(), FC_Y_rel_X.cend() ) );
    }
  }
  WHEN( "Computing the envelope of X relative to Y specified by a predicate" ) {
    auto PredY = [] ( Point p )
    { return ( -4 <= p.dot( Point( 2,5 ) ) ) && ( p.dot( Point( 2,5 ) ) < 9 ); };
    auto FC_X_rel_Y = dconv.relativeEnvelope( X, PredY );
    THEN( "It is fully convex and included in Y" ){
      CAPTURE( FC_X_rel_Y );
      REQUIRE( dconv.isFullyConvex( FC_X_rel_Y ) );
      int nb    = 0;
      int nb_in = 0;
      for ( auto p : FC_X_rel_Y )
        {
          nb_in += PredY( p ) ? 1 : 0;
          nb    += 1;
        }
      REQUIRE( nb == nb_in );
    }
  }
}

SCENARIO( "DigitalConvexity< Z3 > relative envelope", "[rel_envelope][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;

  DConvexity dconv( Point( -360, -360, -360 ), Point( 360, 360, 360 ) );
  
  std::vector< Point > X { Point( -61, -20, -8 ), Point( 43, 25, 9 ) };
  std::vector< Point > Y { Point( -50, -27, -10 ), Point( 40, 37, 17 ) };
  std::sort( X.begin(), X.end() );
  std::sort( Y.begin(), Y.end() );
  X = dconv.envelope( X ); 
  Y = dconv.envelope( Y ); 
  REQUIRE( dconv.isFullyConvex( X ) );
  REQUIRE( dconv.isFullyConvex( Y ) );
  WHEN( "Computing the envelope of X relative to Y and Y relative to X" ) {
    auto FC_X_rel_Y = dconv.relativeEnvelope( X, Y );
    auto FC_Y_rel_X = dconv.relativeEnvelope( Y, X );
    THEN( "Both sets are fully convex" ){
      REQUIRE( dconv.isFullyConvex( FC_X_rel_Y ) );
      REQUIRE( dconv.isFullyConvex( FC_Y_rel_X ) );
    }
    THEN( "There are inclusion rules between sets" ){
      CAPTURE( FC_X_rel_Y );
      CAPTURE( FC_Y_rel_X );
      REQUIRE( std::includes( Y.cbegin(), Y.cend(),
                              FC_X_rel_Y.cbegin(), FC_X_rel_Y.cend() ) );
      REQUIRE( std::includes( X.cbegin(), X.cend(),
                              FC_Y_rel_X.cbegin(), FC_Y_rel_X.cend() ) );
    }
  }
}


SCENARIO( "DigitalConvexity< Z3 > full subconvexity of triangles", "[subconvexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -5, -5, -5 ), Point( 5, 5, 5 ) );
  DConvexity dconv( Point( -6, -6, -6 ), Point( 6, 6, 6 ) );


  WHEN( "Computing many tetrahedra" ) {
    const unsigned int nb   = 50;
    unsigned int nb_fulldim = 0;
    unsigned int nb_ok_tri1 = 0;
    unsigned int nb_ok_tri2 = 0;
    for ( unsigned int l = 0; l < nb; ++l )
      {
        const Point a { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point b { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point c { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const Point d { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
        const std::vector<Point> pts = { a, b, c, d };
        const bool fulldim = dconv.isSimplexFullDimensional(pts.cbegin(), pts.cend());
        nb_fulldim += fulldim ? 1 : 0;
        if ( ! fulldim ) continue;
        auto simplex = dconv.makeSimplex( pts.cbegin(), pts.cend() );
        auto cover   = dconv.makeCellCover( simplex, 0, 3 );
	auto ls      = dconv.StarCvxH( pts );
        {
          unsigned int nb_subconvex1 = 0;
	  unsigned int nb_subconvex2 = 0;
          unsigned int nb_total      = 0;
          for ( unsigned int i = 0; i < 4; i++ )
            for ( unsigned int j = i+1; j < 4; j++ )
              for ( unsigned int k = j+1; k < 4; k++ )
                {
                  auto tri1 = dconv.makeSimplex({ pts[ i ], pts[ j ], pts[ k ] });
                  bool ok1  = dconv.isFullySubconvex( tri1, cover );
                  bool ok2  = dconv.isFullySubconvex( pts[ i ], pts[ j ], pts[ k ], ls );
                  nb_subconvex1 += ok1 ? 1 : 0;
                  nb_subconvex2 += ok2 ? 1 : 0;		  
                  nb_total      += 1;
                  if ( ! ok1 ) {
                    trace.info() << "****** TRIANGLE NOT SUBCONVEX ****" << std::endl; 
                    trace.info() << "splx v =" << a << b << c << d << std::endl;
                    trace.info() << "simplex=" << simplex << std::endl;
                    trace.info() << "tri v  =" << pts[ i ] << pts[ j ]
                                 << pts[ k ] << std::endl;
                    trace.info() << "tri1=" << tri1 << std::endl;
                  }
                  if ( ! ok2 ) {
                    trace.info() << "****** TRIANGLE 3D NOT SUBCONVEX ****" << std::endl; 
                    trace.info() << "splx v =" << a << b << c << d << std::endl;
                    trace.info() << "simplex=" << simplex << std::endl;
                    trace.info() << "tri v  =" << pts[ i ] << pts[ j ]
                                 << pts[ k ] << std::endl;
                  }
                }
          nb_ok_tri1 += ( nb_subconvex1 == nb_total ) ? 1 : 0;
          nb_ok_tri2 += ( nb_subconvex2 == nb_total ) ? 1 : 0;	  
        }
      }
    THEN( "All triangles of a tetrahedron should be subconvex to it." ) {
      REQUIRE( nb_ok_tri1 == nb_fulldim );
    }
    THEN( "All 3D triangles of a tetrahedron should be subconvex to it." ) {
      REQUIRE( nb_ok_tri2 == nb_fulldim );
    }
  }

}

SCENARIO( "DigitalConvexity< Z3 > full subconvexity of points and triangles", "[subconvexity][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -20, -20, -20 ), Point( 20, 20, 20 ) );
  DConvexity dconv ( Point( -21, -21, -21 ), Point( 21, 21, 21 ) );

  WHEN( "Computing many tetrahedra" ) {
    const unsigned int nb   = 50;
    unsigned int nb_total   = 0;
    unsigned int nb_ok_tri  = 0;
    unsigned int nb_subconvex1 = 0;
    unsigned int nb_subconvex2 = 0;
    for ( unsigned int l = 0; l < nb; ++l )
      {
        const Point a { (rand() % 10 - 10), (rand() % 10 - 10), (rand() % 10 - 10) };
        const Point b { (rand() % 20     ), (rand() % 20 - 10), (rand() % 20 - 10) };
        const Point c { (rand() % 20 - 10), (rand() % 20     ), (rand() % 20 - 10) };
        const Point d { (rand() % 20 - 10), (rand() % 20 - 10), (rand() % 20     ) };
        const std::vector<Point> pts = { a, b, c, d };
        const bool fulldim = dconv.isSimplexFullDimensional(pts.cbegin(), pts.cend());
        if ( ! fulldim ) continue;
        auto simplex = dconv.makeSimplex( pts.cbegin(), pts.cend() );
        auto cover   = dconv.makeCellCover( simplex, 0, 3 );
	auto ls      = dconv.StarCvxH( pts );
        {
          for ( unsigned int i = 0; i < 100; i++ )
	    {
	      const Point p { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Point q { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Point r { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Vector n = ( q - p ).crossProduct( r - p );
	      if ( n == Vector::zero ) continue;
	      auto tri1 = dconv.makeSimplex( { p, q, r } );
	      bool ok1  = dconv.isFullySubconvex( tri1, cover );
	      bool ok2  = dconv.isFullySubconvex( p, q, r, ls );
	      nb_subconvex1 += ok1 ? 1 : 0;
	      nb_subconvex2 += ok2 ? 1 : 0;		  
	      if ( ok1 != ok2 ) {
		std::cout << "***** FULL SUBCONVEXITY ERROR ON TRIANGLE ****" << std::endl;
		std::cout << "splx v =" << a << b << c << d << std::endl;
		std::cout << "simplex=" << simplex << std::endl;
		std::cout << "tri v  =" << p << q << r << std::endl;
		std::cout << "tri1=" << tri1 << std::endl;
		std::cout << "tri1 is fully subconvex: " << ( ok1 ? "YES" : "NO" ) << std::endl;
		std::cout << "3 points are fully subconvex: " << ( ok2 ? "YES" : "NO" ) << std::endl;
	      }
	      nb_ok_tri += ( ok1 == ok2 ) ? 1 : 0;
	      nb_total  += 1;
	    }
	}
      }
    THEN( "The number of triangles and point triplets subconvex to it should be equal." ) {
      REQUIRE( nb_subconvex1 == nb_subconvex2 );
    }
    THEN( "Full subconvexity should agree on every subset." ) {
      REQUIRE( nb_ok_tri == nb_total );
    }
  }
  
}

SCENARIO( "DigitalConvexity< Z3 > full covering of segments", "[full_cover][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  // typedef KSpace::Vector                   Vector;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -20, -20, -20 ), Point( 20, 20, 20 ) );
  DConvexity dconv ( Point( -21, -21, -21 ), Point( 21, 21, 21 ) );
  {
    Point a( 0, 0, 0 );
    Point b( 3, 2, 1 );
    auto LS = dconv.CoverCvxH( a, b );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 9 );
  }
  {
    Point a( 0, 0, 3 );
    Point b( 3, 0, 0 );
    auto LS = dconv.CoverCvxH( a, b );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 7 );
  }
  {
    Point a( 3, 0, 0 );
    Point b( 0, 2, 5 );
    auto LS = dconv.CoverCvxH( a, b );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 17 );
  }
}

SCENARIO( "DigitalConvexity< Z3 > full covering of triangles", "[full_cover][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
// typedef KSpace::Vector                   Vector;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -20, -20, -20 ), Point( 20, 20, 20 ) );
  DConvexity dconv ( Point( -21, -21, -21 ), Point( 21, 21, 21 ) );
  {
    Point a( 0, 0, 0 );
    Point b( 2, 1, 0 );
    Point c( 2, 2, 0 );  
    auto LS = dconv.CoverCvxH( a, b, c );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 10 );
  }
  {
    Point a( 0, 0, 1 );
    Point b( 2, 1, 0 );
    Point c( 2, 2, 0 );  
    auto LS = dconv.CoverCvxH( a, b, c );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 10 );
  }
  {
    Point a( 1, 0, 0 );
    Point b( 0, 1, 0 );
    Point c( 0, 0, 1 );  
    auto LS = dconv.CoverCvxH( a, b, c );
    auto P  = LS.toPointRange();
    CAPTURE( P );
    REQUIRE( P.size() == 7 );
  }
}



SCENARIO( "DigitalConvexity< Z3 > full subconvexity and full covering of triangles", "[subconvexity][3d][full_cover]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef KSpace::Vector                   Vector;
  typedef KSpace::Space                    Space;
  typedef HyperRectDomain< Space >         Domain;
  typedef DigitalConvexity< KSpace >       DConvexity;

  Domain     domain( Point( -20, -20, -20 ), Point( 20, 20, 20 ) );
  DConvexity dconv ( Point( -21, -21, -21 ), Point( 21, 21, 21 ) );

  WHEN( "Computing many tetrahedra" ) {
    const unsigned int nb     = 50;
    unsigned int nb_total     = 0;
    unsigned int nb_ok_tri    = 0;
    unsigned int nb_ok_seg    = 0;
    unsigned int nb_ko_seg    = 0;
    unsigned int nb_subconvex = 0;
    unsigned int nb_covered   = 0;
    for ( unsigned int l = 0; l < nb; ++l )
      {
        const Point a { (rand() % 10 - 10), (rand() % 10 - 10), (rand() % 10 - 10) };
        const Point b { (rand() % 20     ), (rand() % 20 - 10), (rand() % 20 - 10) };
        const Point c { (rand() % 20 - 10), (rand() % 20     ), (rand() % 20 - 10) };
        const Point d { (rand() % 20 - 10), (rand() % 20 - 10), (rand() % 20     ) };
        const std::vector<Point> pts = { a, b, c, d };
        const bool fulldim = dconv.isSimplexFullDimensional(pts.cbegin(), pts.cend());
        if ( ! fulldim ) continue;
        auto simplex = dconv.makeSimplex( pts.cbegin(), pts.cend() );
        auto cover   = dconv.makeCellCover( simplex, 0, 3 );
	auto ls      = dconv.StarCvxH( pts );
        {
          for ( unsigned int i = 0; i < 100; i++ )
	    {
	      const Point p { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Point q { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Point r { (rand() % 10 - 5), (rand() % 10 - 5), (rand() % 10 - 5) };
	      const Vector n = ( q - p ).crossProduct( r - p );
	      if ( n == Vector::zero ) continue;
	      auto tri1 = dconv.makeSimplex( { p, q, r } );
	      bool ok1  = dconv.isFullySubconvex( p, q, r, ls );
	      bool ok2  = dconv.isFullyCovered( p, q, r, ls );
	      if ( ok2 )
		{
		  // check covering of segments
		  if ( dconv.isFullyCovered( p, q, ls ) ) nb_ok_seg += 1;
		  else nb_ko_seg += 1;
		  if ( dconv.isFullyCovered( p, r, ls ) ) nb_ok_seg += 1;
		  else nb_ko_seg += 1;
		  if ( dconv.isFullyCovered( q, r, ls ) ) nb_ok_seg += 1;
		  else nb_ko_seg += 1;
		}
	      
	      nb_subconvex += ok1 ? 1 : 0;
	      nb_covered   += ok2 ? 1 : 0;		  
	      bool ok = ok1 == ok2;
	      if ( ! ok )
		{
		  std::cout << "***** FULL SUBCONVEXITY ERROR ON TRIANGLE ****" << std::endl;
		  std::cout << "splx v =" << a << b << c << d << std::endl;
		  std::cout << "simplex=" << simplex << std::endl;
		  std::cout << "tri v  =" << p << q << r << std::endl;
		  std::cout << "tri1=" << tri1 << std::endl;
		  std::cout << "3 points are fully subconvex: " << ( ok1 ? "YES" : "NO" ) << std::endl;
		  std::cout << "3 points are fully covered by star: " << ( ok2 ? "YES" : "NO" ) << std::endl;
		}
	      nb_ok_tri += ( ok ) ? 1 : 0;
	      nb_total  += 1;
	    }
	}
      }
    THEN( "The number of subconvex and covered triangles should be equal." ) {
      REQUIRE( nb_subconvex == nb_covered );
    }
    THEN( "Full subconvexity and covering should agree on every subset." ) {
      REQUIRE( nb_ok_tri == nb_total );
    }
    THEN( "When a triangle is fully covered, its edges are fully covered also." ) {
      REQUIRE( nb_ko_seg == 0 );
    }
  }
  
}

SCENARIO( "DigitalConvexity< Z3 > envelope bug", "[envelope][3d]" )
{
  typedef KhalimskySpaceND<3,int>          KSpace;
  typedef KSpace::Point                    Point;
  typedef DigitalConvexity< KSpace >       DConvexity;
  typedef AffineGeometry< Point >          Affine;
  typedef AffineBasis< Point >             Basis;

  DConvexity dconv( Point( -36, -36, -36 ), Point( 36, 36, 36 ) );

  WHEN( "Using basis B = (1, 0, -2) (1, 0, -1)" ) {
    std::vector< Point > b = { Point( 0, 0, 0), Point(1, 0, -2), Point(1, 0, -1) };
    const auto [ o, B ] = Affine::affineBasis( b );
    Point e  = functions::computeIndependentVector( B );
    Basis AB( Point( 0,0 ), b, Basis::Type::SCALED_REDUCED );
    bool parallel = AB.isParallel( e );
    const auto [ d, L, r ] = AB.decomposeVector( e );
    CAPTURE( B ); 
    CAPTURE( AB.basis() ); 
    CAPTURE( d ); 
    CAPTURE( L );
    CAPTURE( r );
    CAPTURE( e );
    REQUIRE( ! parallel );
    REQUIRE( r != Point::zero );
  }
  
  WHEN( "Computing the envelope Z of a digital set X with direct algorithm" ) {
    std::vector< Point > X = { Point(5, 1, 9), Point(8, 1, 8), Point(9, 1, 1) };
    auto Z = dconv.envelope( X );
    THEN( "Z is fully convex" ){
      CAPTURE( X );
      CAPTURE( dconv.depthLastEnvelope() );
      REQUIRE( dconv.isFullyConvex( Z ) );
    }
  }
}
