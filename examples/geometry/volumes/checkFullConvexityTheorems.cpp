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
 * @file geometry/volumes/checkFullConvexityTheorems.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/20
 *
 * An example file named checkFullConvexityTheorems
 *
 * This file is part of the DGtal library.
 */


/**
   This example shows how to use the fully convex envelope to build a
   digital polyhedron from an arbitrary mesh. All faces have also the
   property that their points lies in the naive/standard plane defined
   by its vertices. It uses DigitalConvexity::relativeEnvelope for
   computations.
   

 \example geometry/volumes/checkFullConvexityTheorems.cpp
 */


///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <queue>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

template <typename Point>
void makeRandom( Point& p, int width )
{
  for ( Dimension i = 0; i < Point::dimension; i++ )
    p[ i ] = rand() % width;
}

template <typename Point>
void makeRandomRange( std::vector< Point >& X, int nb, int width )
{
  X.resize( nb );
  for ( int i = 0; i < nb; i++ )
    makeRandom( X[ i ], width );
}

template <typename ProjectedPoint, typename Point >
void project( ProjectedPoint& pp, const Point& p, Dimension a )
{
  Dimension j = 0;
  for ( Dimension i = 0; i < Point::dimension; i++ )
    if ( i != a ) pp[ j++ ] = p[ i ];
}

template <typename ProjectedPoint, typename Point >
void projectRange( std::vector< ProjectedPoint >& pp,
                   const std::vector< Point > & p, Dimension a )
{
  pp.resize( p.size() );
  for ( auto i = 0; i < p.size(); i++ )
    project( pp[ i ], p[ i ], a );
  std::sort( pp.begin(), pp.end() );
  auto last = std::unique( pp.begin(), pp.end() );
  pp.erase( last, pp.end() );
}


// @param width the width of the domain
template <typename Space>
bool
checkSkelStarCvxHFullConvexity( int width )
{
  typedef typename Space::Integer Integer;
  typedef DGtal::KhalimskySpaceND< Space::dimension, Integer > KSpace;
  typedef DGtal::DigitalConvexity< KSpace > DConvexity;
  typedef typename KSpace::Point  Point;
  typedef std::vector<Point>      PointRange;

  // Generate a random polytope in the specified domain
  Point lo = Point::zero;
  Point hi = Point::diagonal( width );
  DConvexity dconv( lo, hi );
  std::vector< Point > X;
  int   nb = Space::dimension + rand() % 7;
  makeRandomRange( X, nb, width );
  auto  C  = dconv.StarCvxH( X );
  auto  E  = dconv.envelope( X );
  auto  Y  = C.extremaOfCells();
  bool  ok1 = dconv.isFullyConvex( E );
  if ( ! ok1 )
    trace.warning() << "FC*(X) is not fully convex !" << std::endl;
  bool  ok2 = dconv.isFullyConvex( Y );
  if ( ! ok2 )
    {
      trace.warning() << "Extr(Star(CvxH(X))) is not fully convex !" << std::endl;
      for ( auto p : Y ) std::cout << " " << p;
      trace.warning() << std::endl;
    }
  bool ok3 = std::includes( Y.cbegin(), Y.cend(), E.cbegin(), E.cend() );
  trace.info() << "#X=" << X.size()
               << " #FC*(X)=" << E.size() << ( ok1 ? "/FC" : "/ERROR" )
               << " #Extr(Star(CvxH(X)))=" << Y.size()
               << ( ok2 ? "/FC" : "/ERROR" )
               << ( ok3 ? " FC*(X) subset Extr(Star(CvxH(X)))" : " Inclusion ERROR" )
               << std::endl;
  return ok1 && ok2 && ok3;
}

// @param width the width of the domain
template <typename Space>
bool
checkProjectionFullConvexity( int width )
{
  typedef typename Space::Integer    Integer;
  typedef DGtal::KhalimskySpaceND< Space::dimension, Integer > KSpace;
  typedef DGtal::DigitalConvexity< KSpace > DConvexity;
  typedef typename KSpace::Point     Point;
  typedef std::vector<Point>         PointRange;
  typedef DGtal::KhalimskySpaceND< Space::dimension-1, Integer > ProjKSpace;
  typedef DGtal::DigitalConvexity< ProjKSpace > ProjDConvexity;
  typedef typename ProjKSpace::Point ProjPoint;
  typedef std::vector<ProjPoint>     ProjPointRange;

  // Generate a random polytope in the specified domain
  Point lo = Point::zero;
  Point hi = Point::diagonal( width );
  DConvexity dconv( lo, hi );
  std::vector< Point > X;
  int   n = Space::dimension + rand() % 7;
  makeRandomRange( X, n, width );
  auto  E  = dconv.envelope( X );
  unsigned int nb    = 0;
  unsigned int nb_ok = 0;
  for ( Dimension a = 0; a < Space::dimension; a++ )
    {
      ProjPoint plo, phi;
      project( plo, lo, a ); 
      project( phi, hi, a ); 
      ProjDConvexity pdconv( plo, phi );
      std::vector< ProjPoint > PE;
      projectRange( PE, E, a );
      bool ok = pdconv.isFullyConvex( PE );
      if ( !ok )
        trace.warning() << "Projection is not fully convex !" << std::endl;
      nb_ok  += ok ? 1 : 0;
      nb     += 1;
      std::cout << "#E=" << E.size() << " #Proj(E)=" << PE.size()
                << (ok ? "/FC" : "/ERROR" ) << std::endl;
    }
  return nb_ok == nb;
}

int main( int argc, char* argv[] )
{
  int NB_TEST1 = 5;
  int NB_TEST2 = 5;
  {
    trace.beginBlock( "Check SkelStarCvxH(X) full convexity 2D" );
    typedef DGtal::SpaceND< 2, int > Space;
    unsigned int nb    = 0;
    unsigned int nb_ok = 0;
    for ( int i = 0; i < NB_TEST1; i++ )
      {
        nb_ok += checkSkelStarCvxHFullConvexity< Space >( 100 ) ? 1 : 0;
        nb    += 1;
      }
    trace.info() << nb_ok << "/" << nb << " OK tests" << std::endl;
    trace.endBlock();
  }
  {
    trace.beginBlock( "Check SkelStarCvxH(X) full convexity 3D" );
    typedef DGtal::SpaceND< 3, int > Space;
    unsigned int nb    = 0;
    unsigned int nb_ok = 0;
    for ( int i = 0; i < NB_TEST1; i++ )
      {
        nb_ok += checkSkelStarCvxHFullConvexity< Space >( 30 ) ? 1 : 0;
        nb    += 1;
      }
    trace.info() << nb_ok << "/" << nb << " OK tests" << std::endl;
    trace.endBlock();
  }
  {
    trace.beginBlock( "Check SkelStarCvxH(X) full convexity 4D" );
    typedef DGtal::SpaceND< 4, int > Space;
    unsigned int nb    = 0;
    unsigned int nb_ok = 0;
    for ( int i = 0; i < NB_TEST1; i++ )
      {
        nb_ok += checkSkelStarCvxHFullConvexity< Space >( 10 ) ? 1 : 0;
        nb    += 1;
      }
    trace.info() << nb_ok << "/" << nb << " OK tests" << std::endl;
    trace.endBlock();
  }
  {
    trace.beginBlock( "Check Projection full convexity 3D" );
    typedef DGtal::SpaceND< 3, int > Space;
    unsigned int nb    = 0;
    unsigned int nb_ok = 0;
    for ( int i = 0; i < NB_TEST2; i++ )
      {
        nb_ok += checkProjectionFullConvexity< Space >( 30 ) ? 1 : 0;
        nb    += 1;
      }
    trace.info() << nb_ok << "/" << nb << " OK tests" << std::endl;
    trace.endBlock();
  }
  {
    trace.beginBlock( "Check Projection full convexity 4D" );
    typedef DGtal::SpaceND< 4, int > Space;
    unsigned int nb    = 0;
    unsigned int nb_ok = 0;
    for ( int i = 0; i < NB_TEST2; i++ )
      {
        nb_ok += checkProjectionFullConvexity< Space >( 10 ) ? 1 : 0;
        nb    += 1;
      }
    trace.info() << nb_ok << "/" << nb << " OK tests" << std::endl;
    trace.endBlock();
  }

  return 0;
}
