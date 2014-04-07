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
 * @file testCellularGridSpaceND.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/02/08
 *
 * Functions for testing class CellularGridSpaceND.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <boost/math/special_functions/binomial.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellularGridSpaceND.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename KSpace>
bool testCellularGridSpaceND()
{
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::SCell SCell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::DirIterator DirIterator;
  typedef typename KSpace::Cells Cells;
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing block KSpace instantiation and scan ..." );
  KSpace K;
  int xlow[ 4 ] = { -3, -2, -2, -1 };
  int xhigh[ 4 ] = { 5, 3, 2, 3 };
  Point low( xlow );
  Point high( xhigh );
  bool space_ok = K.init( low, high, true );
  nbok += space_ok ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "K.init( low, high )" << std::endl;
  trace.info() << "K.dim()=" << K.dimension << endl;
  int spel[ 4 ] = { 1, 1, 1, 1 }; // pixel
  Point kp( spel );
  Cell center = K.uCell( kp );
  Cell c1 = K.uCell( kp );
  Cell clow = K.uCell( low, kp );
  Cell chigh = K.uCell( high, kp );
  trace.info() << c1 << clow << chigh
         << " topo(c1)=" << K.uTopology( c1 ) << " dirs=";
  for ( DirIterator q = K.uDirs( clow ); q != 0; ++q )
    trace.info() << " " << *q;
  trace.info() << endl;
  Cell f = K.uFirst( c1 );
  Cell l = K.uLast( c1 );
  trace.info() << "Loop in " << clow << chigh << endl;
  c1 = f;
  unsigned int nbelems = 0;
  do {
    ++nbelems;
    // trace.info() << c1;
  } while ( K.uNext( c1, f, l ) );
  trace.info() << " -> " << nbelems << " elements." << endl;
  unsigned int exp_nbelems = 1;
  for ( Dimension i = 0; i < K.dimension; ++i )
    exp_nbelems *= K.size( i );
  nbok += nbelems == exp_nbelems ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << nbelems << " scanned elements == "
         << exp_nbelems << " space size."
         << std::endl;
  trace.endBlock();
  trace.beginBlock ( "Testing neighborhoods in KSpace..." );
  Cells N = K.uNeighborhood( center );
  nbok += N.size() == ( K.dimension*2 + 1 ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << N.size() << "(neighborhood size) == "
         << ( K.dimension*2 + 1 ) << "(2*dim()+1)" << endl;
  Cells Np = K.uProperNeighborhood( center );
  nbok += Np.size() == ( K.dimension*2 ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << Np.size() << "(proper neighborhood size) == "
         << ( K.dimension*2 ) << "(2*dim())" << endl;
  trace.endBlock();

  trace.beginBlock ( "Testing faces in KSpace..." );
  Cells Nf = K.uFaces( center );
  nbok += Nf.size() == ceil( std::pow( 3.0 ,(int) K.dimension ) - 1 ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << Nf.size() << "(faces size) == "
         << floor( std::pow( 3.0, (int)K.dimension ) - 1 ) << "(3^dim()-1)" << endl;
  trace.endBlock();

  trace.beginBlock ( "Testing block Incidence in KSpace..." );
  SCell sspel = K.sCell( kp, K.POS );
  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
  if ( *q1 != *q2 )
    {
      SCell s0 = K.sIncident( sspel, *q1, true );
      SCell s1 = K.sIncident( sspel, *q2, true );
      SCell l10 = K.sIncident( s0, *q2, true );
      SCell l01 = K.sIncident( s1, *q1, true );
      trace.info() << "D+_" << *q2 << "(D+_" << *q1 << "(V))=" << l10
       << " D+_" << *q1 << "(D+_" << *q2 << "(V))=" << l01
       << endl;
      nbok += l10 == K.sOpp( l01 ) ? 1 : 0;
      nb++;
    }
      }
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "anti-commutativity of incidence operators." << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing direct Incidence in KSpace..." );
  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
  if ( *q1 != *q2 )
    {
      SCell s0 = K.sDirectIncident( sspel, *q1 );
      SCell l10 = K.sDirectIncident( s0, *q2 );
      SCell s1 = K.sDirectIncident( sspel, *q2 );
      SCell l01 = K.sDirectIncident( s1, *q1 );
      trace.info() << "Dd_" << *q2 << "(Dd_" << *q1 << "(V))=" << l10
       << " Dd_" << *q1 << "(Dd_" << *q2 << "(V))=" << l01
       << endl;
      nbok += l10 != l01 ? 1 : 0;
      nbok += K.sSign( s0 ) == K.POS ? 1 : 0;
      nbok += K.sSign( s1 ) == K.POS ? 1 : 0;
      nbok += K.sSign( l10 ) == K.POS ? 1 : 0;
      nbok += K.sSign( l01 ) == K.POS ? 1 : 0;
      nbok += s0 == K.sIncident( sspel, *q1, K.sDirect( sspel, *q1 ) )
        ? 1 : 0;
      nbok += s1 == K.sIncident( sspel, *q2, K.sDirect( sspel, *q2 ) )
        ? 1 : 0;
      nbok += l10 == K.sIncident( s0, *q2, K.sDirect( s0, *q2 ) )
        ? 1 : 0;
      nbok += l01 == K.sIncident( s1, *q1, K.sDirect( s1, *q1 ) )
        ? 1 : 0;
      nb += 9;
    }
      }
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "correctness of direct and indirect orientations." << std::endl;

  trace.endBlock();


  return nbok == nb;
}


template <typename KSpace>
bool testSurfelAdjacency()
{
  typedef typename KSpace::Integer Integer;
  typedef typename KSpace::SCell SCell;
  typedef typename KSpace::Point Point;
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing block KSpace instantiation and scan ..." );
  KSpace K;
  int xlow[ 4 ] = { -3, -3, -3, -3 };
  int xhigh[ 4 ] = { 5, 3, 3, 3 };
  Point low( xlow );
  Point high( xhigh );
  bool space_ok = K.init( low, high, true );
  nbok += space_ok ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "K.init( low, high )" << std::endl;
  trace.info() << "K.dim()=" << K.dimension << endl;
  trace.endBlock();
  trace.beginBlock ( "Testing surfel adjacency ..." );
  SurfelAdjacency<KSpace::dimension> SAdj( true );
  for ( Dimension i = 0; i < K.dimension; ++i )
    for ( Dimension j = 0; j < K.dimension; ++j )
      if ( i != j )
  trace.info() << "(" << i << "," << j << ")="
         << ( SAdj.getAdjacency( i, j ) ? "i2e" : "e2i" );
  trace.info() << endl;
  trace.endBlock();

  int spel[ 4 ] = { 1, 1, 1, 1 }; // pixel
  Point kp( spel );
  SCell sspel = K.sCell( kp, K.POS );
  trace.beginBlock ( "Testing surfel directness ..." );
  for ( Dimension k = 0; k < K.dimension; ++k )
    {
      SCell surfel = K.sIncident( sspel, k, true );
      SCell innerspel = K.sDirectIncident( surfel, K.sOrthDir( surfel ) );
      trace.info() << "spel=" << sspel << " surfel=" << surfel
       << " innerspel=" << innerspel << endl;
      nbok += sspel == innerspel ? 1 : 0;
      nb++;
      trace.info() << "(" << nbok << "/" << nb << ") "
       << "spel == innerspel" << std::endl;
      surfel = K.sIncident( sspel, k, false );
      innerspel = K.sDirectIncident( surfel, K.sOrthDir( surfel ) );
      trace.info() << "spel=" << sspel << " surfel=" << surfel
       << " innerspel=" << innerspel << endl;
      nbok += sspel == innerspel ? 1 : 0;
      nb++;
      trace.info() << "(" << nbok << "/" << nb << ") "
       << "spel == innerspel" << std::endl;
    }
  trace.endBlock();

  SurfelNeighborhood<KSpace> SN;
  trace.beginBlock ( "Testing surfel neighborhood ..." );
  SCell surfel = K.sIncident( sspel, 0, false );
  SN.init( &K, &SAdj, surfel );
  trace.info() << "surfel      =" << surfel << endl;
  trace.info() << "follower1(+)=" << SN.follower1( 1, true ) << endl;
  trace.info() << "follower2(+)=" << SN.follower2( 1, true ) << endl;
  trace.info() << "follower3(+)=" << SN.follower3( 1, true ) << endl;
  trace.info() << "follower1(-)=" << SN.follower1( 1, false ) << endl;
  trace.info() << "follower2(-)=" << SN.follower2( 1, false ) << endl;
  trace.info() << "follower3(-)=" << SN.follower3( 1, false ) << endl;
  trace.endBlock();

  trace.beginBlock ( "Testing surface tracking ..." );
  typedef SpaceND< KSpace::dimension, Integer > Space;
  typedef HyperRectDomain<Space> Domain;
  typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
  Domain domain( low, high );
  DigitalSet shape_set( domain );
  int center[ 4 ] = { 1, 0, 0, 0 }; // pixel
  Point pcenter( center );
  Shapes<Domain>::addNorm1Ball( shape_set, pcenter, 1 );
  trace.info() << "surfel      = " << surfel << endl;
  SCell other1, other2;
  SN.getAdjacentOnDigitalSet( other1, shape_set, 1, K.sDirect( surfel, 1 ) );
  SN.getAdjacentOnDigitalSet( other2, shape_set, 1, !K.sDirect( surfel, 1 ) );
  trace.info() << "directNext  = " << other1 << endl;
  trace.info() << "indirectNext= " << other2 << endl;
  std::set<SCell> bdry;

  // surfel = Surfaces<KSpace>::findABel( K, shape_set );

  Surfaces<KSpace>::trackBoundary( bdry,
           K, SAdj, shape_set, surfel );
  trace.info() << "tracking finished, size=" << bdry.size()
         << ", should be " << 2*K.dimension*(2*K.dimension-1) << endl;
  nbok += bdry.size() == ( 2*K.dimension*(2*K.dimension-1) ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "bdry.size() == ( 2*K.dimension*(2*K.dimension-1) )"
         << std::endl;
  std::set<SCell> bdry_direct;
  Surfaces<KSpace>::trackClosedBoundary( bdry_direct,
           K, SAdj, shape_set, surfel );
  trace.info() << "fast direct tracking finished, size=" << bdry_direct.size()
         << ", should be " << 2*K.dimension*(2*K.dimension-1) << endl;
  nbok += bdry_direct.size() == ( 2*K.dimension*(2*K.dimension-1) ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "bdry_direct.size() == ( 2*K.dimension*(2*K.dimension-1) )"
         << std::endl;

  trace.endBlock();
  if ( K.dimension == 2 )
    {
      Board2D board;
      board.setUnit( LibBoard::Board::UCentimeter );
      board << SetMode( domain.className(), "Paving" )
      << domain;
      for ( typename std::set<SCell>::const_iterator it = bdry_direct.begin(),
        it_end = bdry_direct.end(); it != it_end; ++it )
  board << *it;
      board.saveEPS( "cells-2.eps" );
      board.saveSVG( "cells-2.svg" );
    }
  return nbok == nb;
}

template <typename KSpace>
bool testCellDrawOnBoard()
{
  typedef typename KSpace::Integer Integer;
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::SCell SCell;
  typedef typename KSpace::Point Point;
  typedef SpaceND<2, Integer> Z2;
  typedef HyperRectDomain<Z2> Domain;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  trace.beginBlock ( "Testing cell draw on digital board ..." );
  KSpace K;
  int xlow[ 4 ] = { -3, -3 };
  int xhigh[ 4 ] = { 5, 3 };
  Point low( xlow );
  Point high( xhigh );
  bool space_ok = K.init( low, high, true );
  Domain domain( low, high );
  Board2D board;
  board.setUnit( LibBoard::Board::UCentimeter );
  board << SetMode( domain.className(), "Paving" )
  << domain;
  int spel[ 2 ] = { 1, 1 }; // pixel 0,0
  Point kp( spel );
  Cell uspel = K.uCell( kp );
  board << uspel
  << low << high
  << K.uIncident( uspel, 0, false )
  << K.uIncident( uspel, 1, false );
  int spel2[ 2 ] = { 5, 1 }; // pixel 2,0
  Point kp2( spel2 );
  SCell sspel2 = K.sCell( kp2, K.POS );
  board << CustomStyle( sspel2.className(),
      new CustomPen( Color( 200, 0, 0 ),
               Color( 255, 100, 100 ),
               2.0,
               Board2D::Shape::SolidStyle ) )
  << sspel2
      << K.sIncident( sspel2, 0, K.sDirect( sspel2, 0 ) )
  << K.sIncident( sspel2, 1, K.sDirect( sspel2, 0 ) );
  board.saveEPS( "cells-1.eps" );
  board.saveSVG( "cells-1.svg" );
  trace.endBlock();
  board.clear();
  board << domain;
  SCell slinel0 = K.sIncident( sspel2, 0, K.sDirect( sspel2, 0 ) );
  SCell spointel01 = K.sIncident( slinel0, 1, K.sDirect( slinel0, 1 ) );
  board << CustomStyle( sspel2.className(),
      new CustomColors( Color( 200, 0, 0 ),
            Color( 255, 100, 100 ) ) )
  << sspel2
  << CustomStyle( slinel0.className(),
      new CustomColors( Color( 0, 200, 0 ),
            Color( 100, 255, 100 ) ) )
  << slinel0
  << CustomStyle( spointel01.className(),
      new CustomColors( Color( 0, 0, 200 ),
            Color( 100, 100, 255 ) ) )
  << spointel01;
  board.saveEPS( "cells-3.eps" );
  board.saveSVG( "cells-3.svg" );

  return ((space_ok) && (nbok == nb));
}

template <typename KSpace>
bool testFindABel()
{
  typedef typename KSpace::Point Point;
  typedef SpaceND< KSpace::dimension, typename KSpace::Integer > Space;
  typedef HyperRectDomain<Space> Domain;
  typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
  typedef typename KSpace::SCell SCell;

  trace.beginBlock("Test FindABel");
  Point low(-3,-3,-3), high(3,3,3);
  Domain domain( low, high );
  DigitalSet shape_set( domain );
  KSpace K;
  K.init( low, high, true );

  Point p000(0,0,0), p001(0,0,1), p010(0,1,0), p011(0,1,1),
        p100(1,0,0), p101(1,0,1), p110(1,1,0), p111(1,1,1);

  shape_set.insert( p000 );
  shape_set.insert( p100 );

  Surfaces<KSpace>::findABel( K, shape_set , p000 , p011 );
  Surfaces<KSpace>::findABel( K, shape_set , p000 , p110 );
  Surfaces<KSpace>::findABel( K, shape_set , p000 , p111 );
  Surfaces<KSpace>::findABel( K, shape_set , p000 , p101 );
  SCell s010 = Surfaces<KSpace>::findABel( K, shape_set , p000 , p010 );
  SCell s001 = Surfaces<KSpace>::findABel( K, shape_set , p000 , p001 );

  trace.endBlock();
  return ( (s010 == SCell( Point(1,2,1), true  ) ) &&
           (s001 == SCell( Point(1,1,2), false ) ) );
}

template <typename KSpace>
bool testCellularGridSpaceNDFaces()
{
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::Cells Cells;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Dimension N = KSpace::dimension;
  Point low = Point::diagonal(-5);
  Point high = Point::diagonal(5);
  KSpace K;
  K.init( low, high, true );
  Cell vox = K.uSpel( Point::zero );
  Cells faces = K.uFaces( vox );
  // Check that there is no duplicates.
  trace.beginBlock( "Check CellularGridSpaceND::uFaces" );
  for ( Dimension k = 0; k < N; ++k )
    {
      trace.info() << "[" << k << "]";
      DGtal::int64_t nf = 0;
      for ( typename Cells::const_iterator it = faces.begin(), itE = faces.end(); it != itE; ++it )
        if ( K.uDim( *it ) == k ) { std::cout << " " << *it; ++nf; }
      trace.info() << " -> " << nf << std::endl;
      // Number of k-faces of N-cube is binom(n,k)*2^(n-k)
      DGtal::int64_t exp_nf = (DGtal::int64_t) round( boost::math::binomial_coefficient<double>(N, k) );
      exp_nf <<= N-k;
      ++nb, nbok += ( nf == exp_nf );
      trace.info() << "(" << nbok << "/" << nb << ") "
                   << nf << " == " << exp_nf << " (Number of " << k << "-cells faces of a " << N << "-cell)"
                   << std::endl;
    }
  trace.endBlock();
  return nb == nbok;
}

template <typename KSpace>
bool testCellularGridSpaceNDCoFaces()
{
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::Cells Cells;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Dimension N = KSpace::dimension;
  Point low = Point::diagonal(-5);
  Point high = Point::diagonal(5);
  KSpace K;
  K.init( low, high, true );
  Cell pointel = K.uPointel( Point::zero );
  Cells cofaces = K.uCoFaces( pointel );
  // Check that there is no duplicates.
  trace.beginBlock( "Check CellularGridSpaceND::uCoFaces" );
  for ( Dimension k = 1; k <= N; ++k )
    {
      trace.info() << "[" << k << "]";
      DGtal::int64_t nf = 0;
      for ( typename Cells::const_iterator it = cofaces.begin(), itE = cofaces.end(); it != itE; ++it )
        if ( K.uDim( *it ) == k ) { std::cout << " " << *it; ++nf; }
      trace.info() << " -> " << nf << std::endl;
      // Number of k-faces of N-cube is binom(n,k)*2^(n-k)
      DGtal::int64_t exp_nf = (DGtal::int64_t) round( boost::math::binomial_coefficient<double>(N, N-k) );
      exp_nf <<= k;
      ++nb, nbok += ( nf == exp_nf );
      trace.info() << "(" << nbok << "/" << nb << ") "
                   << nf << " == " << exp_nf << " (Number of " << k << "-cells cofaces of a " << 0 << "-cell)"
                   << std::endl;
    }
  trace.endBlock();
  return nb == nbok;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CellularGridSpaceND" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  typedef KhalimskySpaceND<2> K2;
  typedef KhalimskySpaceND<3> K3;
  typedef KhalimskySpaceND<4> K4;
  BOOST_CONCEPT_ASSERT(( CCellularGridSpaceND< K2 > ));
  BOOST_CONCEPT_ASSERT(( CCellularGridSpaceND< K3 > ));
  BOOST_CONCEPT_ASSERT(( CCellularGridSpaceND< K4 > ));

  bool res = testCellularGridSpaceND<K2>()
    && testCellularGridSpaceND<K3>()
    && testCellularGridSpaceND<K4>()
    && testSurfelAdjacency<K2>()
    && testSurfelAdjacency<K3>()
    && testSurfelAdjacency<K4>()
    && testCellDrawOnBoard<K2>()
    && testFindABel<K3>()
    && testCellularGridSpaceNDFaces<K2>()
    && testCellularGridSpaceNDFaces<K3>()
    && testCellularGridSpaceNDFaces<K4>()
    && testCellularGridSpaceNDCoFaces<K2>()
    && testCellularGridSpaceNDCoFaces<K3>()
    && testCellularGridSpaceNDCoFaces<K4>();

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
