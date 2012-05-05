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
 * @file testConvexIntegerPolygon.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/04/19
 *
 * Functions for testing class ConvexIntegerPolygon.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#define DEBUG_ConvexIntegerPolygon

#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/ConvexIntegerPolygon.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ConvexIntegerPolygon.
///////////////////////////////////////////////////////////////////////////////

template <typename ConvexIntegerPolygon>
bool
checkCut( ConvexIntegerPolygon & cip, 
          typename ConvexIntegerPolygon::HalfSpace hs )
{
  trace.beginBlock ( "Check cut, see <cip.eps> and <cip2.eps>" );
  typedef typename ConvexIntegerPolygon::Space Space;
  typedef typename ConvexIntegerPolygon::Domain Domain;
  typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;
  typedef typename DigitalSet::ConstIterator ConstIterator;

  Domain d = cip.boundingBoxDomain();
#ifdef DEBUG_ConvexIntegerPolygon
  Board2D board;
  board << SetMode( d.className(), "Grid" ) << d;
  DigitalSet aSet( d );
  Shapes<Domain>::makeSetFromPointPredicate( aSet, hs );
  Color col1( 100, 100, 255 );
  Color col2( 180, 180, 255 );
  board << CustomStyle( aSet.className(), new CustomColors( col1, col2 ) )
        << aSet;
  board << SetMode( cip.className(), "Transparent" ) << cip;
  board.saveEPS( "cip.eps" );
  board.clear();
#endif

  DigitalSet cipSet( d );
  DigitalSet cipSet2( d );
  cip.getIncludedDigitalPoints( cipSet );
  cip.cut( hs );
  cip.getIncludedDigitalPoints( cipSet2 );
#ifdef DEBUG_ConvexIntegerPolygon
  board << SetMode( d.className(), "Grid" ) << d;
  board << CustomStyle( aSet.className(), new CustomColors( col1, col2 ) )
        << cipSet;
  board << CustomStyle( aSet.className(), new CustomColors( Color( 255, 180, 20 ), Color( 200, 170, 0 ) ) )
        << cipSet2;
  board << SetMode( cip.className(), "Transparent" ) << cip;
  board.saveEPS( "cip2.eps" );
#endif
  
  unsigned int nbok = 0;
  unsigned int nb = 0;
  for ( ConstIterator it = cipSet2.begin(), it_end = cipSet2.end();
        it != it_end; ++it )
    {
      nbok += ( cipSet.find( *it ) != cipSet.end() ) ? 1 : 0; 
      ++nb;
      nbok += hs( *it );
      ++nb;
    }
  for ( ConstIterator it = cipSet.begin(), it_end = cipSet.end();
        it != it_end; ++it )
    {
      if ( cipSet2.find( *it ) == cipSet2.end() )
        nbok += ! hs( *it );
      else
        nbok += hs( *it );
      ++nb;
    }
  trace.info() << "(" << nbok << "/" << nb << ")" 
               << " cip.size()=" << cip.size()
               << " #before=" << cipSet.size() 
               << " #after=" << cipSet2.size() 
               << std::endl;
  trace.endBlock();
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
template <typename Space>
bool testConvexIntegerPolygon()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ConvexIntegerPolygon area and centroid" );
  typedef typename Space::Point Point;
  typedef typename Space::Vector Vector;
  typedef typename Space::Integer Integer;
  typedef ConvexIntegerPolygon<Space> CIP;
  typedef typename CIP::Point3I Point3I;
  typedef typename CIP::Domain Domain;
  typedef typename CIP::HalfSpace HalfSpace;
  typedef typename CIP::Iterator Iterator;
  typedef typename CIP::SizeCouple SizeCouple;
  typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;

  CIP cip;
  cip.push_back( Point( 0, 0 ) );
  cip.push_back( Point( 5, 0 ) );
  cip.push_back( Point( 0, 3 ) );
  Integer area2 = cip.twiceArea();
  trace.info() << "- 2*area   = " << area2 << std::endl;
  ++nb, nbok += ( area2 == 15 ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "2*area == 15" << std::endl;
  Point3I c = cip.centroid( area2 );
  trace.info() << "- centroid = " << c << std::endl;
  ++nb, nbok += ( c == Point3I( 75, 45, 45 ) ) ? 1 : 0; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "centroid == [75,45,45]" << std::endl;
  Domain d = cip.boundingBoxDomain();
  trace.info() << "- domain = " << d << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Output ConvexIntegerPolygon in <cip.eps>" );
  cip.push_back( Point( -4, 4 ) );
  cip.push_back( Point( -7, 2 ) );
  cip.push_back( Point( -5, 1 ) );
  Board2D board;
  d = cip.boundingBoxDomain();
  board << SetMode( d.className(), "Grid" ) << d;
  DigitalSet aSet( d );
  HalfSpace h( Vector( 1, 3 ), 8 );
  //HalfSpace h = cip.halfSpace( ++cip.begin() );
  Shapes<Domain>::makeSetFromPointPredicate( aSet, h );
  Color col1( 100, 100, 255 );
  Color col2( 180, 180, 255 );
  board << CustomStyle( aSet.className(), new CustomColors( col1, col2 ) )
        << aSet;
  cip.getIncludedDigitalPoints( aSet );
  board << CustomStyle( aSet.className(), new CustomColors( Color( 255, 180, 20 ), Color( 200, 130, 0 ) ) )
        << aSet;
  board << SetMode( cip.className(), "Transparent" ) << cip;
  Iterator itA1, itB2;
  SizeCouple nbs = cip.findCut( itA1, itB2, h );
  Iterator itB1 = itA1; ++itB1;
  if ( itB1 == cip.end() ) itB1 = cip.begin();
  Iterator itA2 = itB2; ++itA2;
  if ( itA2 == cip.end() ) itA2 = cip.begin();
  Color col3( 0, 255, 0 );
  Color col4( 255, 0, 0 );
  board << CustomStyle( Point().className(), new CustomColors( col3, col3 ) )
        << *itA1 << *itA2;
  board << CustomStyle( Point().className(), new CustomColors( col4, col4 ) )
        << *itB1 << *itB2;
  board.saveEPS( "cip.eps" );
  board.saveSVG( "cip.svg" );
  trace.endBlock();

  trace.beginBlock ( "Output cut ConvexIntegerPolygon in <cip2.eps>" );
  board.clear();
  board << SetMode( d.className(), "Grid" ) << d;
  board << SetMode( cip.className(), "Transparent" ) << cip;
  bool wasCut = cip.cut( h );
  board << SetMode( cip.className(), "Filled" ) << cip;
  board.saveEPS( "cip2.eps" );
  board.saveSVG( "cip2.svg" );
  trace.endBlock();

  checkCut( cip, HalfSpace( Vector( -2, 3 ), 4 ) );

  return nbok == nb;
}

int myRandom( int nb )
{
  return random() % nb;
}

/**
 * Example of a test. To be completed.
 *
 */
template <typename Space>
bool exhaustiveTestConvexIntegerPolygon()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ConvexIntegerPolygon area and centroid" );
  typedef typename Space::Point Point;
  typedef typename Space::Vector Vector;
  typedef typename Space::Integer Integer;
  typedef ConvexIntegerPolygon<Space> CIP;
  typedef typename CIP::Point3I Point3I;
  typedef typename CIP::Domain Domain;
  typedef typename CIP::HalfSpace HalfSpace;
  typedef typename CIP::Iterator Iterator;
  typedef typename CIP::SizeCouple SizeCouple;
  typedef typename DigitalSetSelector< Domain, BIG_DS+HIGH_BEL_DS >::Type DigitalSet;

  CIP cip;
  cip.push_back( Point( 0, 0 ) );
  cip.push_back( Point( 8, -3 ) );
  cip.push_back( Point( 17, 2 ) );
  cip.push_back( Point( 21, 13 ) );
  cip.push_back( Point( 13, 19 ) );
  cip.push_back( Point( 6, 17 ) );
  cip.push_back( Point( -3, 6 ) );
  Integer area2 = cip.twiceArea();
  trace.info() << "- 2*area   = " << area2 << std::endl;
  Point3I c = cip.centroid( area2 );
  trace.info() << "- centroid = " << c << std::endl;
  Domain d = cip.boundingBoxDomain();
  trace.info() << "- domain = " << d << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Check cuts..." );
  for ( unsigned int j = 0; j < 1000; ++j )
    {
      CIP cip2 = cip;
      for ( unsigned int i = 0; i < 10; ++i )
        {
          int x = 0;
          int y = 0;
          while ( ( x == 0 ) && ( y == 0 ) )
            {
              x = myRandom( 31 ) - 15;
              y = myRandom( 31 ) - 15;
            }
          int g = IntegerComputer<int>::staticGcd( x , y );
          x /= g; y /= g;
          int c = myRandom( 4 ) *x + myRandom( 4 ) * y + myRandom( 20 ) - 20;
          HalfSpace h( Vector( x, y ), 8 );
          trace.info() << "[" << j << " size=" << cip2.size() << "]"
                       << " cut by (" << x << "," << y << ")," << c << std::endl;
          ++nb, nbok += checkCut( cip2, h ) ? 1 : 0;
          trace.info() << "(" << nbok << "/" << nb << ") cuts" << std::endl;
          //std::cerr << " " << cip2.size() << flush;
          if ( nb != nbok ) break;
        }
      std::cerr << std::endl;
      if ( nb != nbok ) break;
    }
  trace.endBlock();
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int, char** )
{
  trace.beginBlock ( "Testing class ConvexIntegerPolygon" );

  typedef SpaceND<2, DGtal::int64_t> Z2;
  typedef SpaceND<2, DGtal::BigInteger> Z2I;
  bool res = testConvexIntegerPolygon<Z2>()
    && testConvexIntegerPolygon<Z2I>()
    && exhaustiveTestConvexIntegerPolygon<Z2>();
  //&& exhaustiveTestConvexIntegerPolygon<Z2I>();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
