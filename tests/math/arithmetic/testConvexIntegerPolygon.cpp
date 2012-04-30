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
  board << SetMode( cip.className(), "Transparent" ) << cip;
  Iterator itA1, itB2;
  unsigned int nbWithin = cip.findCut( itA1, itB2, h );
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
    && testConvexIntegerPolygon<Z2I>();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
