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
 * @file test_DigitalSet.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France

 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et
 * Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/01
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_DigitalSet <p>
 * Aim: simple tests of models of \ref CDigitalSet
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/kernel/sets/CDigitalSetArchetype.h"
#include "DGtal/kernel/domains/CDomainArchetype.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/kernel/sets/DigitalSetDomain.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/io/boards/Board2D.h"


using namespace DGtal;
using namespace std;


#define INBLOCK_TEST(x) \
  nbok += ( x ) ? 1 : 0; \
  nb++; \
  trace.info() << "(" << nbok << "/" << nb << ") " \
  << #x << std::endl;

#define INBLOCK_TEST2(x,y) \
  nbok += ( x ) ? 1 : 0; \
  nb++; \
  trace.info() << "(" << nbok << "/" << nb << ") " \
  << y << std::endl;



struct MyDomainStyleCustomRed : public DrawableWithBoard2D
{
  virtual void setStyle(Board2D & aboard) const
  {
    aboard.setFillColorRGBi(255, 0, 0);
    aboard.setPenColorRGBi(0, 255, 0);
  }
};


bool testDigitalSetBoardSnippet()
{
  typedef SpaceND<2> Z2;
  typedef HyperRectDomain<Z2> Domain;
  typedef Z2::Point Point;
  Point p1(  -10, -10  );
  Point p2(  10, 10  );
  Domain domain( p1, p2 );
  typedef DigitalSetSelector < Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type SpecificSet;

  BOOST_CONCEPT_ASSERT(( CDigitalSet< SpecificSet > ));

  SpecificSet mySet( domain );

  Point c(  0, 0  );
  mySet.insert( c );
  Point d(  5, 2  );
  mySet.insert( d );
  Point e(  1, -3  );
  mySet.insert( e );

  Board2D board;
  board.setUnit(LibBoard::Board::UCentimeter);
  board << mySet;
  board.saveSVG("myset-export.svg");

  board.clear();

  board.setUnit(LibBoard::Board::UCentimeter);
  board << SetMode( domain.className(), "Grid" ) << domain << mySet;
  board.saveSVG("simpleSet-grid.svg");

  board.clear();

  board.setUnit(LibBoard::Board::UCentimeter);
  board << SetMode( domain.className(), "Paving" ) << domain;
  board << mySet;
  board.saveSVG("simpleSet-paving.svg");


  board.clear();

  board.setUnit(LibBoard::Board::UCentimeter);
  board << CustomStyle( mySet.className(), new MyDomainStyleCustomRed );
  board << mySet;
  board.saveSVG("simpleSet-color.svg");

  return true;
}

template < typename DigitalSetType >
bool testDigitalSet( const typename DigitalSetType::Domain & domain )
{
  BOOST_CONCEPT_ASSERT(( CDigitalSet< DigitalSetType > ));

  typedef typename DigitalSetType::Domain Domain;
  typedef typename Domain::Point Point;
  typedef typename Point::Coordinate Coordinate;
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Constructor." );

  DigitalSetType set1( domain );
  nbok += set1.size() == 0 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  << "Empty set: " << set1 << std::endl;
  trace.endBlock();

  Coordinate t [] = { 4, 3, 3 , 4};
  Point p1( t );
  Coordinate t2[] = { 2, 5, 3 , 5};
  Point p2( t2);
  Coordinate t3[] =  { 2, 5, 3 , 4} ;
  Point p3( t3);

  trace.beginBlock ( "Insertion." );
  set1.insert( p1 );
  set1.insert( p2 );
  set1.insert( p3 );
  set1.insert( p2 );
  nbok += set1.size() == 3 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  << "Set (3 elements): " << set1 << std::endl;
  trace.endBlock();

  return nbok == nb;
}

template < typename DigitalDomain, int props >
bool testDigitalSetSelector( const DigitalDomain & domain,
    const std::string & comment )
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Test DigitalSetSelector( " + comment + ")." );

  typedef typename DigitalSetSelector
  < DigitalDomain, props >::Type SpecificSet;
  SpecificSet set1( domain );
  set1.insert( domain.lowerBound() );
  set1.insert( domain.upperBound() );
  nbok += set1.size() == 2 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
  << comment << " (2 elements): " << set1 << std::endl;

  trace.endBlock();


  return nbok == nb;
}

bool testDigitalSetDraw()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef SpaceND<2> Z2;
  typedef HyperRectDomain<Z2> Domain;
  typedef Z2::Point Point;
  Point p1(  -10, -10  );
  Point p2(  10, 10  );
  Domain domain( p1, p2 );
  typedef DigitalSetSelector
  < Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type SpecificSet;

  BOOST_CONCEPT_ASSERT(( CDigitalSet< SpecificSet > ));
  SpecificSet disk( domain );
  Point c(  0, 0  );

  trace.beginBlock ( "Creating disk( r=5.0 ) ..." );
  for ( Domain::ConstIterator it = domain.begin();
  it != domain.end();
      ++it )
  {
    if ( (*it - c ).norm() < 5.0 )
      // insertNew is very important for vector container.
      disk.insertNew( *it );
  }

  //Board export test
  trace.beginBlock("SVG Export");
  Board2D board;
  board << SetMode( domain.className(), "Grid" ) << domain;
  board << disk;

  board.scale(10);
  board.saveSVG( "disk-set.svg" );
  trace.endBlock();

  return nbok == nb;
}

bool testDigitalSetDomain()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef SpaceND<2> Z2;
  typedef HyperRectDomain<Z2> Domain;
  typedef Z2::Point Point;
  Point p1(  -49, -49  );
  Point p2(  49, 49  );
  Domain domain( p1, p2 );
  typedef DigitalSetSelector
  < Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type SpecificSet;
  BOOST_CONCEPT_ASSERT(( CDigitalSet< SpecificSet > ));

  SpecificSet disk( domain );
  Point c(  0, 0  );
  Point l(  49, 0  );

  trace.beginBlock ( "Creating disk( r=50.0 ) ..." );
  for ( Domain::ConstIterator it = domain.begin(); 
  it != domain.end();
  ++it )
    {
      if ( (*it - c ).norm() < 50.0 )
  // insertNew is very important for vector container.
  disk.insertNew( *it );
    }
  disk.erase( c );
  INBLOCK_TEST( disk.size() == 7824 );
  trace.info() << "disk.size()=" << disk.size() << std::endl;
  trace.endBlock();

  typedef DigitalSetDomain< SpecificSet > RestrictedDomain;
  RestrictedDomain disk_domain( disk );
  trace.beginBlock ( "Iterating over disk domain ..." );
  unsigned int nb_in_domain = 0;
  for ( RestrictedDomain::ConstIterator it = disk_domain.begin();
      it != disk_domain.end();
      ++it )
  {
    ++nb_in_domain;
  }
  INBLOCK_TEST( nb_in_domain == 7824 );
  INBLOCK_TEST( disk_domain.lowerBound() == Point(  -49, -49 ) );
  INBLOCK_TEST( disk_domain.upperBound() == Point(   49,  49 ) );
  trace.endBlock();

  return nbok == nb;
}

bool testDigitalSetConcept()
{
  typedef Z2i::Point Value;
  typedef std::vector<Value>::iterator vector_iterator;
  typedef std::set<Value>::iterator set_iterator;
  //BOOST_CONCEPT_ASSERT(( boost::Mutable_BidirectionalIterator< vector_iterator > ));
  //BOOST_CONCEPT_ASSERT(( boost::Mutable_BidirectionalIterator< set_iterator > ));
  BOOST_CONCEPT_ASSERT(( CDigitalSet<Z2i::DigitalSet> ));
  BOOST_CONCEPT_ASSERT(( CDigitalSet<Z3i::DigitalSet> ));

  typedef Z2i::Space Space;
  BOOST_CONCEPT_ASSERT(( CDomain< CDomainArchetype< Space > > ));
  typedef CDigitalSetArchetype<Z2i::Domain> DigitalSetArchetype;
  BOOST_CONCEPT_ASSERT(( CDigitalSet<DigitalSetArchetype> ));
  
  return true;
}

int main()
{
  typedef SpaceND<4> Space4Type;
  typedef HyperRectDomain<Space4Type> Domain;
  typedef Space4Type::Point Point;

  DGtal::int32_t t[] =  { 1, 2, 3 , 4};
  Point a ( t );
  DGtal::int32_t t2[] = { 5, 5, 3 , 5};
  Point b ( t2);
  trace.beginBlock ( "HyperRectDomain init" );

  ///Domain characterized by points a and b
  Domain domain ( a, b );
  trace.info() << domain << std::endl;
  trace.info() << "Domain size= " << domain.size() << std::endl;
  trace.endBlock();

  trace.beginBlock( "DigitalSetBySTLVector" );
  bool okVector = testDigitalSet< DigitalSetBySTLVector<Domain> >( domain );
  trace.endBlock();

  trace.beginBlock( "DigitalSetBySTLSet" );
  bool okSet = testDigitalSet< DigitalSetBySTLSet<Domain> >( domain );
  trace.endBlock();

  bool okSelectorSmall = testDigitalSetSelector
      < Domain, SMALL_DS + LOW_VAR_DS + LOW_ITER_DS + LOW_BEL_DS >
      ( domain, "Small set" );

  bool okSelectorBig = testDigitalSetSelector
      < Domain, BIG_DS + LOW_VAR_DS + LOW_ITER_DS + LOW_BEL_DS >
      ( domain, "Big set" );

  bool okSelectorMediumHBel = testDigitalSetSelector
      < Domain, MEDIUM_DS + LOW_VAR_DS + LOW_ITER_DS + HIGH_BEL_DS >
      ( domain, "Medium set + High belonging test" );

  bool okDigitalSetDomain = testDigitalSetDomain();

  bool okDigitalSetDraw = testDigitalSetDraw();

  bool okDigitalSetDrawSnippet = testDigitalSetBoardSnippet();

  bool res = okVector && okSet
      && okSelectorSmall && okSelectorBig && okSelectorMediumHBel
      && okDigitalSetDomain && okDigitalSetDraw && okDigitalSetDrawSnippet;
  trace.endBlock();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  return res ? 0 : 1;
}

/** @ingroup Tests **/
