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
 * @file testHyperRectDomain.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * @author Guillaume Damiand
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testHyperRectDomain <p>
 * Aim: simple test of \ref HyperRectDomain
 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/base/CConstBidirectionalRange.h"

using namespace DGtal;
using namespace std;


/**
* Simple test of HyperRectDomain construction.
*
**/
bool testSimpleHyperRectDomain()
{

  typedef SpaceND<4> Space4Type;
  typedef Space4Type::Point Point;

  DGtal::int32_t t [] = { 1, 2, 3 , 4};
  Point a ( t );
  DGtal::int32_t t2[] = { 5, 5, 3 , 4};
  Point b ( t2 );

  trace.beginBlock ( "HyperRectDomain init" );
  // Checking that HyperRectDomain is a model of CDomain.
  typedef HyperRectDomain<Space4Type> HRDomain4;
  BOOST_CONCEPT_ASSERT(( CDomain< HRDomain4 > ));
  BOOST_CONCEPT_ASSERT(( CConstBidirectionalRange<HRDomain4> ));
      
  ///Empty domain using the default constructor
  HyperRectDomain<Space4Type> myEmptyDomain;
  trace.info() << "Empty Domain: " << myEmptyDomain << std::endl;

  ///Domain characterized by points a and b
  HyperRectDomain<Space4Type> myHyperRectDomain ( a, b );
  trace.info() << myHyperRectDomain << std::endl;

  trace.info() << "Domain Size= " << myHyperRectDomain.size() << std::endl;


  trace.endBlock();


  trace.beginBlock("Test Copy Constructor");
  HyperRectDomain<Space4Type> myHyperRectDomainBis( myHyperRectDomain );
  trace.info() << "Domain Size= " << myHyperRectDomainBis.size() << std::endl;
  trace.endBlock();

  trace.beginBlock("Test Assignement");
  HyperRectDomain<Space4Type> myHyperRectDomainTer;

  myHyperRectDomainTer = myHyperRectDomain;

  trace.info() << "Domain Size= " << myHyperRectDomainTer.size() << std::endl;
  trace.endBlock();

  return myHyperRectDomain.isValid();

}

bool testIterator()
{
  typedef SpaceND<2> TSpace;
  typedef TSpace::Point Point;
  Point a ( 1, 1);
  Point b ( 5, 5);
  Point c (2, 2);

  trace.beginBlock ( "HyperRectDomain Iterator" );
  HyperRectDomain<TSpace> myHyperRectDomain ( a, b );

  trace.info() << myHyperRectDomain << std::endl;

  trace.emphase() << "Iterator 2d: ";
  for ( HyperRectDomain<TSpace>::ConstIterator it = myHyperRectDomain.begin();
  it != myHyperRectDomain.end(); ++it )
    trace.warning() << ( *it ) << std::endl;

  trace.emphase() << "Reverse Iterator 2d: ";
  for ( HyperRectDomain<TSpace>::ConstReverseIterator it = myHyperRectDomain.rbegin(),
    itend = myHyperRectDomain.rend(); it != itend; ++it )
    trace.warning() << ( *it ) << std::endl;
  
#ifdef CPP11_INITIALIZER_LIST
  trace.emphase() << "Iterator 2d (permutation initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstIterator 
    it = myHyperRectDomain.subRange( {1, 0} ).begin();
  it != myHyperRectDomain.subRange( {1, 0} ).end(); ++it )
    trace.warning() << ( *it ) << std::endl;
  trace.emphase() << "Reverse Iterator 2d (permutation initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstReverseIterator 
    it = myHyperRectDomain.subRange( {1, 0} ).rbegin(),
    itend=myHyperRectDomain.subRange( {1, 0} ).rend(); it!=itend;  ++it )
    trace.warning() << ( *it ) << std::endl;

  trace.emphase() << "Iterator 2d (permutation+starting initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstIterator
    it = myHyperRectDomain.subRange( {1, 0} ).begin(c);
  it != myHyperRectDomain.subRange( {1, 0} ).end(); ++it )
    trace.warning() << ( *it ) << std::endl;
  trace.emphase() << "Reverse Iterator 2d (permutation+starting initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstReverseIterator 
    it = myHyperRectDomain.subRange( {1, 0} ).rbegin(c),
    itend=myHyperRectDomain.subRange( {1, 0} ).rend(); it !=itend ; ++it )
    trace.warning() << ( *it ) << std::endl;

  trace.emphase() << "Iterator 2d (span initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstIterator
    it = myHyperRectDomain.subRange( {1} ).begin();
  it != myHyperRectDomain.subRange( {1} ).end(); ++it )
    trace.warning() << ( *it ) << std::endl;
  trace.emphase() << "Reverse Iterator 2d (span initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstReverseIterator 
    it = myHyperRectDomain.subRange( {1} ).rbegin(),
    itend=myHyperRectDomain.subRange( {1} ).rend(); it != itend; ++it )
    trace.warning() << ( *it ) << std::endl;

  trace.emphase() << "Iterator 2d (span+starting initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstIterator
    it = myHyperRectDomain.subRange( {1} , c ).begin(c);
  it != myHyperRectDomain.subRange( {1} , c ).end(); ++it )
    trace.warning() << ( *it ) << std::endl;
  trace.emphase() << "Reverse Iterator 2d (span+starting initializer list): ";
  for ( HyperRectDomain<TSpace>::ConstSubRange::ConstReverseIterator 
    it = myHyperRectDomain.subRange( {1} , c ).rbegin(c),
    itend=myHyperRectDomain.subRange( {1} , c ).rend(); it !=itend; ++it )
    trace.warning() << ( *it ) << std::endl;
#endif

  trace.emphase() << "Iterator 4d: ";
  typedef SpaceND<4> TSpace4D;
  typedef TSpace4D::Point Point4D;

  DGtal::int32_t t[] = {1, 1, 1, 1};
  Point4D a4D ( t );
  DGtal::int32_t t2[] = {3, 3, 3, 3};
  Point4D b4D ( t2 );

  HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D, b4D );
  trace.emphase() << myHyperRectDomain4D << std::endl;
  for ( HyperRectDomain<TSpace4D>::ConstIterator
    it = myHyperRectDomain4D.begin();
      it != myHyperRectDomain4D.end(); ++it )
    trace.info() << ( *it ) << std::endl;

  trace.emphase() << "Reverse Iterator 4d: ";
  for ( HyperRectDomain<TSpace4D>::ConstReverseIterator
    it = myHyperRectDomain4D.rbegin(),
    itend=myHyperRectDomain4D.rend(); it != itend; ++it )
    trace.info() << ( *it ) << std::endl;

  
#ifdef CPP11_INITIALIZER_LIST
  trace.emphase() << "Iterator 4d by using order different from lexicographic initializer list: ";
  for ( HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator
    it = myHyperRectDomain4D.subRange( {3, 2, 1, 0}).begin();
  it != myHyperRectDomain4D.subRange( {3, 2, 1, 0}).end(); ++it )
    trace.info() << ( *it ) << std::endl;

  trace.emphase() << "Decreasing Iterator 4d by using order different from lexicographic initializer list: ";
  HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator
    it1 = myHyperRectDomain4D.subRange( {3, 2, 1, 0}).begin();
  HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator
    it2 = myHyperRectDomain4D.subRange( {3, 2, 1, 0}).end();
  --it1;
  --it2;
  for ( ; it1 != it2; --it2 )
    trace.info() << ( *it2 ) << std::endl;

  trace.emphase() << "Iterator on a subset of 4d by using order different from lexicographic initializer list: ";
  for ( HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator
    it3 = myHyperRectDomain4D.subRange( {1, 3}).begin();
  it3 != myHyperRectDomain4D.subRange( {1, 3}).end(); ++it3 )
    trace.info() << ( *it3 ) << std::endl;

  trace.emphase() << "Decreasing iterator on a subset of 4d by using order different from lexicographic initializer list: ";
  HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator 
    it4 = myHyperRectDomain4D.subRange({1, 3}).begin();
  HyperRectDomain<TSpace4D>::ConstSubRange::ConstIterator
    it5 = myHyperRectDomain4D.subRange({1, 3}).end();
  --it4;
  --it5;
  for ( ; it4 != it5; --it5 )
    trace.info() << ( *it5 ) << std::endl;
#endif

  return myHyperRectDomain.isValid();
}


bool testReverseIterator()
{
  typedef SpaceND<4> TSpace4D;
  typedef TSpace4D::Point Point4D;
  DGtal::int32_t t[] = {1, 1, 1, 1};
  Point4D a4D (t);
  DGtal::int32_t t2[] = {3, 3, 3, 3};
  Point4D b4D (t2);

  trace.beginBlock ( "Test reverse iterator" );

  HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D, b4D );
  trace.emphase() << myHyperRectDomain4D << std::endl;

  trace.emphase() << "Increasing order: ";

  HyperRectDomain<TSpace4D>::ConstIterator 
    it = myHyperRectDomain4D.begin();
  for ( ; it != myHyperRectDomain4D.end(); ++it )
    trace.info() << ( *it ) << std::endl;

  trace.emphase() << "Now decreasing order: ";
  HyperRectDomain<TSpace4D>::ConstIterator 
    it2 = myHyperRectDomain4D.begin();
  --it;
  --it2;
  for ( ; it != it2; --it )
    trace.info() << ( *it ) << std::endl;

  trace.endBlock();

  return myHyperRectDomain4D.isValid();
}



bool testSTLCompat()
{
  typedef SpaceND<4> TSpace4D;
  typedef TSpace4D::Point Point4D;
  DGtal::int32_t t[] = {1, 1, 1, 1};
  Point4D a4D (t);
  DGtal::int32_t t2[] = {3, 3, 3, 3};
  Point4D b4D (t2);

  trace.beginBlock ( "TestSTL Compatibility" );

  HyperRectDomain<TSpace4D> myHyperRectDomain4D ( a4D, b4D );
  trace.emphase() << myHyperRectDomain4D << std::endl;

  std::copy ( myHyperRectDomain4D.begin(),
      myHyperRectDomain4D.end(),
      ostream_iterator<Point4D> ( trace.info(), " " ) );

  trace.info() << std::endl;
  trace.endBlock();

  return myHyperRectDomain4D.isValid();
}


int main()
{
  if ( testSimpleHyperRectDomain() && testIterator() && testReverseIterator() && testSTLCompat() )
    return 0;
  else
    return 1;
}

/** @ingroup Tests **/
