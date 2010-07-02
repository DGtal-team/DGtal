/**
 * @file test_HyperRectDomain.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
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

#include "DGtal/base/Common.h"
#include "DGtal/kernel/Space.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"


using namespace DGtal;
using namespace std;


/**
* Test of DigitalSetBySTLVector.
*
**/
bool testDigitalSetBySTLVector()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef Space<int,4> Space4Type;
  typedef HyperRectDomain<Space4Type> DomainType;
  typedef Space4Type::Point Point;
  
  Point a ( { 1, 2, 3 ,4} );
  Point b ( { 5, 5, 3 ,5} );
  Point p1( { 4, 3, 3 ,4} );
  Point p2( { 2, 5, 3 ,5} );
  Point p3( { 2, 5, 3 ,4} );
  trace.beginBlock ( "HyperRectDomain init" );

  ///Domain characterized by points a and b
  DomainType domain ( a,b );
  trace.info() << domain << std::endl;
  trace.info() << "Domain Extent= "<< domain.extent() << std::endl;
  trace.endBlock();
 
  trace.beginBlock ( "DigitalSetBySTLVector constructor." );
  DigitalSetBySTLVector<DomainType> set1( domain );
  nbok += set1.size() == 0 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Empty set: " << set1 << std::endl;
  trace.endBlock();

  trace.beginBlock ( "DigitalSetBySTLVector insert." );
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

/**
* Test of DigitalSetBySTLSet.
*
**/
// bool testDigitalSetBySTLSet()
// {
//   unsigned int nbok = 0;
//   unsigned int nb = 0;
  
//   typedef Space<int,4> Space4Type;
//   typedef HyperRectDomain<Space4Type> DomainType;
//   typedef Space4Type::Point Point;
  
//   Point a ( { 1, 2, 3 ,4} );
//   Point b ( { 5, 5, 3 ,5} );
//   Point p1( { 4, 3, 3 ,4} );
//   Point p2( { 2, 5, 3 ,5} );
//   Point p3( { 2, 5, 3 ,4} );
//   trace.beginBlock ( "HyperRectDomain init" );

//   ///Domain characterized by points a and b
//   DomainType domain ( a,b );
//   trace.info() << domain << std::endl;
//   trace.info() << "Domain Extent= "<< domain.extent() << std::endl;
//   trace.endBlock();
 
//   trace.beginBlock ( "DigitalSetBySTLVector constructor." );
//   DigitalSetBySTLSet<DomainType> set1( domain );
//   nbok += set1.size() == 0 ? 1 : 0; 
//   nb++;
//   trace.info() << "(" << nbok << "/" << nb << ") "
// 	       << "Empty set: " << set1 << std::endl;
//   trace.endBlock();

//   trace.beginBlock ( "DigitalSetBySTLSet insert." );
//   set1.insert( p1 );
//   set1.insert( p2 );
//   set1.insert( p3 );
//   set1.insert( p2 );
//   nbok += set1.size() == 3 ? 1 : 0; 
//   nb++;
//   trace.info() << "(" << nbok << "/" << nb << ") "
// 	       << "Set (3 elements): " << set1 << std::endl;
//   trace.endBlock();

//   return nbok == nb;
// }

int main()
{
  if ( testDigitalSetBySTLVector()
       /* && testDigitalSetBySTLVector() */ )
    return 0;
  else
    return 1;
}

