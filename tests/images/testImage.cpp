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
 * @file test_Image.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */



#include <cstdio>
#include <cmath>
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"


using namespace DGtal;
using namespace std;


/**
 * Simple test of Image construction.
 *
 **/
bool testSimpleImage()
{
  typedef DGtal::int64_t Integer;
  typedef SpaceND<4, Integer > Space4Type;
  typedef HyperRectDomain<Space4Type> Domain;
  typedef Domain::Point Point;

  //Default image selector = STLVector
  typedef ImageContainerBySTLVector<Domain, int> Image;

  const Integer t[ ] = { 1, 2, 3 ,4};
  const Integer t2[ ] = { 5, 5, 3 ,4};
  const Integer t3[ ] = { 2, 2, 3 ,4};
  Point a ( t );
  Point b ( t2 );
  Point c ( t3 );

  trace.beginBlock ( "Image init" );

  ///Domain characterized by points a and b
  Image myImage ( Domain( a,b ));
  trace.info() << myImage << std::endl;

  trace.endBlock();

  myImage.setValue( c, 128 );

  trace.beginBlock("Test of built-in iterators");
  for ( Image::ConstIterator it = myImage.begin();
	it != myImage.end();
	++it)
    trace.info() << (*it) <<" ";
 
  trace.info()<<std::endl;
  trace.endBlock();
  
  return myImage.isValid();
}



/**
 * Simple test of Image construction.
 *
 **/
template<typename Image>
bool testImageContainer()
{
  typedef typename Image::Domain Domain;
  typedef typename Image::Value Value;
  typedef typename Domain::Coordinate Coordinate;
  typedef typename Domain::Point Point;
  
  const Coordinate t[ ] = { 1, 2, 3 ,4};
  const Coordinate t2[ ] = { 5, 5, 3 ,4};
  const Coordinate t3[ ] = { 2, 2, 3 ,4};
  Point a ( t );
  Point b ( t2 );
  Point c ( t3 );

  int nbPerfect=0;
  int nbok = 0;


  trace.beginBlock ( "Image API test" );

  ///Domain characterized by points a and b
  Image myImage ( Domain( a,b ));
  trace.info() << myImage << std::endl;
  trace.info() << myImage.domain() << std::endl;

  trace.endBlock();


  myImage.setValue( c, 128 );
  typename Image::Iterator it = myImage.begin();
  it ++;
  nbok += ((*it) == 128);
  nbPerfect++;

  //getIterator
  typename Image::Iterator itc = myImage.getIterator(c);
  trace.info() << "Checking point "<<c<<".. got: "<<(*itc)<< " expected = 128"<<std::endl;
  nbok += ((*itc) == 128);
  nbPerfect++;
  
  //Pointer arithmetic
  it += 3;
  (*it) = 64;
  nbok += ((*it) == 64);
  nbPerfect++;

  trace.beginBlock("Test of built-in iterators");
  for ( typename Image::ConstIterator itconst = myImage.begin();
	itconst != myImage.end();
	++itconst)
    trace.info() << (*itconst) <<" ";
 
  trace.info()<<std::endl;
  trace.endBlock();
  
  trace.info() << "["<<nbok<<"/"<<nbPerfect<<"]"<< std::endl;
  return myImage.domain().isValid() && myImage.isValid() && (nbok == nbPerfect);
}

int main()
{
 
  typedef DGtal::int64_t Integer;
  typedef SpaceND<4, Integer > Space4Type;
  typedef HyperRectDomain<Space4Type> Domain;
 
  trace.beginBlock("Image Tests");
  int ok;
  
  if ( testSimpleImage() && testImageContainer<ImageContainerBySTLVector<Domain, DGtal::uint64_t> >() )
    ok = 0;
  else
    ok = 1;

  trace.endBlock();
  return ok;


}

