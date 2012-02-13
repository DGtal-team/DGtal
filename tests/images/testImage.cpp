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
#include "DGtal/images/CImage.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerBySTLMap.h"


using namespace DGtal;
using namespace std;


// /**
//  * Simple test of Image construction.
//  *
//  **/
// bool testSimpleImage()
// {
//   typedef DGtal::int64_t Integer;
//   typedef SpaceND<4, Integer > Space4Type;
//   typedef HyperRectDomain<Space4Type> Domain;
//   typedef Domain::Point Point;

//   //Default image selector = STLVector
//   typedef ImageContainerBySTLVector<Domain, int> Image;

//   const Integer t[ ] = { 1, 2, 3 ,4};
//   const Integer t2[ ] = { 5, 5, 3 ,4};
//   const Integer t3[ ] = { 2, 2, 3 ,4};
//   Point a ( t );
//   Point b ( t2 );
//   Point c ( t3 );

//   trace.beginBlock ( "Image init" );

//   ///Domain characterized by points a and b
//   Image myImage ( Domain( a,b ));
//   trace.info() << myImage << std::endl;

//   trace.endBlock();

//   myImage.setValue( c, 128 );

//   trace.beginBlock("Test of built-in iterators");
//   for ( Image::ConstIterator it = myImage.begin();
// 	it != myImage.end();
// 	++it)
//     trace.info() << (*it) <<" ";
 
//   trace.info()<<std::endl;
//   trace.endBlock();
  
//   return myImage.isValid();
// }



/**
 * Simple test of Image construction.
 *
 **/
template<typename Image>
bool testImage(const typename Image::Domain& d)
{

  BOOST_CONCEPT_ASSERT(( CImage<Image> )); 

  int nb = 0;
  int nbok = 0;

  ////////////////////////////////////////////////
  trace.beginBlock ( "Main services, range" );

  Image img(d); //ctor
  Image img2 = img; //copy

  //fill
  typename Image::Domain::ConstIterator dit = img.domain().begin(); 
  typename Image::Domain::ConstIterator ditEnd = img.domain().end(); 
  for (int i = 0; dit != ditEnd; ++dit, ++i)
    {
      img.setValue(*dit, i);
      img2.setValue(*dit, 0); 
    }
  Image img3(d); 
  img3 = img; //assign

  //ranges comparison
  typename Image::ConstRange rimg = img.range(); 
  typename Image::ConstRange rimg2 = img2.range(); 
  typename Image::ConstRange rimg3 = img3.range(); 

  bool flag2 = std::equal(rimg.begin(), rimg.end(), rimg2.begin()); 
  bool flag3 = std::equal(rimg.begin(), rimg.end(), rimg3.begin()); 

  nbok += (!flag2 && flag3)?1:0;
  nb++;  
  trace.info() << "(" <<nbok << "/" << nb << ")" << std::endl;
  trace.endBlock();

  ////////////////////////////////////////////////
  trace.beginBlock ( "Output iterator" );
  std::copy(rimg.begin(), rimg.end(), img2.output()); 

  flag2 = std::equal(rimg.begin(), rimg.end(), rimg2.begin()); 
  nbok += (flag2)?1:0;
  nb++;  
  trace.info() << "(" <<nbok << "/" << nb << ")" << std::endl;
  trace.endBlock();

  ////////////////////////////////////////////////
  trace.beginBlock ( " Getters / setters " );
  typename Image::Domain::Point p = img.domain().upperBound(); 
  img.setValue( p, 128 );
  bool flag4 = ( img(p) == 128 );
  std::copy( rimg.begin(), rimg.end(), std::ostream_iterator<int>(cout,", ") ); 
  cout << endl;  
  flag2 = std::equal(rimg.begin(), rimg.end(), rimg2.begin()); 
  std::copy( rimg2.begin(), rimg2.end(), std::ostream_iterator<int>(cout,", ") ); 
  cout << endl;  
  flag3 = std::equal(rimg.begin(), rimg.end(), rimg3.begin()); 
  std::copy( rimg3.begin(), rimg3.end(), std::ostream_iterator<int>(cout,", ") ); 
  cout << endl;  
  nbok += ( flag4 && (!flag2) && (!flag3) )?1:0;
  nb++;  
  trace.info() << "(" <<nbok << "/" << nb << ")" << std::endl;
  trace.endBlock();

  ////////////////////////////////////////////////
  trace.beginBlock ( " Display " );
  trace.info() << img << std::endl; 
  trace.info() << img2 << std::endl; 
  trace.info() << img3 << std::endl; 
  trace.endBlock();

  return ( img.isValid() && img2.isValid() && img3.isValid() && (nbok == nb) );
}

int main( int argc, char** argv )
{
 
  trace.beginBlock ( "Testing image classes" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  /// domain
  typedef DGtal::int64_t Integer; 
  typedef SpaceND<2,Integer> Space; 
  typedef Space::Point Point; 
  typedef HyperRectDomain<Space> Domain; 

  const Integer size = 5; 
  Point p = Point::diagonal(0); 
  Point q = Point::diagonal(size-1); 
  Domain d(p,q); 

  /// images
  typedef short Value; 
  typedef ImageContainerBySTLVector<Domain,Value> VImage; 
  typedef ImageContainerBySTLMap<Domain,Value> MImage; 

  /// tests
  bool res = testImage<VImage>(d); 
  res = res && testImage<MImage>(d);

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}

