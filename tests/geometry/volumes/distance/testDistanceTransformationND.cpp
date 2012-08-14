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
 * @file testDistanceTransformND.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/12/24
 *
 * Functions for testing class DistanceTransformND.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/geometry/volumes/distance/SeparableMetricHelper.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/images/imagesSetsUtils/SimpleThresholdForegroundPredicate.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DistanceTransformND.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDistanceTransformND()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing dT dim=5 ..." );
  
  typedef SpaceND<5> TSpace;
  typedef TSpace::Point Point;
  typedef HyperRectDomain<TSpace> Domain;
  typedef HueShadeColorMap<unsigned char, 2> HueTwice;
  typedef GrayscaleColorMap<unsigned char> Gray;
  int t[5] = {0,0,0,0,0};
  Point a ( t );
  int t2[5] = {15,15,15,15,15};
  Point b ( t2 );
  int t3[5] = {3,3,3,3,3};
  Point c ( t3 );
  Point d;

  typedef ImageSelector<Domain, unsigned int>::Type Image;
  Domain domain(a,b);
  Image image(domain);

  //We create an object image with a signle background point (set to 0)
  for (Image::Iterator it=image.begin(),itend=image.end(); it!=itend; ++it)
    *it = 128;
  image.setValue( c , 0 );

  typedef SimpleThresholdForegroundPredicate<Image> Predicate;
  Predicate aPredicate(image,0);

  DistanceTransformation<TSpace,Predicate,2> dt(domain,aPredicate);
  typedef  DistanceTransformation<TSpace,Predicate, 2>::OutputImage ImageLong;

  dt.checkTypesValidity (  );

  //Distance transformation computation
  ImageLong result = dt.compute (  );
  
  //We check the result
  bool res=true;
  for(Domain::ConstIterator itDom = domain.begin(), itDomend = domain.end();
      itDom != itDomend; ++itDom)
    {
      //distance from the point to the seed
      d = (*itDom) - c;
      ImageLong::Value norm2=0;
      for(Point::Iterator itd=d.begin(), itdend=d.end(); itd!=itdend; ++itd)
	norm2+= (*itd)*(*itd);

       if ( result( (*itDom) ) != norm2)
  {
    trace.error()<<"Error at "<<(*itDom)
           << ": expected="<<norm2<<" and computed="<<result(*itDom)<<endl;
  res=false;
  }
    }
  nbok += res ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DistanceTransformND" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDistanceTransformND(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
