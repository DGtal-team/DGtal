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
 * @file testPowerMap.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/10/25
 *
 * Functions for testing class PowerMap.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/volumes/distance/PowerMap.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpPowerSeparableMetric.h"
#include "DGtal/kernel/sets/DigitalSetDomain.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PowerMap.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testPowerMap()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing PowerMap2D ..." );

  Z2i::Domain domain(Z2i::Point(0,0),Z2i::Point(10,10));
  Z2i::Domain domainLarge(Z2i::Point(0,0),Z2i::Point(10,10));

  DigitalSetBySTLSet<Z2i::Domain > set(domain);
  set.insertNew(Z2i::Point(3,3)); 
  //set.insertNew(Z2i::Point(3,7)); 
  set.insertNew(Z2i::Point(7,7));
  DigitalSetDomain< DigitalSetBySTLSet<Z2i::Domain > > setDomain(set); 
  
  typedef ImageContainerBySTLMap< DigitalSetDomain< DigitalSetBySTLSet<Z2i::Domain > > , DGtal::int64_t> Image;
  Image image(setDomain);
  
  //Setting some values
  image.setValue(Z2i::Point(3,3), 9); 
  //  image.setValue(Z2i::Point(3,7), 0); 
  image.setValue(Z2i::Point(7,7), 16);
  
  Z2i::L2PowerMetric l2power;
  PowerMap<Image, Z2i::L2PowerMetric> power(&domainLarge, &image, &l2power);
  for(unsigned int i=0; i<11; i++)
    {
      for(unsigned int j=0; j<11; j++)
	if (image.domain().isInside(Z2i::Point(i,j)))
	  trace.info()<< image(Z2i::Point(i,j))<<" ";
	else
	  trace.info()<< "0 ";
      trace.info()<<std::endl;
    }
  trace.info()<<std::endl;

  //Power Map
  for(unsigned int i=0; i<11; i++)
    {
      for(unsigned int j=0; j<11; j++)
	trace.info()<< power(Z2i::Point(i,j))[0]<<","<<power(Z2i::Point(i,j))[1]<<" ";
      trace.info()<<std::endl;
    }
  trace.info() << "REDT"<<std::endl;
  trace.info()<<std::endl;
  //Reconstruction
  for(unsigned int i=0; i<11; i++)
    {
      for(unsigned int j=0; j<11; j++)
	{
	  Z2i::Point p(i,j);
	  DGtal::int32_t dist = (i-power(p)[0])*(i-power(p)[0]) +
	    ( j-power(p)[1])*(j-power(p)[1])  - image(power(p));
	  trace.info()<< dist;
	}
      std::cerr<<std::endl;
    }
  trace.info()<<std::endl;
 
  //Reconstruction
  for(unsigned int i=0; i<11; i++)
    {
      for(unsigned int j=0; j<11; j++)
	{
	  Z2i::Point p(i,j);
	  DGtal::int32_t dist = (i-power(p)[0])*(i-power(p)[0]) +
	    ( j-power(p)[1])*(j-power(p)[1])  - image(power(p));
	  if (dist>=0)
	    std::cerr<< "0 ";
	   else
	     std::cerr<< "X ";
	}
      std::cerr<<std::endl;
    }
  
  nbok += true ? 1 : 0; 
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
  trace.beginBlock ( "Testing class PowerMap" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testPowerMap(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
