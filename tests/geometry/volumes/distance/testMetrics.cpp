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
 * @file testMetrics.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/29
 *
 * Functions for testing class Metrics.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/volumes/distance/SeparableMetricHelper.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Metrics.
///////////////////////////////////////////////////////////////////////////////
bool testMetrics()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing separable metrics ..." );

  Z2i::Point a( 0,0), b(5, 0), bb(5,-10), bbb(5,5),c(10,0);
  Z2i::Point starting( 0, 5), endpoint(10,5);
  
  SeparableMetricHelper<Z2i::Point, DGtal::uint64_t, 3> metric;

  trace.info()<< "a= "<<a<<std::endl;
  trace.info()<< "b= "<<b<<std::endl;
  trace.info()<< "bb= "<<bb<<std::endl;
  trace.info()<< "bbb= "<<bbb<<std::endl;
  trace.info()<< "c= "<<c<<std::endl;


  bool hidden  =metric.hiddenBy(a,b,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,b,c) returns false" << std::endl;
      
  hidden  =metric.hiddenBy(a,bb,c,starting,endpoint,0); 
  nbok += (hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bb,c) returns true" << std::endl;
  
  hidden  =metric.hiddenBy(a,bbb,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bbb,c) returns false" << std::endl;
  
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class Metrics" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testMetrics(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
