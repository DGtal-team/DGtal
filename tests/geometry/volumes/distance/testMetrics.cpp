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

#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpWeightedSeparableMetric.h"
#include "DGtal/geometry/volumes/distance/InexactPredicateLpSeparableMetric.h"
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
  
  trace.beginBlock ( "Testing separable metrics l_2 ..." );

  Z2i::Point a( 0,0), b(5, 0), bb(5,-10), bbb(5,5),c(10,0), d(3,3);
  Z2i::Point starting( 0, 5), endpoint(10,5);
  
  ExactPredicateLpSeparableMetric<Z2i::Space, 2> metric;

  trace.info()<< "a= "<<a<<std::endl;
  trace.info()<< "b= "<<b<<std::endl;
  trace.info()<< "bb= "<<bb<<std::endl;
  trace.info()<< "bbb= "<<bbb<<std::endl;
  trace.info()<< "c= "<<c<<std::endl;

  trace.info() << "distance between a and bb = "<< metric.distance(a,bb)<< std::endl;


  DGtal::Closest closest  =metric.closest(a,d,c);
  nbok += (closest == ClosestFIRST) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "closest(a,d,c) returns d" << std::endl;
      
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

  trace.beginBlock ( "Testing separable metrics l_3 ..." );

   
  ExactPredicateLpSeparableMetric<Z2i::Space, 3> metric3;

  trace.info()<< "a= "<<a<<std::endl;
  trace.info()<< "b= "<<b<<std::endl;
  trace.info()<< "bb= "<<bb<<std::endl;
  trace.info()<< "bbb= "<<bbb<<std::endl;
  trace.info()<< "c= "<<c<<std::endl;


  hidden  =metric3.hiddenBy(a,b,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,b,c) returns false" << std::endl;
      
  hidden  =metric3.hiddenBy(a,bb,c,starting,endpoint,0); 
  nbok += (hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bb,c) returns true" << std::endl;
  
  hidden  =metric3.hiddenBy(a,bbb,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bbb,c) returns false" << std::endl;
  
  trace.endBlock();
  
  return nbok == nb;
}

bool testInexactMetrics()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing inexact predicate separable metrics l_2.1 ..." );

  Z2i::Point a( 0,0), b(5, 0), bb(5,-10), bbb(5,5),c(10,0);
  Z2i::Point starting( 0, 5), endpoint(10,5);
  
  InexactPredicateLpSeparableMetric<Z2i::Space> metric (2.1);

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

  trace.beginBlock ( "Testing inexact predicate separable metrics l_3.1 ..." );

   
  InexactPredicateLpSeparableMetric<Z2i::Space> metric3(3.1);
 
  trace.info()<< "a= "<<a<<std::endl;
  trace.info()<< "b= "<<b<<std::endl;
  trace.info()<< "bb= "<<bb<<std::endl;
  trace.info()<< "bbb= "<<bbb<<std::endl;
  trace.info()<< "c= "<<c<<std::endl;


  hidden  =metric3.hiddenBy(a,b,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,b,c) returns false" << std::endl;
      
  hidden  =metric3.hiddenBy(a,bb,c,starting,endpoint,0); 
  nbok += (hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bb,c) returns true" << std::endl;
  
  hidden  =metric3.hiddenBy(a,bbb,c,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bbb,c) returns false" << std::endl;
  
  trace.endBlock();
  
  return nbok == nb;
}
bool testWeightedMetrics()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing separable weighted metrics ..." );

  Z2i::Point a( 0,0), bbis(4, 1), b(5,0), bb(5,-10), bbb(5,5),c(10,0);
  Z2i::Point d(5,-2);
  Z2i::Point starting( 0, 5), endpoint(10,5);
  
  typedef ExactPredicateLpWeightedSeparableMetric<Z2i::Space, 2> Metric;
  Metric metric;

  trace.info()<< "a= "<<a<<std::endl;
  trace.info()<< "b= "<<b<<std::endl;
  trace.info()<< "bb= "<<bb<<std::endl;
  trace.info()<< "bbb= "<<bbb<<std::endl;
  trace.info()<< "c= "<<c<<std::endl;
  
  bool closer = (metric.closestWeighted(bbis,a,0,c,0) == DGtal::ClosestFIRST);  
  nbok += (closer) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "a is closer" << std::endl;

  closer = (metric.closestWeighted(bbis,a,10,c,35) == DGtal::ClosestFIRST);
  nbok += (!closer) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "c is closer with w_a=10 w_c=35" << std::endl;
  trace.endBlock();


  trace.beginBlock("Testing Hidden with w=0");
  bool hidden  =metric.hiddenByWeighted(a,0,b,0,c,0,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,b,c) returns false" << std::endl;
      
  hidden  =metric.hiddenByWeighted(a,0,bb,0,c,0,starting,endpoint,0); 
  nbok += (hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bb,c) returns true" << std::endl;
  
  hidden  =metric.hiddenByWeighted(a,0,bbb,0,c,0,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,bbb,c) returns false" << std::endl;

  hidden  =metric.hiddenByWeighted(a,0,d,0,c,0,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,d,c) returns false" << std::endl;
  trace.endBlock();

  trace.beginBlock("Testing Hidden with w!=0");

  hidden  =metric.hiddenByWeighted(a,10,d,0,c,10,starting,endpoint,0); 
  nbok += (hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,10,d,0,c,10) returns true" << std::endl;

  hidden  =metric.hiddenByWeighted(a,0,d,10,c,0,starting,endpoint,0); 
  nbok += (!hidden) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "(a,0,d,10,c,0) returns false" << std::endl;
  
  
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

  bool res = testMetrics()
    && testInexactMetrics()
    && testWeightedMetrics(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
