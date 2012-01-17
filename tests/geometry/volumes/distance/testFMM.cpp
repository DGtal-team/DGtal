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
 * @file testFMM.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/01/16
 *
 * @brief Functions for the fast marching method.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iomanip>
#include "DGtal/base/Common.h"

#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/BasicPointPredicates.h"

#include "DGtal/geometry/volumes/distance/IncrementalMetricComputers.h"
#include "DGtal/geometry/volumes/distance/FMM.h"

/*
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/geometry/volumes/distance/DistanceTransformation.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/io/boards/Board2D.h"
*/
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


/**
 * Example of a test. To be completed.
 *
 */
bool testFMM()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef PointVector<2,int> Point; 
  typedef double Distance;  
  typedef PointVector<2,Distance> Distances;  
  typedef IncrementalEuclideanMetricComputer<2> MetricComputer; 

  trace.beginBlock ( "Testing metric computer " );
 
  MetricComputer m; 
  trace.info() << m.compute( Distances(0,m.infinity()) ) << std::endl; 
  trace.info() << m.compute( Distances(1,m.infinity()) ) << std::endl; 
  trace.info() << m.compute( Distances(1,1) ) << std::endl; 
  trace.info() << m.compute( Distances(std::sqrt(2),2) ) << std::endl; 

  trace.endBlock();

  trace.beginBlock ( "Testing FMM " );
 
  std::map<Point, Distance> map; 
  map.insert( std::pair<Point, Distance>( Point(0,0), 0.0 ) );
 
  FMM<MetricComputer, DGtal::TruePointPredicate<Point> > f(map, m); 

  trace.info() << f << std::endl; 

  trace.endBlock();


  return nbok == nb;
}









///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main ( int argc, char** argv )
{
  trace.beginBlock ( "Testing FMM" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res =  testFMM()
;
  //&& ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
