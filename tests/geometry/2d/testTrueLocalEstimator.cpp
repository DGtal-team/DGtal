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
 * @file testTrueLocalEstimator.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/06/30
 *
 * Functions for testing class TrueLocalEstimator.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/ShapeFactory.h"

#include "DGtal/geometry/2d/TrueLocalEstimatorOnPoints.h"

#include "DGtal/geometry/2d/ParametricShapeCurvatureFunctor.h"
#include "DGtal/geometry/2d/ParametricShapeTangentFunctor.h"


#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetSelector.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"


#include "DGtal/geometry/2d/GridCurve.h"


#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class TrueLocalEstimator.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testTrueLocalEstimator(const std::string &filename)
{
  trace.info() << "Reading GridCurve " << endl;
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);
  typedef KhalimskySpaceND<2> Kspace; //space
  GridCurve<Kspace> c(instream); //building grid curve
  typedef GridCurve<Kspace >::PointsRange Range;//range
  Range r = c.getPointsRange();//building range

  
  typedef Ball2D<Z2i::Space> Shape;
  typedef GridCurve<KhalimskySpaceND<2> >::PointsRange Range;
  typedef Range::ConstIterator ConstIteratorOnPoints;
  typedef ParametricShapeCurvatureFunctor< Shape, ConstIteratorOnPoints > Curvature;
  typedef ParametricShapeTangentFunctor< Shape, ConstIteratorOnPoints > Tangent;

  Shape ball(Z2i::Point(0,0), 30);

   
  TrueLocalEstimatorOnPoints< ConstIteratorOnPoints, Shape, Curvature  >  curvatureEstimator;

  TrueLocalEstimatorOnPoints< ConstIteratorOnPoints, Shape, Tangent  >  tangentEstimator;

  curvatureEstimator.init( 1, r.begin(), r.end(), &ball, true);
  tangentEstimator.init( 1, r.begin(), r.end(), &ball, true);
 
  ConstIteratorOnPoints it = r.begin();
  trace.info() << "Current point = "<<*it<<std::endl;
  trace.info() << "Eval curvature (begin, h=1) = "<< curvatureEstimator.eval(it)<<std::endl;
  trace.info() << "Eval tangent (begin, h=1) = "<< tangentEstimator.eval(it)<<std::endl;
  
  return true;

}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class TrueLocalEstimator" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;



  std::string sinus2D4 = testPath + "samples/sinus2D4.dat";

  bool res = testTrueLocalEstimator(sinus2D4); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
