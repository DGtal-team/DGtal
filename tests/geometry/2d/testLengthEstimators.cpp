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
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) 
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/07
 *
 * Functions for testing classes of length estimators.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include "DGtal/base/Common.h"

//space / domain
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"

//shape and digitizer
#include "DGtal/helpers/ShapeFactory.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Surfaces.h"

#include "DGtal/geometry/nd/GaussDigitizer.h"
#include "DGtal/geometry/2d/GridCurve.h"

//estimators
#include "DGtal/geometry/2d/TrueLocalEstimatorOnPoints.h"
#include "DGtal/geometry/2d/TrueGlobalEstimatorOnPoints.h"
#include "DGtal/geometry/2d/ParametricShapeCurvatureFunctor.h"
#include "DGtal/geometry/2d/ParametricShapeTangentFunctor.h"
#include "DGtal/geometry/2d/ParametricShapeArcLengthFunctor.h"

#include "DGtal/geometry/2d/L1LengthEstimator.h"
#include "DGtal/geometry/2d/MLPLengthEstimator.h"
#include "DGtal/geometry/2d/FPLengthEstimator.h"
#include "DGtal/geometry/2d/DSSLengthEstimator.h"

#include "ConfigTest.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing Length Estimator classes.
///////////////////////////////////////////////////////////////////////////////

bool testLengthEstimatorsOnBall(double radius, double h)
{

  // Types
  typedef Ball2D<Space> Shape;
  typedef Space::Point Point;
  typedef Space::RealPoint RealPoint;
  typedef Space::Integer Integer;
  typedef HyperRectDomain<Space> Domain;
  typedef KhalimskySpaceND<Space::dimension,Integer> KSpace;
  typedef KSpace::SCell SCell;
  typedef GridCurve<KSpace>::PointsRange PointsRange;
  typedef GridCurve<KSpace>::ArrowsRange ArrowsRange;
  typedef PointsRange::ConstIterator ConstIteratorOnPoints;
  typedef ParametricShapeTangentFunctor< Shape, ConstIteratorOnPoints > Tangent;
  typedef ParametricShapeCurvatureFunctor< Shape, ConstIteratorOnPoints > Curvature;


  //Forme
  Shape aShape(Point(0,0), radius);

  trace.info() << "#ball created, r=" << radius << endl;

  // Window for the estimation
  RealPoint xLow ( -radius-1, -radius-1 );
  RealPoint xUp( radius+1, radius+1 );
  GaussDigitizer<Space,Shape> dig;  
  dig.attach( aShape ); // attaches the shape.
  dig.init( xLow, xUp, h ); 
  // The domain size is given by the digitizer according to the window
  // and the step.
  Domain domain = dig.getDomain();
  // Create cellular space
  KSpace K;
  bool ok = K.init( dig.getLowerBound(), dig.getUpperBound(), true );
  if ( ! ok )
    {
      std::cerr << " "
		<< " error in creating KSpace." << std::endl;
      return false;
    }
  try {

    // Extracts shape boundary
    SurfelAdjacency<KSpace::dimension> SAdj( true );
    SCell bel = Surfaces<KSpace>::findABel( K, dig, 10000 );
    // Getting the consecutive surfels of the 2D boundary
    std::vector<Point> points;
    Surfaces<KSpace>::track2DBoundaryPoints( points, K, SAdj, dig, bel );
    // Create GridCurve
    GridCurve<KSpace> gridcurve;
    gridcurve.initFromVector( points );

    trace.info() << "#grid curve created, h=" << h << endl;

    //ranges
    ArrowsRange ra = gridcurve.getArrowsRange(); 
    PointsRange rp = gridcurve.getPointsRange(); 

    ////////////////////////////////////////estimations
    double trueValue = M_PI*2*radius;
    L1LengthEstimator< GridCurve<KSpace>::ArrowsRange::ConstIterator > l1length;
    l1length.init(h, ra.begin(), ra.end(), gridcurve.isClosed());
    DSSLengthEstimator< GridCurve<KSpace>::PointsRange::ConstIterator > DSSlength;
    DSSlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());
    MLPLengthEstimator< GridCurve<KSpace>::PointsRange::ConstIterator > MLPlength;
    MLPlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());
    FPLengthEstimator< GridCurve<KSpace>::PointsRange::ConstIterator > FPlength;
    FPlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());

    trace.info() << "#Estimations" <<std::endl;
    trace.info() << "#h true naive DSS MLP FP " <<std::endl;
    trace.info() << h << " " << trueValue  
    << " " << l1length.eval() <<  " " << DSSlength.eval() 
    << " " << MLPlength.eval() <<  " " << FPlength.eval() << std::endl;

  }    
  catch ( InputException e )
    {
      std::cerr << " "
		<< " error in finding a bel." << std::endl;
      return false;
    }



  return true;
}



///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class LengthEstimators" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  double r = 5; 
  bool res = testLengthEstimatorsOnBall(r,1)
  && testLengthEstimatorsOnBall(r,0.1)
  && testLengthEstimatorsOnBall(r,0.01)
  && testLengthEstimatorsOnBall(r,0.001)
;

  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
