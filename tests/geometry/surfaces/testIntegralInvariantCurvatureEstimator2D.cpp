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
 * @file testIntegralInvariantCurvatureEstimator2D.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/11/28
 *
 * Functions for testing class IntegralInvariantCurvatureEstimator2D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/parametric/AccFlower2D.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/geometry/surfaces/FunctorOnCells.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeCurvatureFunctor.h"
#include "DGtal/geometry/curves/estimation/TrueLocalEstimatorOnPoints.h"

///////////////////////////////////////////////////////////////////////////////


using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantCurvatureEstimator2D.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testIntegralInvariantCurvatureEstimator2D ( double h, double delta )
{
  typedef AccFlower2D< Z2i::Space > MyShape;
  typedef Z2i::KSpace::Surfel Surfel;
  typedef GaussDigitizer< Z2i::Space, MyShape > MyGaussDigitizer;
  typedef LightImplicitDigitalSurface< Z2i::KSpace, MyGaussDigitizer > MyLightImplicitDigitalSurface;
  typedef DigitalSurface< MyLightImplicitDigitalSurface > MyDigitalSurface;
  typedef ImageSelector< Z2i::Domain, unsigned int >::Type Image;
  typedef ImageToConstantFunctor< Image, MyGaussDigitizer > MyPointFunctor;
  typedef FunctorOnCells< MyPointFunctor, Z2i::KSpace > MyCellFunctor;
  typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
  typedef GraphVisitorRange< Visitor > VisitorRange;
  typedef IntegralInvariantMeanCurvatureEstimator< Z2i::KSpace, MyCellFunctor > MyIIMeanEstimator;
  typedef VisitorRange::ConstIterator SurfelConstIterator;
  typedef MyIIMeanEstimator::Quantity Quantity;

  double max_radius_shape = 20.00217;
  double min_radius_shape = 5.00217;
  double re_convolution_kernel = 1.842015749;

  trace.beginBlock ( "Testing initialization of integral invariant 2D curvature ..." );

  MyShape shape( 0, 0, max_radius_shape, min_radius_shape, 3, 0.0 );

  MyGaussDigitizer gaussDigShape;
  gaussDigShape.attach( shape );
  gaussDigShape.init( shape.getLowerBound(), shape.getUpperBound(), h );
  Z2i::Domain domainShape = gaussDigShape.getDomain();
  Z2i::KSpace kSpace;
  bool space_ok = kSpace.init( domainShape.lowerBound(), domainShape.upperBound(), true );
  if ( !space_ok )
  {
    trace.error() << "Error in the Khalimsky space construction." << std::endl;
    return 2;
  }

  Image image( domainShape );
  DGtal::imageFromRangeAndValue( domainShape.begin(), domainShape.end(), image );

  SurfelAdjacency< Z2i::KSpace::dimension > SAdj( true );
  Surfel bel = Surfaces< Z2i::KSpace >::findABel( kSpace, gaussDigShape, 100000 );
  MyLightImplicitDigitalSurface lightImplDigSurf( kSpace, gaussDigShape, SAdj, bel );
  MyDigitalSurface digSurfShape( lightImplDigSurf );

  MyPointFunctor pointFunctor( &image, &gaussDigShape, 1 );
  MyCellFunctor functorShape( pointFunctor, kSpace );
  MyIIMeanEstimator estimator( kSpace, functorShape );

  try
  {
    estimator.init( h, re_convolution_kernel );
  }
  catch(...)
  {
    trace.endBlock();
    return false;
  }

  std::vector< Quantity > resultsIICurvature;
  back_insert_iterator< std::vector< Quantity > > resultsIICurvatureIterator( resultsIICurvature );

  VisitorRange range( new Visitor( digSurfShape, *digSurfShape.begin() ) );
  SurfelConstIterator abegin = range.begin();
  SurfelConstIterator aend = range.end();

  trace.endBlock();
  trace.beginBlock ( "Testing integral invariant 2D curvature computation ..." );

  try
  {
    estimator.eval( abegin, aend, resultsIICurvatureIterator );
  }
  catch(...)
  {
    trace.endBlock();
    return false;
  }

  trace.endBlock();

  typedef ParametricShapeCurvatureFunctor< MyShape > CurvatureFunctor;
  typedef GridCurve< Z2i::KSpace >::PointsRange PointsRange;
  typedef PointsRange::ConstIterator ConstIteratorOnPoints;
  typedef Z2i::Space::Point Point;
  typedef TrueLocalEstimatorOnPoints< ConstIteratorOnPoints, MyShape, CurvatureFunctor > MyTrueLocalEstimator;

  std::vector< Point > points;
  Surfaces< Z2i::KSpace >::track2DBoundaryPoints( points, kSpace, SAdj, gaussDigShape, bel );
  GridCurve< Z2i::KSpace > gridcurve;
  gridcurve.initFromVector( points );
  PointsRange r = gridcurve.getPointsRange();
  MyTrueLocalEstimator trueCurvatureEstimator;
  trueCurvatureEstimator.attach ( &shape );
  trueCurvatureEstimator.init( h, r.begin(), r.end() );
  std::vector< MyTrueLocalEstimator::Quantity > resultsTrueCurvature;
  back_insert_iterator< std::vector< MyTrueLocalEstimator::Quantity > > resultsTrueCurvatureIterator( resultsTrueCurvature );
  trueCurvatureEstimator.eval( r.begin(), r.end(), resultsTrueCurvatureIterator );

  trace.beginBlock ( "Comparing results of integral invariant 2D curvature and true local 2D curvature ..." );

  unsigned int rsize = r.size() - 1;
  double L2error = 0.0;
  double diff = 0.0;
  for ( unsigned int i = 0; i < rsize; ++i )
  {
    diff = std::abs ( resultsTrueCurvature[ i ] - resultsIICurvature[ rsize - i ] );
    L2error += diff * diff;
  }

  if (( sqrt( L2error ) / rsize ) > delta )
  {
    trace.endBlock();
    return false;
  }

  trace.endBlock();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class IntegralInvariantCurvatureEstimator2D" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testIntegralInvariantCurvatureEstimator2D( 0.05, 0.00334 ); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
