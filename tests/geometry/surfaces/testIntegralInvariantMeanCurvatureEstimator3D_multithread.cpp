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
 * @file testIntegralInvariantMeanCurvatureEstimator3D.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/11/28
 *
 * Functions for testing class IntegralInvariantMeanCurvatureEstimator3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"

#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/geometry/surfaces/FunctorOnCells.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantMeanCurvatureEstimator3D.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testIntegralInvariantMeanCurvatureEstimator3D( double h, double delta )
{
  typedef Z3i::KSpace::Surfel Surfel;
  typedef Z3i::Space::RealPoint::Coordinate Ring;
  typedef MPolynomial< 3, Ring > Polynomial3;
  typedef MPolynomialReader< 3, Ring > Polynomial3Reader;
  typedef ImplicitPolynomial3Shape< Z3i::Space > MyShape;
  typedef GaussDigitizer< Z3i::Space, MyShape > MyGaussDigitizer;
  typedef LightImplicitDigitalSurface< Z3i::KSpace, MyGaussDigitizer > MyLightImplicitDigitalSurface;
  typedef DigitalSurface< MyLightImplicitDigitalSurface > MyDigitalSurface;
  typedef ImageSelector< Z3i::Domain, unsigned int >::Type Image;
  typedef ImageToConstantFunctor< Image, MyGaussDigitizer > MyPointFunctor;
  typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
  typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
  typedef GraphVisitorRange< Visitor > VisitorRange;
  typedef VisitorRange::ConstIterator SurfelConstIterator;
  typedef IntegralInvariantMeanCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyIIMeanEstimator;
  typedef MyIIMeanEstimator::Quantity Quantity;
  typedef MyShape::RealPoint RealPoint;

  std::string poly_str = "x^2 + y^2 + z^2 - 25";
  double border_min[3] = { -10, -10, -10 };
  double border_max[3] = { 10, 10, 10 };
  double re_convolution_kernel = 4.217163327;
  double realValue = 0.2; // = 1/r

  trace.beginBlock ( "Testing integral invariant 3D Mean curvature initialization ..." );

  Polynomial3 poly;
  Polynomial3Reader reader;
  std::string::const_iterator iter = reader.read ( poly, poly_str.begin(), poly_str.end() );
  if ( iter != poly_str.end() )
  {
    std::cerr << "ERROR: I read only <"
              << poly_str.substr( 0, iter - poly_str.begin() )
              << ">, and I built P=" << poly << std::endl;
    return false;
  }

  MyShape shape( poly );

  MyGaussDigitizer gaussDigShape;
  gaussDigShape.attach( shape );
  gaussDigShape.init( RealPoint( border_min ), RealPoint( border_max ), h );
  Z3i::Domain domain = gaussDigShape.getDomain();
  Z3i::KSpace kSpace;
  bool space_ok = kSpace.init( domain.lowerBound(), domain.upperBound(), true );
  if (!space_ok)
  {
    trace.error() << "Error in the Khalimsky space construction."<<std::endl;
    return false;
  }

  Image image( domain );
  DGtal::imageFromRangeAndValue( domain.begin(), domain.end(), image );

  SurfelAdjacency< Z3i::KSpace::dimension > SAdj( true );
  Surfel bel = Surfaces< Z3i::KSpace >::findABel( kSpace, gaussDigShape, 100000 );
  MyLightImplicitDigitalSurface lightImplDigSurf( kSpace, gaussDigShape, SAdj, bel );
  MyDigitalSurface digSurfShape( lightImplDigSurf );

  MyPointFunctor pointFunctor( &image, &gaussDigShape, 1, true );
  MyCellFunctor functorShape ( pointFunctor, kSpace );
  MyIIMeanEstimator estimator ( kSpace, functorShape );

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
  std::back_insert_iterator< std::vector< Quantity > > resultsIICurvatureIterator( resultsIICurvature );

  VisitorRange range( new Visitor( digSurfShape, *digSurfShape.begin() ) );
  SurfelConstIterator abegin = range.begin();
  SurfelConstIterator aend = range.end();

  trace.endBlock();
  trace.beginBlock ( "Testing integral invariant 3D mean curvature computation ..." );
  
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

  trace.beginBlock ( "Comparing results of integral invariant 3D mean curvature ..." );

  double mean = 0.0;
  unsigned int rsize = resultsIICurvature.size();

  for ( unsigned int i = 0; i < rsize; ++i )
  {
    mean += resultsIICurvature[ i ];
  }
  mean /= rsize;

  if ( std::abs ( realValue - mean ) > delta )
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
  trace.beginBlock ( "Testing class IntegralInvariantMeanCurvatureEstimator3D" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << std::endl;

  bool res = testIntegralInvariantMeanCurvatureEstimator3D( 0.6, 0.008 ); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
