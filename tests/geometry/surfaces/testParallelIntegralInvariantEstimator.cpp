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
 * @file testIntegralInvariantVolumeEstimator.cpp
 * @ingroup Tests
 * @author Bastien DOIGNIES (\c bastien.doignies@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2014/06/26
 *
 * Functions for testing class IntegralInvariantVolumeEstimator and IIGeometricFunctor.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <type_traits>
#include "DGtal/base/Common.h"

/// Shape
#include "DGtal/shapes/implicit/ImplicitBall.h"

/// Digitization
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"

/// Estimator
#include "DGtal/geometry/surfaces/estimation/IIGeometricFunctors.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantVolumeEstimator.h"
#include "DGtal/geometry/surfaces/estimation/ParallelIIEstimator.h"
#include "DGtal/kernel/domains/DomainSplitter.h"


///////////////////////////////////////////////////////////////////////////////


using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantVolumeEstimator and IIGeometricFunctor.
///////////////////////////////////////////////////////////////////////////////

bool testCurvature2dP ( double h )
{
  typedef ImplicitBall<Z2i::Space> ImplicitShape;
  typedef GaussDigitizer<Z2i::Space, ImplicitShape> DigitalShape;
  typedef LightImplicitDigitalSurface<Z2i::KSpace,DigitalShape> Boundary;
  typedef DigitalSurface< Boundary > MyDigitalSurface;
  typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
  typedef GraphVisitorRange< Visitor > VisitorRange;
  typedef VisitorRange::ConstIterator VisitorConstIterator;

  typedef functors::IICurvatureFunctor<Z2i::Space> MyIICurvatureFunctor;
  typedef IntegralInvariantVolumeEstimator< Z2i::KSpace, DigitalShape, MyIICurvatureFunctor > MyIICurvatureEstimator;

  //! [exampleParallelII-type]
  typedef RegularDomainSplitter<HyperRectDomain<Z2i::Space>> Splitter;
  typedef ParallelIIEstimator<MyIICurvatureEstimator, Splitter> MyIICurvatureEstimatorP;
  //! [exampleParallelII-type]

  typedef MyIICurvatureEstimator::Quantity Value;

  static_assert(std::is_same_v<typename MyIICurvatureEstimator::Quantity, typename MyIICurvatureEstimatorP::Quantity>);

  double re = 10;
  double radius = 15;

  trace.beginBlock( "[PARALLEL] Shape initialisation ..." );

  ImplicitShape ishape( Z2i::RealPoint( 0, 0 ), radius );
  DigitalShape dshape;
  dshape.attach( ishape );
  dshape.init( Z2i::RealPoint( -20.0, -20.0 ), Z2i::RealPoint( 20.0, 20.0 ), h );

  Z2i::KSpace K;
  if ( !K.init( dshape.getLowerBound(), dshape.getUpperBound(), true ) )
  {
    trace.error() << "Problem with Khalimsky space" << std::endl;
    return false;
  }

  Z2i::KSpace::Surfel bel = Surfaces<Z2i::KSpace>::findABel( K, dshape, 10000 );
  Boundary boundary( K, dshape, SurfelAdjacency<Z2i::KSpace::dimension>( true ), bel );
  MyDigitalSurface surf ( boundary );

  trace.endBlock();

  trace.beginBlock( "Curvature estimator initialisation ...");

  // Visitor ranges are typically unique and single pass. We need
  // to create one for each estimator in this case
  VisitorRange range( new Visitor( surf, *surf.begin() ));
  VisitorConstIterator ibegin = range.begin();
  VisitorConstIterator iend = range.end();
  // Parallel iterations
  VisitorRange rangeP( new Visitor( surf, *surf.begin() ));
  VisitorConstIterator ibeginP = rangeP.begin();
  VisitorConstIterator iendP = rangeP.end();

  MyIICurvatureFunctor curvatureFunctor;
  curvatureFunctor.init( h, re );

  //! [exampleParallelII-construction]
  MyIICurvatureEstimator  curvatureEstimator (    curvatureFunctor);
  curvatureEstimator.attach( K, dshape );
  curvatureEstimator.setParams( re/h );
  curvatureEstimator.init( h, ibegin, iend );

  // Parallel version expects a number of thread as first argument.
  // Subsequent arguments are forwarded to underlying estimator
  // init / setParams and init (and eval) remains the same.

  MyIICurvatureEstimatorP curvatureEstimatorP( 4, curvatureFunctor );
  curvatureEstimatorP.attach( K, dshape );
  curvatureEstimatorP.setParams( re/h );
  curvatureEstimatorP.init( h, ibeginP, iendP );
  //! [exampleParallelII-construction]

  trace.endBlock();

  trace.beginBlock( "Curvature estimator evaluation ...");

  std::vector< Value > results, resultsP;
  std::back_insert_iterator< std::vector< Value > > resultsIt ( results  );
  std::back_insert_iterator< std::vector< Value > > resultsItP( resultsP );

  curvatureEstimator .eval( ibegin , iend , resultsIt  );
  curvatureEstimatorP.eval( ibeginP, iendP, resultsItP );

  trace.endBlock();

  trace.beginBlock ( "Comparing results of integral invariant 2D curvature ..." );

  unsigned int rsize  = results.size();
  unsigned int rsizeP = resultsP.size();

  if (rsize != rsizeP)
  {
    trace.error() << "Size mismatch between parallel and non-parallel versions: " << rsize << " / " << rsizeP;
    trace.endBlock();
    return false;
  }

  for ( unsigned int i = 0; i < rsize; ++i )
  {
    if (std::abs(results[i] - resultsP[i]) >= 1e-2)
    {
      trace.error() << "Result mismatch between parallel and non-parallel versions at voxel " << i << ": " << results[i] << " / " << resultsP[i] << "\n";
      trace.endBlock();
      return false;
    }
  }
  trace.endBlock();
  return true;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
int main( int /*argc*/, char** /*argv*/ )
{
    trace.beginBlock ( "Testing class ParrallelIIEstimator with IntegralInvariantVolumeEstimator in 2d" );

    bool res = testCurvature2dP( 0.05 );
    trace.emphase() << ( res ? "Passed." : "Error." ) << std::endl;
    trace.endBlock();
    return res ? 0 : 1;
}
