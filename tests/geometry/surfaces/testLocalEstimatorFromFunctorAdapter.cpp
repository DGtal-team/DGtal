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
 * @file testLocalEstimatorFromFunctorAdapter.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/28
 *
 * Functions for testing class LocalEstimatorFromFunctorAdapter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/graph/GraphVisitorRange.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/kernel/CanonicEmbedder.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/graph/DistanceBreadthFirstVisitor.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/surfaces/estimation/LocalEstimatorFromSurfelFunctorAdapter.h"
#include "DGtal/geometry/surfaces/estimation/BasicEstimatorFromSurfelsFunctors.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class LocalEstimatorFromFunctorAdapter.
///////////////////////////////////////////////////////////////////////////////



template <typename TPoint3>
struct ImplicitDigitalEllipse3 {
  typedef TPoint3 Point;
  inline
  ImplicitDigitalEllipse3( double a, double b, double c )
  : myA( a ), myB( b ), myC( c )
  {}
  inline
  bool operator()( const TPoint3 & p ) const
  {
    double x = ( (double) p[ 0 ] / myA );
    double y = ( (double) p[ 1 ] / myB );
    double z = ( (double) p[ 2 ] / myC );
    return ( x*x + y*y + z*z ) <= 1.0;
  }
  double myA, myB, myC;
};


/**
 * Example of a test. To be completed.
 *
 */
bool testLocalEstimatorFromFunctorAdapter()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  trace.beginBlock ( "Testing init ..." );

  using namespace Z3i;
  typedef ImplicitDigitalEllipse3<Point> ImplicitDigitalEllipse;
  typedef LightImplicitDigitalSurface<KSpace,ImplicitDigitalEllipse> Surface;
  typedef Surface::SurfelConstIterator ConstIterator;
  typedef Surface::Tracker Tracker;
  typedef Surface::Surfel Surfel;
  

  trace.beginBlock("Creating Surface");
  Point p1( -10, -10, -10 );
  Point p2( 10, 10, 10 );
  KSpace K;
  nbok += K.init( p1, p2, true ) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "K.init() is ok" << std::endl;
  ImplicitDigitalEllipse ellipse( 6.0, 4.5, 3.4 );
  Surfel bel = Surfaces<KSpace>::findABel( K, ellipse, 10000 );
  Surface surface( K, ellipse,
                    SurfelAdjacency<KSpace::dimension>( true ), bel );
  unsigned int nbsurfels = 0;
  for ( ConstIterator it = surface.begin(), it_end = surface.end();
       it != it_end; ++it )
  {
    ++nbsurfels;
  }
  trace.info() << nbsurfels << " surfels found." << std::endl;
  trace.endBlock();
  
  trace.beginBlock("Creating  adapter");
  typedef DummyEstimatorFromSurfels<Surfel, CanonicSCellEmbedder<KSpace> > Functor;
  typedef LocalEstimatorFromSurfelFunctorAdapter<Surface, Z3i::L2Metric, Functor> Reporter;
  
  Functor estimator(CanonicSCellEmbedder<KSpace>(surface.space()), 1);
                    
  Reporter reporter(surface, l2Metric, estimator);
  
  reporter.init(1, 5);
  Functor::Quantity val = reporter.eval( surface.begin());
  trace.info() <<  "Value with radius 5= "<<val << std::endl;
  nbok += (val == 124) ? 1 : 0;
  nb++;
 
  reporter.init(1, 20);
  Functor::Quantity val2 = reporter.eval( surface.begin());
  trace.info() <<  "Value with radius 20= "<<val2 << std::endl;
  nbok += (val2 == 398) ? 1 : 0;
  nb++;
  
  trace.endBlock();
  trace.endBlock();
  
  nbok += true ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class LocalEstimatorFromFunctorAdapter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testLocalEstimatorFromFunctorAdapter(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
