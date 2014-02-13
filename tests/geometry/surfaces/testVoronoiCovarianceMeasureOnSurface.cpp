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
 * @file testVoronoiCovarianceMeasure.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/02/09
 *
 * Functions for testing class VoronoiCovarianceMeasure.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/math/Statistic.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/geometry/surfaces/estimation/VoronoiCovarianceMeasureOnDigitalSurface.h"

///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class VoronoiCovarianceMeasureOnSurface
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
bool testVoronoiCovarianceMeasureOnSurface()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  using namespace DGtal;
  using namespace DGtal::Z3i; // gets Space, Point, Domain

  typedef ImplicitDigitalEllipse3<Point> ImplicitDigitalEllipse;
  typedef LightImplicitDigitalSurface<KSpace,ImplicitDigitalEllipse> SurfaceContainer;
  typedef DigitalSurface< SurfaceContainer > Surface;
  typedef Surface::ConstIterator ConstIterator;
  typedef SurfaceContainer::Surfel Surfel;
  typedef ExactPredicateLpSeparableMetric<Space,2> Metric;
  typedef VoronoiCovarianceMeasureOnDigitalSurface<SurfaceContainer,Metric> VCMOnSurface;

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
  SurfaceContainer* surfaceContainer = new SurfaceContainer
    ( K, ellipse, SurfelAdjacency<KSpace::dimension>( true ), bel );
  CountedConstPtrOrConstPtr<Surface> ptrSurface( new Surface( surfaceContainer ) ); // acquired
  trace.endBlock();

  trace.beginBlock("Computing VCM on surface." );
  VCMOnSurface vcm_surface( ptrSurface, VCMOnSurface::Pointels, 5.0, 4.0, 4.0, Metric(), true );
  trace.endBlock();

  trace.beginBlock("Evaluating normals." );
  Statistic<double> error;
  for ( ConstIterator it = ptrSurface->begin(), itE = ptrSurface->end(); it != itE; ++it )
    {
      const VCMOnSurface::Normals & normals = vcm_surface.surfelNormals().find( *it )->second;
      error.addValue( normals.vcmNormal.dot( normals.trivialNormal ) );
    }
  error.terminate();
  trace.info() << "cos angle avg = " << error.mean() << std::endl;
  trace.info() << "cos angle dev = " << sqrt( error.unbiasedVariance() ) << std::endl;
  nbok += error.mean() > 0.95 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "cos angle avg > 0.95" << std::endl;
  nbok += sqrt( error.unbiasedVariance() ) < 0.05 ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "cos angle dev < 0.05" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  using namespace std;
  using namespace DGtal;
  trace.beginBlock ( "Testing VoronoiCovarianceMeasureOnSurface ..." );
  bool res = testVoronoiCovarianceMeasureOnSurface();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
