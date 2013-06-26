/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 **/

/**
 * @file testPolynomial.cpp
 * @ingroup tests
 * @author Anis Benyoub (anis.benyoub@insa-lyon.fr)
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/06/21
 *
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <QtGui/QApplication>
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/shapes/implicit/ImplicitFunctionDiff1LinearCellEmbedder.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/topology/SCellsFunctors.h"
#include "DGtal/topology/helpers/BoundaryPredicate.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include <boost/math/special_functions/fpclassify.hpp>

#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.h"
#include "DGtal/geometry/surfaces/FunctorOnCells.h"
#include "DGtal/images/ImageHelper.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////


void usage( int /*argc*/, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <Polynomial> <Px> <Py> <Pz> <Qx> <Qy> <Qz> <step>" << std::endl;
  std::cerr << "\t - displays the boundary of a shape defined implicitly by a 3-polynomial <Polynomial>." << std::endl;
  std::cerr << "\t - P and Q defines the bounding box." << std::endl;
  std::cerr << "\t - step is the grid step." << std::endl;
  std::cerr << "\t - You may try x^3y+xz^3+y^3z+z^3+5z or (y^2+z^2-1)^2 +(x^2+y^2-1)^3 " << std::endl;
  std::cerr << "\t - See http://www.freigeist.cc/gallery.html" << std::endl;
}

  typedef Space::RealPoint RealPoint;
  typedef RealPoint::Coordinate Ring;
  typedef MPolynomial<3, Ring> Polynomial3;
  typedef MPolynomialReader<3, Ring> Polynomial3Reader;
  typedef ImplicitPolynomial3Shape<Space> ImplicitShape;
  typedef GaussDigitizer<Space,ImplicitShape> DigitalShape;
  typedef DigitalShape::PointEmbedder DigitalEmbedder;

template < typename TKSpace, typename TShape >
class NearestPointEmbedder
{
public:
  typedef TKSpace KSpace;
  typedef TShape Shape;
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::Space Space;
  typedef typename Space::RealPoint RealPoint;
  typedef Cell Argument;
  typedef RealPoint Value;

  NearestPointEmbedder()
    :shape(0)
  {}

  ~NearestPointEmbedder(){}

  void init( const KSpace & K, const double h, const Shape & s )
  {
    k = K;
    step = h;
    shape = &s;
  }

  RealPoint operator()( const Cell & cell ) const
  {
    Point B = k.uKCoords( cell );
    RealPoint A( B[ 0 ], B[1], B[2] );
    A /= 2.0;
    A *= step;
    A = shape->nearestPoint( A, 0.01 * step, 200, 0.1 * step );
    return A;
  }

protected:
  KSpace k;
  double step;
  const Shape* shape;
};


int main( int argc, char** argv )
{
  if ( argc < 10 )
  {
    usage( argc, argv );
    return 1;
  }
  double p1[ 3 ];
  double p2[ 3 ];
  for ( unsigned int i = 0; i < 3; ++i )
  {
    p1[ i ] = atof( argv[ 2 + i ] );
    p2[ i ] = atof( argv[ 5 + i ] );
  }
  double step = atof( argv[ 8 ] );

  double re = atof( argv[ 9 ] );


  Polynomial3 P;
  Polynomial3Reader reader;
  std::string poly_str = argv[ 1 ];
  std::string::const_iterator iter
      = reader.read( P, poly_str.begin(), poly_str.end() );
  if ( iter != poly_str.end() )
  {
    std::cerr << "ERROR: I read only <"
              << poly_str.substr( 0, iter - poly_str.begin() )
              << ">, and I built P=" << P << std::endl;
    return 1;
  }


  ImplicitShape ishape( P );
  DigitalShape dshape;
  dshape.attach( ishape );
  dshape.init( RealPoint( p1 ), RealPoint( p2 ), step );
  Domain domain = dshape.getDomain();

  KSpace K;

  bool space_ok = K.init( domain.lowerBound(),
                          domain.upperBound(), true
                          );
  if ( !space_ok )
  {
    trace.error() << "Error in the Khamisky space construction." << std::endl;
    return 2;
  }

  typedef SurfelAdjacency< KSpace::dimension > MySurfelAdjacency;
  MySurfelAdjacency surfAdj( true ); // interior in all directions.


  typedef KSpace::Surfel Surfel;
  typedef KSpace::SurfelSet SurfelSet;
  typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
  typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;


  MySetOfSurfels theSetOfSurfels( K, surfAdj );
  Surfel bel = Surfaces< KSpace >::findABel( K, dshape, 100000 );
  Surfaces< KSpace >::trackBoundary( theSetOfSurfels.surfelSet(),
                                   K, surfAdj,
                                   dshape, bel );

  MyDigitalSurface digSurf( theSetOfSurfels );


  QApplication application( argc, argv );
  Viewer3D viewer;
  viewer.show();
  viewer << SetMode3D( domain.className(), "BoundingBox" ) << domain;

  typedef typename ImageSelector< Domain, unsigned int >::Type Image;
  typedef ImageToConstantFunctor< Image, DigitalShape > MyPointFunctor;
  typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
  typedef IntegralInvariantGaussianCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator


  Image image( domain );
  DGtal::imageFromRangeAndValue( domain.begin(), domain.end(), image );
  MyPointFunctor pointFunctor( &image, &dshape, 1, true );
  MyCellFunctor functor ( pointFunctor, K );
  MyCurvatureEstimator iigaussest ( K, functor );
  iigaussest.init( step, re );


  std::vector< double > resultTrue;
  std::vector< double > resultII;
  std::vector< RealPoint > nearestPoints;
  //-----------------------------------------------------------------------
  // Looking for the min and max values

  double minCurv = 1;
  double maxCurv = 0;
  SCellToMidPoint< KSpace > midpoint( K );
  for ( std::set< SCell >::iterator it = theSetOfSurfels.begin(), it_end = theSetOfSurfels.end();
        it != it_end; ++it)
  {

    RealPoint A = midpoint( *it ) * step;
    A = ishape.nearestPoint (A, 0.01 * step, 200, 0.1 * step);
//    double a = ishape.meanCurvature( A );
    double a = ishape.gaussianCurvature(A);
    double b = iigaussest.eval( it );

    resultTrue.push_back( a );
    resultII.push_back( b );
    nearestPoints.push_back( A );

    if ( boost::math::isnan( a ))
    {
      a = 0;
    }

    double Linf = std::abs ( a - b );
    if ( a > maxCurv )
      {
        maxCurv = a;
      }
    else if ( a < minCurv )
      {
        minCurv = a;
      }

  }

  trace.info() << " Min = " << minCurv << std::endl;
  trace.info() << " Max = " << maxCurv << std::endl;


  //-----------------------------------------------------------------------
  //Specifing a color map

  GradientColorMap< double > cmap_grad( minCurv, maxCurv );
  cmap_grad.addColor( Color::Blue );
  cmap_grad.addColor( Color::White );
  cmap_grad.addColor( Color::Red );
//  cmap_grad.addColor( Color( 50, 50, 255 ) );
//  cmap_grad.addColor( Color( 255, 0, 0 ) );
//  cmap_grad.addColor( Color( 255, 255, 10 ) );

  //------------------------------------------------------------------------------------
  //drawing
  unsigned int nbSurfels = 0;

//  ofstream out( "rounded2.off" );
  int i = 0;
  for ( std::set<SCell>::iterator it = theSetOfSurfels.begin(),
        it_end = theSetOfSurfels.end();
        it != it_end; ++it, ++nbSurfels, ++i )
  {
    double a = resultTrue[ i ];
    double b = resultII[ i ];
    double Linf = std::abs ( a - b );

    viewer << CustomColors3D( Color::Black, cmap_grad( a ));
    viewer << *it;
  }

//  typedef NearestPointEmbedder< KSpace, ImplicitShape > NPEmbedder;
//  NPEmbedder npEmbedder;
//  npEmbedder.init( K, step, ishape );

//  if ( out.good() )
//    digSurf.exportEmbeddedSurfaceAs3DOFF( out, npEmbedder );
//  out.close();

  viewer << Viewer3D::updateDisplay;

  return application.exec();
}
