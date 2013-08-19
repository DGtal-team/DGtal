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
 * @file testIntegralInvariantGaussianCurvatureEstimator3D.cpp
 * @ingroup Tests
 * @author Jérémy Levallois (\c jeremy.levallois@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), INSA-Lyon, France
 * LAboratoire de MAthématiques - LAMA (CNRS, UMR 5127), Université de Savoie, France
 *
 * @date 2012/11/28
 *
 * Functions for testing class IntegralInvariantGaussianCurvatureEstimator3D.
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
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.h"

#include "DGtal/kernel/BasicPointFunctors.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"

#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IntegralInvariantGaussianCurvatureEstimator3D.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */



bool testII()
{
    typedef ImplicitBall< Z2i::Space > Ball;
    typedef Z2i::RealPoint RealPoint;
    typedef Z2i::Domain Domain;

    /// Euclidean shape
    RealPoint rcenter( 0.0, 0.0 );
    Ball ball( rcenter, 5.0 );



    /// Digital shape
    typedef Z2i::Point Point;
    Point dcenter( 0, 0 );
    double h = 0.1;

    typedef GaussDigitizer< Z2i::Space, Ball > Digitizer;
    Digitizer* gauss = new Digitizer();
    gauss->attach( ball );
    gauss->init( RealPoint( -6.0, -6.0 ), RealPoint( 6.0, 6.0 ), h );
    Domain domain = gauss->getDomain();

    typedef PointFunctorFromPointPredicateAndDomain< Digitizer, Domain, unsigned int > MyPointFunctor;
    MyPointFunctor pointFunctor( gauss, domain, 1, 0 );

    typedef typename DigitalSetSelector< Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type MySet;
    MySet set( domain );
    Shapes< Domain >::digitalShaper( set, *gauss );

    /// Khalimsky shape

    Z2i::KSpace k;
    k.init( domain.lowerBound(), domain.upperBound(), true );

    typedef FunctorOnCells< MyPointFunctor, Z2i::KSpace > MyCellFunctor;
    MyCellFunctor cellFunctor ( pointFunctor, k );

    typedef IntegralInvariantGaussianCurvatureEstimator< Z2i::KSpace, MyCellFunctor > Estimator;
    Estimator estimator( k, cellFunctor );
    estimator.init( h, 3.0 );

    std::vector< double > results;
    std::back_insert_iterator< std::vector< double > > insertResults( results );

    typedef LightImplicitDigitalSurface< Z2i::KSpace, Digitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > DigSurface;
    typedef typename Z2i::KSpace::SCell SCell;
    SurfelAdjacency< Z2i::KSpace::dimension > SAdj( true );
    SCell bel;
    try
    {
        bel = Surfaces< Z2i::KSpace >::findABel( k, *gauss, 10000 );
    }
    catch( ... )
    {
        return false;
    }
    LightImplicitDigSurface LightImplDigSurf( k, *gauss, SAdj, bel );
    DigSurface digSurf( LightImplDigSurf );

    typedef DepthFirstVisitor< DigSurface > Visitor;
    typedef GraphVisitorRange< Visitor > VisitorRange;
    typedef typename VisitorRange::ConstIterator It;
    VisitorRange range( new Visitor( digSurf, *digSurf.begin() ) );
    It ibegin = range.begin();
    It iend = range.end();

    estimator.eval( ibegin, iend, ball, insertResults );


    /// Board
    Board2D board;
    //board.setUnit(LibBoard::Board::UCentimeter);
    Color red( 255, 0, 0 );
    Color dred( 192, 0, 0 );
    Color dgreen( 0, 192, 0 );
    Color blue( 0, 0, 255 );
    Color dblue( 0, 0, 192 );

    board << SetMode( domain.className(), "Grid" )
          << domain
//          << SetMode( gauss->className(), "Paving" )
          << set;
    board << CustomStyle( dcenter.className(), new CustomColors( red, dred ) )
          << dcenter;
    board.saveSVG("testII.svg");

    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;

    VisitorRange range2( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range2.begin();
    iend = range2.end();
    while ( ibegin != iend )
    {
        Dimension kdim = k.sOrthDir( *ibegin );
        SCell currentSCell = k.sIndirectIncident( *ibegin, kdim );
        CanonicSCellEmbedder< Z2i::KSpace > SCellToPoint( k );
        Point currentPoint = ( SCellToPoint( currentSCell ));
        board << currentPoint;
        ++ibegin;
    }
    board << CustomStyle( dcenter.className(), new CustomColors( red, dred ) )
          << dcenter;
    board.saveSVG("testII-outter.svg");

    board.clear();

    VisitorRange range3( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range3.begin();
    iend = range3.end();
    while ( ibegin != iend )
    {
        Dimension kdim = k.sOrthDir( *ibegin );
        SCell currentSCell = k.sDirectIncident( *ibegin, kdim );
        CanonicSCellEmbedder< Z2i::KSpace > SCellToPoint( k );
        Point currentPoint = ( SCellToPoint( currentSCell ));
        board << currentPoint;
        ++ibegin;
    }
    board << CustomStyle( dcenter.className(), new CustomColors( red, dred ) )
          << dcenter;
    board.saveSVG("testII-inner.svg");

    return true;
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testII();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
