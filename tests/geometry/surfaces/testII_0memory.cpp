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
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
#include "DGtal/io/readers/MPolynomialReader.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/topology/LightImplicitDigitalSurface.h"
#include "DGtal/kernel/BasicPointFunctors.h"
#include "DGtal/geometry/surfaces/FunctorOnCells.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator_0memory.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator_0memory.h"



/*#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/graph/DepthFirstVisitor.h"
#include "DGtal/graph/GraphVisitorRange.h"



#include "DGtal/io/boards/Board2D.h"


#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/implicit/ImplicitFunctionDiff1LinearCellEmbedder.h"
#include "DGtal/topology/SCellsFunctors.h"
#include "DGtal/topology/helpers/BoundaryPredicate.h"
#include "DGtal/topology/SetOfSurfels.h"*/


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

template < typename TKSpace, typename TShape >
class NearestPointEmbedder
{
public:
    typedef TKSpace KSpace;
    typedef TShape Shape;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Space Space;
    typedef typename Space::RealPoint RealPoint;
    typedef Z3i::Cell Argument;
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

    RealPoint operator()( const SCell & cell ) const
    {
        SCellToMidPoint< KSpace > embedder;
        RealPoint A = embedder( cell ) * step;
        //    Point B = k.uKCoords( cell );
        //    RealPoint A( B[ 0 ], B[1], B[2] );
        //    A /= 2.0;
        //    A *= step;
        A = shape->nearestPoint( A, 0.01 * step, 200, 0.1 * step );
        return A;
    }

protected:
    KSpace k;
    double step;
    const Shape* shape;
};

bool testII2D_Gaussian()
{
    return false;
    /*
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
//        SCellToOuterPoint< Z2i::KSpace > SCellToPoint( k );
//        CanonicSCellEmbedder< Z2i::KSpace > SCellToPoint( k );
        Point currentPoint = ( k.sCoords( currentSCell ));
        board << currentPoint;
        ++ibegin;
    }
    board << CustomStyle( dcenter.className(), new CustomColors( red, dred ) )
          << dcenter;
    board.saveSVG("testII-outer.svg");

    board.clear();

    VisitorRange range3( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range3.begin();
    iend = range3.end();
    while ( ibegin != iend )
    {
        Dimension kdim = k.sOrthDir( *ibegin );
        SCell currentSCell = k.sDirectIncident( *ibegin, kdim );
//        SCellToInnerPoint< Z2i::KSpace > SCellToPoint( k );
//        CanonicSCellEmbedder< Z2i::KSpace > SCellToPoint( k );
        Point currentPoint = ( k.sCoords( currentSCell ));
        board << currentPoint;
        ++ibegin;
    }
    board << CustomStyle( dcenter.className(), new CustomColors( red, dred ) )
          << dcenter;
    board.saveSVG("testII-inner.svg");

    return true;*/
}

int testII3D_Gaussian( )//int argc, char** argv )
{
    typedef Z3i::Space::RealPoint RealPoint;
    typedef Z3i::Point Point;
    typedef Z3i::KSpace::Surfel Surfel;
    typedef ImplicitBall<Z3i::Space> ImplicitShape;
    typedef GaussDigitizer<Z3i::Space, ImplicitShape> DigitalShape;
    typedef LightImplicitDigitalSurface<Z3i::KSpace,DigitalShape> Boundary;
    typedef Boundary::SurfelConstIterator ConstIterator;

    double h = 0.2;
    double re = 3.0;
    double radius = 5.0;

    ImplicitShape* ishape = new ImplicitShape( RealPoint( 0, 0, 0 ), radius );
    DigitalShape* dshape = new DigitalShape();
    dshape->attach( *ishape );
    dshape->init( RealPoint( -10.0, -10.0, -10.0 ), RealPoint( 10.0, 10.0, 10.0 ), h );

    Z3i::KSpace K;
    if ( !K.init( dshape->getLowerBound(), dshape->getUpperBound(), true ) )
    {
        std::cout << "Problem" << std::endl;
        return false;
    }

    Surfel bel = Surfaces<Z3i::KSpace>::findABel( K, *dshape, 10000 );
    Boundary boundary( K, *dshape, SurfelAdjacency<Z3i::KSpace::dimension>( true ), bel );

//    std::vector< int > trololo = {1,2,3,4};
//    trololo.erase(trololo.begin()+1);
//    std::cout << trololo[0] << " " << trololo[1] << " " << trololo[2] << std::endl;
//    std::cout << "trololo memory size " << trololo.capacity() * sizeof(int) << std::endl;
//    std::cout << "trololo memory size " << trololo.size() * sizeof(int) << std::endl;
//    return false;
    std::cout << "euclidean shape memory size " << sizeof(*ishape) << std::endl;
    std::cout << "digital shape memory size " << sizeof(*dshape) << std::endl;
    std::cout << "KSpace memory size " << sizeof(K) << std::endl;
    std::cout << "boundary memory size " << sizeof(boundary) << std::endl;

    std::cout << "----------------------------------" << std::endl;

    typedef PointFunctorFromPointPredicateAndDomain< DigitalShape, Z3i::Domain, unsigned int > MyPointFunctor;
    typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
    typedef IntegralInvariantGaussianCurvatureEstimator_0memory< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator_0memory; // Gaussian curvature estimator
    typedef IntegralInvariantGaussianCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator

    MyPointFunctor pointFunctor( dshape, dshape->getDomain(), 1, 0 );
    MyCellFunctor functor ( pointFunctor, K );

    std::cout << "STEP 0" << std::endl;
    MyCurvatureEstimator_0memory estimator_0mem ( K, functor );
    estimator_0mem.init( h, re );
    std::cout << "STEP 1" << std::endl;

    string filename = "toto_g.dat";//std::tmpnam(nullptr);
    std::ofstream file( filename.c_str() );
    file.flags( std::ios_base::unitbuf );
    std::ostream_iterator< double > out_it( file, "\n" );
//    std::cout << filename << std::endl;

    estimator_0mem.eval( boundary.begin(), boundary.end(), out_it, *ishape );

    file.close();
    std::cout << "STEP 2" << std::endl;

    /*MyCurvatureEstimator estimator ( K, functor );
    estimator.init( h, re );

    std::vector< double > resultII;
    std::back_insert_iterator< std::vector< double > > resultIIIterator( resultII );
    estimator.eval( boundary.begin(), boundary.end(), resultIIIterator, *ishape );
*/

    delete ishape;
    delete dshape;
    ishape = NULL;
    dshape = NULL;

    return 0;

    /*DigitalShape* dshape = new DigitalShape();
    dshape->attach( *ishape );
    dshape->init( RealPoint( p1 ), RealPoint( p2 ), step );
    Z3i::Domain domain = dshape->getDomain();

    Z3i::KSpace K;

    bool space_ok = K.init( domain.lowerBound(),
                            domain.upperBound(), true
                            );
    if ( !space_ok )
    {
        trace.error() << "Error in the Khamisky space construction." << std::endl;
        return -2;
    }

    typedef SurfelAdjacency< Z3i::KSpace::dimension > MySurfelAdjacency;
    MySurfelAdjacency surfAdj( true ); // interior in all directions.


    typedef Z3i::KSpace::SurfelSet SurfelSet;
    typedef SetOfSurfels< Z3i::KSpace, SurfelSet > MySetOfSurfels;
    typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;


    MySetOfSurfels theSetOfSurfels( K, surfAdj );
    Surfel bel = Surfaces< Z3i::KSpace >::findABel( K, *dshape, 100000 );
    Surfaces< Z3i::KSpace >::trackBoundary( theSetOfSurfels.surfelSet(),
                                            K, surfAdj,
                                            *dshape, bel );

    MyDigitalSurface digSurf( theSetOfSurfels );


    QApplication application( argc, argv );
    Viewer3D viewer;
    viewer.show();
    viewer << SetMode3D( domain.className(), "BoundingBox" ) << domain;

    typedef PointFunctorFromPointPredicateAndDomain< DigitalShape, Z3i::Domain, unsigned int > MyPointFunctor;
    typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
    typedef IntegralInvariantGaussianCurvatureEstimator_0memory< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator

    MyPointFunctor pointFunctor( dshape, domain, 1, 0 );
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
    //  SCellToMidPoint< KSpace > midpoint( K );
    NearestPointEmbedder< Z3i::KSpace, ImplicitShape > ScellToRealPoint;
    ScellToRealPoint.init( K, step, *ishape );

    std::back_insert_iterator< std::vector< double > > resultIIIterator( resultII );
    iigaussest.eval( theSetOfSurfels.begin(), theSetOfSurfels.end(), resultIIIterator, *ishape );

    int p = 0;
    for ( std::set< Z3i::SCell >::iterator it = theSetOfSurfels.begin(), it_end = theSetOfSurfels.end();
          it != it_end; ++it, ++p)
    {

        //    RealPoint A = midpoint( *it ) * step;
        //    A = ishape.nearestPoint (A, 0.01 * step, 200, 0.1 * step);
        //    double a = ishape.meanCurvature( A );
        RealPoint A = ScellToRealPoint( *it );
        double a = 0.04;//ishape->gaussianCurvature( A );
        double b = resultII[ p ];
//        std::cout << b << std::endl;
        resultTrue.push_back( a );
        nearestPoints.push_back( A );

        //      if ( boost::math::isnan( a ))
        //      {
        //        a = 0;
        //      }

        double Linf = std::abs ( a - b );
        if ( b > maxCurv )
        {
            maxCurv = b;
        }
        else if ( b < minCurv )
        {
            minCurv = b;
        }

    }

    trace.info() << " Min = " << minCurv << std::endl;
    trace.info() << " Max = " << maxCurv << std::endl;


    //-----------------------------------------------------------------------
    //Specifing a color map

    GradientColorMap< double > cmap_grad( minCurv, maxCurv );
//    cmap_grad.addColor( Color::Blue );
//    cmap_grad.addColor( Color::White );
//    cmap_grad.addColor( Color::Red );
      cmap_grad.addColor( Color( 50, 50, 255 ) );
      cmap_grad.addColor( Color( 255, 0, 0 ) );
      cmap_grad.addColor( Color( 255, 255, 10 ) );

    //------------------------------------------------------------------------------------
    //drawing
    unsigned int nbSurfels = 0;

    //  ofstream out( "rounded2.off" );
    int i = 0;
    for ( std::set<Z3i::SCell>::iterator it = theSetOfSurfels.begin(),
          it_end = theSetOfSurfels.end();
          it != it_end; ++it, ++nbSurfels, ++i )
    {
        double a = resultTrue[ i ];
        double b = resultII[ i ];
        double Linf = std::abs ( a - b );

        std::cout << b << std::endl;

        viewer << CustomColors3D( Color::Black, cmap_grad( b ));
        viewer << *it;
    }

    viewer << Viewer3D::updateDisplay;

    return application.exec();*/
}

int testII3D_Mean()
{
    typedef Z3i::Space::RealPoint RealPoint;
    typedef Z3i::Point Point;
    typedef Z3i::KSpace::Surfel Surfel;

    typedef Z3i::Space::RealPoint::Coordinate Ring;
    typedef MPolynomial< 3, Ring > Polynomial3;
    typedef MPolynomialReader<3, Ring> Polynomial3Reader;
    typedef ImplicitPolynomial3Shape<Z3i::Space> ImplicitShape;

//    typedef ImplicitBall<Z3i::Space> ImplicitShape;
    typedef GaussDigitizer<Z3i::Space, ImplicitShape> DigitalShape;
    typedef LightImplicitDigitalSurface<Z3i::KSpace,DigitalShape> Boundary;
    typedef Boundary::SurfelConstIterator ConstIterator;
    typedef Boundary::Tracker Tracker;


    typedef DigitalShape::PointEmbedder DigitalEmbedder;

    double h = 0.2;
    double re = 3.0;
    double radius = 5.0;

    ///////////////////


    std::string poly_str = "x^2 + y^2 + z^2 - 25";
    Polynomial3 poly;
    Polynomial3Reader reader;
    std::string::const_iterator iter = reader.read( poly, poly_str.begin(), poly_str.end() );
    if ( iter != poly_str.end() )
    {
        std::cerr << "ERROR: I read only <"
                  << poly_str.substr( 0, iter - poly_str.begin() )
                  << ">, and I built P=" << poly << std::endl;
        return 1;
    }

    ImplicitShape* ishape = new ImplicitShape( poly );
    ///////////////////

//    ImplicitShape* ishape = new ImplicitShape( RealPoint( 0, 0, 0 ), radius );
    DigitalShape* dshape = new DigitalShape();
    dshape->attach( *ishape );
    dshape->init( RealPoint( -10.0, -10.0, -10.0 ), RealPoint( 10.0, 10.0, 10.0 ), h );

    Z3i::KSpace K;
    if ( !K.init( dshape->getLowerBound(), dshape->getUpperBound(), true ) )
    {
        std::cout << "Problem" << std::endl;
        return false;
    }

    Surfel bel = Surfaces<Z3i::KSpace>::findABel( K, *dshape, 10000 );
    Boundary boundary( K, *dshape, SurfelAdjacency<Z3i::KSpace::dimension>( true ), bel );

    typedef PointFunctorFromPointPredicateAndDomain< DigitalShape, Z3i::Domain, unsigned int > MyPointFunctor;
    typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
    typedef IntegralInvariantMeanCurvatureEstimator_0memory< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator_0memory; // Mean curvature estimator
    typedef IntegralInvariantMeanCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Mean curvature estimator

    MyPointFunctor pointFunctor( dshape, dshape->getDomain(), 1, 0 );
    MyCellFunctor functor ( pointFunctor, K );

    std::cout << "STEP 0" << std::endl;
    MyCurvatureEstimator_0memory estimator_0mem ( K, functor );
    estimator_0mem.init( h, re );
    std::cout << "STEP 1" << std::endl;

    string filename = "toto_m.dat";//std::tmpnam(nullptr);
    std::ofstream file( filename.c_str() );
    file.flags( std::ios_base::unitbuf );
    std::ostream_iterator< double > out_it( file, "\n" );
//    std::cout << filename << std::endl;

    estimator_0mem.eval( boundary.begin(), boundary.end(), out_it, *ishape );

    file.close();
    std::cout << "STEP 2" << std::endl;

    delete ishape;
    delete dshape;
    ishape = NULL;
    dshape = NULL;

    return 0;
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

//    testII2D_Gaussian( );
//    testII3D_Gaussian( );//argc, argv );
    testII3D_Mean();
    trace.endBlock();

    return 1;
    //  bool res = testII3D();
    //  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
    //  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
