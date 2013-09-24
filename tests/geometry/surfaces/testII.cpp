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
#include "DGtal/geometry/volumes/KanungoNoise.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantGaussianCurvatureEstimator.h"
#include "DGtal/geometry/surfaces/estimation/IntegralInvariantMeanCurvatureEstimator.h"

#include "DGtal/kernel/BasicPointFunctors.h"

#include "DGtal/shapes/implicit/ImplicitBall.h"
#include "DGtal/shapes/ShapeFactory.h"

#include "DGtal/io/boards/Board2D.h"
#include "DGtal/shapes/DigitalShapesDecorator.h"

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

//#define EUCLIDEAN
#define EXPORT


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

template < typename TSpace, typename Point >
typename TSpace::RealPoint RegularPointEmbedder_( const Point & p, const double & h )
{
    typedef typename TSpace::Integer Integer;
    typename TSpace::RealPoint aRealPoint;
    for ( Dimension i = 0; i < TSpace::dimension; ++i )
        aRealPoint[ i ] = NumberTraits<Integer>::castToDouble( p[ i ] ) * h;
    return aRealPoint;
}

bool test2DTopology()
{
    typedef Z2i::RealPoint RealPoint;
    typedef Z2i::Space Space;
    typedef Z2i::Domain Domain;
    typedef typename Space::Point Point;
    typedef typename Space::Vector Vector;
//    typedef AccFlower2D< Space > Shape;
    typedef Ball2D< Space > Shape;

    typedef GaussDigitizer< Space, Shape > Digitizer;

    typedef Z2i::KSpace KSpace;
    typedef typename KSpace::SCell SCell;

    typedef typename GridCurve< KSpace >::PointsRange PointsRange;
    typedef typename GridCurve< KSpace >::MidPointsRange MidPointsRange;

    typedef LightImplicitDigitalSurface< KSpace, Digitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > DigSurface;

    typedef DepthFirstVisitor< DigSurface > Visitor;
    typedef GraphVisitorRange< Visitor > VisitorRange;
    typedef typename VisitorRange::ConstIterator SurfelConstIterator;


    RealPoint center( 0.0, 0.0 );
    double radius = 20.0;
    double varsmallradius = 5.0;
    int k = 3;
    double phi = 0.2;

    double h = 1;

//    Shape * aShape = new Shape( center, radius, varsmallradius, k, phi );
    Shape * aShape = new Shape( center, radius );
    Digitizer * digShape = new Digitizer();
    digShape->attach( *aShape );
    Vector vlow( -1, -1 );
    Vector vup( 1, 1 );
    digShape->init( aShape->getLowerBound() + vlow, aShape->getUpperBound() + vup, h );
    Domain domain = digShape->getDomain();

    KSpace K;
    K.init( digShape->getLowerBound(), digShape->getUpperBound(), true );
    SCell bel = Surfaces< KSpace >::findABel( K, *digShape, 10000 );
    SurfelAdjacency< KSpace::dimension > SAdj( true );

    std::vector< Point > points;
    Surfaces< KSpace >::track2DBoundaryPoints( points, K, SAdj, *digShape, bel );
    std::cout << "points: " << points.size() << std::endl;
    std::cout << points[0] << std::endl;
    std::cout << points[1] << std::endl;

#ifdef EXPORT
    Color white( 255, 255, 255, 255 );
    Color black( 0, 0, 0, 255 );
    Color red( 255, 0, 0, 255 );
    Color green( 0, 255, 0, 255 );
    Color blue( 0, 0, 255, 255 );
    Color dwhite( 192, 192, 192, 255 );
    Color dblack( 0, 0, 0, 255 );
    Color dred( 192, 0, 0, 255 );
    Color dgreen( 0, 192, 0, 255 );
    Color dblue( 0, 0, 192, 255 );
    Board2D board;
    board << SetMode( domain.className(), "Grid" )
          << domain;
    Point origin(0,0);
    for( Domain::ConstIterator ibegin = domain.begin(), iend = domain.end(); ibegin != iend; ++ibegin )
    {
        if( digShape->operator ()( *ibegin ) )
        {
            board << CustomStyle( origin.className(), new CustomColors( black, dblack ) )
                  << *ibegin;
        }
    }
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
    board.saveSVG("testII_topology_full.svg");

    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
    for( int i = 0; i < points.size(); ++i )
    {
        board << CustomStyle( points[ i ].className(), new CustomColors( blue, dblue ) )
              << points[ i ];
    }
    board.saveSVG("testII_topology_points.svg");
#endif
    std::cout << "oooooooooooooooooooooo" << std::endl;

    std::vector< SCell > pointsScell;
    Surfaces< KSpace >::track2DBoundary( pointsScell, K, SAdj, *digShape, bel );
    GridCurve< KSpace > gridcurve2;
    gridcurve2.initFromSCellsVector( pointsScell );
    MidPointsRange pointsRange2 = gridcurve2.getMidPointsRange();

#ifdef EXPORT
    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
    Point last;
    int recount2 = 0;
    for( MidPointsRange::ConstIterator ibegin = pointsRange2.begin(), iend = pointsRange2.end(); ibegin != iend; ++ibegin )
    {
        if( ibegin != pointsRange2.begin() )
        {
            if( last == *ibegin )
            {
                recount2++;
            }
        }
        board << CustomStyle( (*ibegin).className(), new CustomColors( blue, dblue ) )
              << *ibegin;

        last = *ibegin;
    }
    std::cout << "RECOUNT2 " << recount2 << std::endl;
    board.saveSVG("testII_topology_pointRange2.svg");

    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
    for( int i = 0; i < points.size() - 1; ++i )
    {
        board << CustomStyle( pointsScell[ i ].className(), new CustomColors( blue, dblue ) )
              << pointsScell[ i ];
    }
    board.saveSVG("testII_topology_pointsScell.svg");

    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
    for( int i = 0; i < points.size() - 1; ++i )
    {
        board << CustomStyle( pointsScell[ i ].className(), new CustomColors( blue, dblue ) )
              << pointsScell[ i ];
    }
    board.saveSVG("testII_topology_pointsRange2.svg");
#endif
    std::cout << "pointsScell: " << pointsScell.size() << std::endl;
    std::cout << "oooooooooooooooooooooo" << std::endl;

    std::cout << "pointsRange2: " << pointsRange2.size() << std::endl;
    std::cout << "oooooooooooooooooooooo" << std::endl;

    GridCurve< KSpace > gridcurve;
    gridcurve.initFromVector( points );
    PointsRange pointsRange = gridcurve.getPointsRange();
    std::cout << "pointsRange: " << pointsRange.size() << std::endl;
    PointsRange::ConstIterator prbegin = pointsRange.begin();
    std::cout << *prbegin << std::endl;
    ++prbegin;
    std::cout << *prbegin << std::endl;
    std::cout << "oooooooooooooooooooooo" << std::endl;

    LightImplicitDigSurface LightImplDigSurf( K, *digShape, SAdj, bel );
    DigSurface surf( LightImplDigSurf );
    std::cout << "surf: " << surf.size() << std::endl;
    DigSurface::ConstIterator sbegin = surf.begin();
    std::cout << *sbegin << std::endl;
    ++sbegin;
    std::cout << *sbegin << std::endl;
    std::cout << "oooooooooooooooooooooo" << std::endl;

    int count = 0;
    VisitorRange range( new Visitor( surf, *surf.begin() ) );
#ifdef EXPORT
    board.clear();
    board << SetMode( domain.className(), "Grid" )
          << domain;
    board << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;

    Board2D board2;
    board2 << SetMode( domain.className(), "Grid" )
           << domain;
    board2 << CustomStyle( origin.className(), new CustomColors( red, dred ) )
          << origin;
#endif
    for( SurfelConstIterator ibegin = range.begin(), iend = range.end(); ibegin != iend; ++ibegin )
    {
        Dimension track = *( K.sDirs( *ibegin ) );
        SCell pointel = K.sIndirectIncident( *ibegin, track );
        Point current = K.sCoords( pointel );
#ifdef EXPORT
        board << CustomStyle( current.className(), new CustomColors( green, dgreen ) )
              << current;
        board2 << CustomStyle( (*ibegin).className(), new CustomColors( black, dblack ) )
              << (*ibegin);
#endif
        if( count == 0 || count == 1 )
            std::cout << *ibegin << std::endl;

        ++count;
    }
#ifdef EXPORT
    board.saveSVG("testII_topology_surfel.svg");
    board2.saveSVG("testII_topology_surfel2.svg");
#endif
    std::cout << "range: " << count << std::endl;

    delete digShape;
    delete aShape;
}

bool testII2D_kernels()
{
    typedef ImplicitBall< Z2i::Space > ShapeA;
    typedef ImplicitBall< Z2i::Space > ShapeB;
    typedef ImplicitBall< Z2i::Space > ShapeC;
    typedef GaussDigitizer< Z2i::Space, ShapeA > DigitalShapeA;
    typedef GaussDigitizer< Z2i::Space, ShapeB > DigitalShapeB;
    typedef GaussDigitizer< Z2i::Space, ShapeC > DigitalShapeC;
    typedef EuclideanShapesMinus< ShapeA, ShapeB > EuclideanMinusShapeAB;
    typedef EuclideanShapesMinus< ShapeB, ShapeC > EuclideanMinusShapeBC;
    typedef DigitalShapesMinus< DigitalShapeA, DigitalShapeB > DigitalDigitalMinusShapeAB;
    typedef DigitalShapesMinus< DigitalShapeB, DigitalShapeC > DigitalDigitalMinusShapeBC;
    typedef GaussDigitizer< Z2i::Space, EuclideanMinusShapeAB > DigitalEuclideanMinusShapeAB;
    typedef GaussDigitizer< Z2i::Space, EuclideanMinusShapeBC > DigitalEuclideanMinusShapeBC;
    typedef Z2i::DigitalSet DigitalSetA;
    typedef Z2i::DigitalSet DigitalSetB;
    typedef Z2i::DigitalSet DigitalSetC;
    typedef Z2i::DigitalSet DigitalSetDigitalMinusAB;
    typedef Z2i::DigitalSet DigitalSetDigitalMinusBC;
    typedef Z2i::DigitalSet DigitalSetEuclideanMinusAB;
    typedef Z2i::DigitalSet DigitalSetEuclideanMinusBC;

    typedef Z2i::Space Space;
    typedef Z2i::Domain Domain;
    typedef Domain::Point Point;
    typedef Space::RealPoint RealPoint;

    double radius = 5.0;
    double h = 0.1;

    Point pOrigin( 0, 0 );

    RealPoint rOriginA( 0, 0 );
    Point pOriginA( 0, 0 );
    ShapeA shapeA( rOriginA, radius );

    RealPoint rOriginB( h, h );
    Point pOriginB( 1, 1 );
    ShapeA shapeB( rOriginB, radius );

    RealPoint rOriginC( 2.0 * h, 2.0 * h );
    Point pOriginC( 2, 2 );
    ShapeC shapeC( rOriginC, radius );

    //////// Euclidean

    EuclideanMinusShapeAB euclideanMinusShapeAB( shapeA, shapeB );
    EuclideanMinusShapeAB euclideanMinusShapeBC( shapeB, shapeC );

    RealPoint rBug = RegularPointEmbedder_< Space, Point >( Point( -47, -13 ), h );
    RealPoint rNormalInside1 = RegularPointEmbedder_< Space, Point >( Point( -47, -14 ), h );
    RealPoint rNormalInside2 = RegularPointEmbedder_< Space, Point >( Point( -48, -13 ), h );

    std::cout << " //////////// Euclidean " << std::endl;

    std::cout << "Bug:(" << rBug << ") - shapeA.orientation:" << shapeA.orientation( rBug ) << " - shapeB.orientation:" << shapeB.orientation( rBug ) << " - minus.orientation:" << euclideanMinusShapeAB.orientation( rBug ) << std::endl;
    std::cout << "Bug:(" << rNormalInside1 << ") - shapeA.orientation:" << shapeA.orientation( rNormalInside1 ) << " - shapeB.orientation:" << shapeB.orientation( rNormalInside1 ) << " - minus.orientation:" << euclideanMinusShapeAB.orientation( rNormalInside1 ) << std::endl;
    std::cout << "Bug:(" << rNormalInside2 << ") - shapeA.orientation:" << shapeA.orientation( rNormalInside2 ) << " - shapeB.orientation:" << shapeB.orientation( rNormalInside2 ) << " - minus.orientation:" << euclideanMinusShapeAB.orientation( rNormalInside2 ) << std::endl;

    //////// Digital

    DigitalShapeA digitalShapeA;
    digitalShapeA.attach( shapeA );
    digitalShapeA.init( shapeA.getLowerBound(), shapeA.getUpperBound(), h );

    DigitalShapeB digitalShapeB;
    digitalShapeB.attach( shapeB );
    digitalShapeB.init( shapeB.getLowerBound(), shapeB.getUpperBound(), h );

    DigitalShapeC digitalShapeC;
    digitalShapeC.attach( shapeC );
    digitalShapeC.init( shapeC.getLowerBound(), shapeC.getUpperBound(), h );

    DigitalDigitalMinusShapeAB digitalDigitalMinusShapeAB( digitalShapeA, digitalShapeB );
    DigitalDigitalMinusShapeBC digitalDigitalMinusShapeBC( digitalShapeB, digitalShapeC );

    Point pBug( -47, -13 );
    Point pNormalInside1( -47, -14 );
    Point pNormalInside2( -48, -13 );

    std::cout << " //////////// Digital " << std::endl;

    std::cout << "Bug:(" << pBug << ") - digitalShapeA.orientation:" << digitalShapeA.orientation( pBug ) << " - digitalShapeB.orientation:" << digitalShapeB.orientation( pBug ) << " - minus.orientation:" << digitalDigitalMinusShapeAB.orientation( pBug ) << std::endl;
    std::cout << "Bug:(" << pNormalInside1 << ") - digitalShapeA.orientation:" << digitalShapeA.orientation( pNormalInside1 ) << " - digitalShapeB.orientation:" << digitalShapeB.orientation( pNormalInside1 ) << " - minus.orientation:" << digitalDigitalMinusShapeAB.orientation( pNormalInside1 ) << std::endl;
    std::cout << "Bug:(" << pNormalInside2 << ") - digitalShapeA.orientation:" << digitalShapeA.orientation( pNormalInside2 ) << " - digitalShapeB.orientation:" << digitalShapeB.orientation( pNormalInside2 ) << " - minus.orientation:" << digitalDigitalMinusShapeAB.orientation( pNormalInside2 ) << std::endl;

    std::cout << "Hum ... " << shapeA.orientation( RealPoint( -5.0, 0.0 ) ) << " " << shapeB.orientation( RealPoint( -5.0+h, h ) ) << " " << shapeC.orientation( RealPoint( -5.0+2*h, 2*h ) ) << std::endl;
    std::cout << "Hum ... " << digitalShapeA.orientation( Point( -50, 0 ) ) << " " << digitalShapeB.orientation( Point( -49, 1 ) ) << " " << digitalShapeC.orientation( Point( -48, 2 ) ) << std::endl;

    std::cout << RegularPointEmbedder_<Space, Point>( Point( -48, 2 ), h ) << " instead of " << RealPoint( -5.0+2*h, 2*h ) << std::endl;
    std::cout << RegularPointEmbedder_<Space, Point>( Point( -48, 2 ), h ) << " -> " << shapeC.orientation( RegularPointEmbedder_<Space, Point>( Point( -48, 2 ), h ) ) << std::endl;
    std::cout << RealPoint( -5.0+2*h, 2*h ) << " -> " << shapeC.orientation( RealPoint( -5.0+2*h, 2*h ) ) << std::endl;

#ifdef EXPORT
    ///////// Export

    Color white( 255, 255, 255, 255 );
    Color black( 0, 0, 0, 255 );
    Color red( 255, 0, 0, 255 );
    Color green( 0, 255, 0, 255 );
    Color blue( 0, 0, 255, 255 );
    Color dwhite( 192, 192, 192, 255 );
    Color dblack( 0, 0, 0, 255 );
    Color dred( 192, 0, 0, 255 );
    Color dgreen( 0, 192, 0, 255 );
    Color dblue( 0, 0, 192, 255 );

    DigitalSetA setA( digitalShapeA.getDomain() );
    Shapes< Domain >::digitalShaper ( setA, digitalShapeA );

    DigitalSetB setB( digitalShapeB.getDomain() );
    Shapes< Domain >::digitalShaper ( setB, digitalShapeB );

    DigitalSetB setC( digitalShapeC.getDomain() );
    Shapes< Domain >::digitalShaper ( setC, digitalShapeC );

    DigitalSetDigitalMinusAB setDigitalMinusAB( digitalShapeA.getDomain() );
    Shapes< Domain >::digitalShaper ( setDigitalMinusAB, digitalDigitalMinusShapeAB );

    DigitalSetDigitalMinusBC setDigitalMinusBC( digitalShapeB.getDomain() );
    Shapes< Domain >::digitalShaper ( setDigitalMinusBC, digitalDigitalMinusShapeBC );

    DigitalEuclideanMinusShapeAB digitalEuclideanMinusShapeAB;
    digitalEuclideanMinusShapeAB.attach( euclideanMinusShapeAB );
    digitalEuclideanMinusShapeAB.init( digitalShapeA.getLowerBound(), digitalShapeB.getUpperBound(), h );
    DigitalSetEuclideanMinusAB setEuclideanMinusAB( digitalShapeA.getDomain() );
    Shapes< Domain >::digitalShaper ( setEuclideanMinusAB, digitalEuclideanMinusShapeAB );

    DigitalEuclideanMinusShapeBC digitalEuclideanMinusShapeBC;
    digitalEuclideanMinusShapeBC.attach( euclideanMinusShapeBC );
    digitalEuclideanMinusShapeBC.init( digitalShapeB.getLowerBound(), digitalShapeC.getUpperBound(), h );
    DigitalSetEuclideanMinusBC setEuclideanMinusBC( digitalShapeB.getDomain() );
    Shapes< Domain >::digitalShaper ( setEuclideanMinusBC, digitalEuclideanMinusShapeBC );

    Board2D board;
    board << SetMode( digitalShapeA.getDomain().className(), "Grid" )
          << digitalShapeA.getDomain();

    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
          << pOrigin;

    for( DigitalSetA::ConstIterator it = setA.begin(), itend = setA.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( black, dblack ) );
        board << *it;
    }

    for( DigitalSetB::ConstIterator it = setB.begin(), itend = setB.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( white, dwhite ) );
        board << *it;
    }

    for( DigitalSetC::ConstIterator it = setC.begin(), itend = setC.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( black, dblack ) );
        board << *it;
    }

    board.saveSVG("testII_old_new.svg");

    board.clear();
    board << SetMode( digitalShapeA.getDomain().className(), "Grid" )
          << digitalShapeA.getDomain();

    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
          << pOrigin;

    for( DigitalSetEuclideanMinusAB::ConstIterator it = setEuclideanMinusAB.begin(), itend = setEuclideanMinusAB.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
        board << *it;
    }

    board.saveSVG("testII_euclideanMinus_AB.svg");

    board.clear();
    board << SetMode( digitalShapeA.getDomain().className(), "Grid" )
          << digitalShapeA.getDomain();

    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
          << pOrigin;

    for( DigitalSetDigitalMinusAB::ConstIterator it = setDigitalMinusAB.begin(), itend = setDigitalMinusAB.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( green, dgreen ) );
        board << *it;
    }

    board.saveSVG("testII_digitalMinus_AB.svg");

    board.clear();
    board << SetMode( digitalShapeB.getDomain().className(), "Grid" )
          << digitalShapeB.getDomain();

    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
          << pOrigin;

    for( DigitalSetEuclideanMinusBC::ConstIterator it = setEuclideanMinusBC.begin(), itend = setEuclideanMinusBC.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
        board << *it;
    }

    board.saveSVG("testII_euclideanMinus_BC.svg");

    board.clear();
    board << SetMode( digitalShapeB.getDomain().className(), "Grid" )
          << digitalShapeB.getDomain();

    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
          << pOrigin;

    for( DigitalSetDigitalMinusBC::ConstIterator it = setDigitalMinusBC.begin(), itend = setDigitalMinusBC.end(); it != itend; ++it )
    {
        board << CustomStyle( pOrigin.className(), new CustomColors( green, dgreen ) );
        board << *it;
    }

    board.saveSVG("testII_digitalMinus_BC.svg");


    if( setEuclideanMinusAB.size() != setEuclideanMinusBC.size() )
    {
        trace.error() << " ------------------------------------------ " << std::endl;
        trace.error() << " setEuclideanMinusAB != setEuclideanMinusBC " << std::endl;
        trace.error() << " setEuclideanMinusAB = " << setEuclideanMinusAB.size() << std::endl;
        trace.error() << " setEuclideanMinusBC = " << setEuclideanMinusBC.size() << std::endl;
        trace.error() << " ------------------------------------------ " << std::endl;
    }
    if( setDigitalMinusAB.size() != setDigitalMinusBC.size() )
    {
        trace.error() << " -------------------------------------- " << std::endl;
        trace.error() << " setDigitalMinusAB != setDigitalMinusBC " << std::endl;
        trace.error() << " setDigitalMinusAB = " << setDigitalMinusAB.size() << std::endl;
        trace.error() << " setDigitalMinusBC = " << setDigitalMinusBC.size() << std::endl;
        trace.error() << " -------------------------------------- " << std::endl;
    }

#endif
    return true;
}

bool testII2D_kernels_2()
{
    typedef Z2i::KSpace KSpace;
    typedef Z2i::Domain Domain;
    typedef KSpace::Space::RealPoint RealPoint;
    typedef ImplicitBall< Z2i::Space > KernelSupport;
    typedef KSpace::SurfelSet SurfelSet;
    typedef Z2i::DigitalSet DigitalSet;
    typedef Domain::Point Point;


    //////////////////////////////

    double radius = 5.0;
    double h = 0.1;
    RealPoint rOrigin = RealPoint ( 0.0, 0.0 );
    Point pOrigin = Point( 0, 0 );
    KernelSupport kernel( rOrigin, radius );
    KSpace KSpaceKernel;

    std::vector< SurfelSet > kernels = std::vector< SurfelSet > ( 9 );
    typedef GaussDigitizer< Z2i::Space, KernelSupport > DigitalShape;

#ifdef EUCLIDEAN
    typedef EuclideanShapesMinus< KernelSupport, KernelSupport > MinusShape;
#else
    typedef DigitalShapesMinus< DigitalShape, DigitalShape > MinusShape;
#endif

    DigitalShape digKernel;
    digKernel.attach( kernel );
    digKernel.init( kernel.getLowerBound() - Domain::Point( 1, 1 ), kernel.getUpperBound() + Domain::Point( 1, 1 ), h );

    Domain domainKernel = digKernel.getDomain();
    DigitalSet setKernel( domainKernel );
    Shapes< Domain >::digitalShaper ( setKernel, digKernel );

    bool space_ok = KSpaceKernel.init( domainKernel.lowerBound(), domainKernel.upperBound(), true );
    if( !space_ok )
    {
        trace.error() << "Error in the Khalimsky space construction." << std::endl;
        return false;
    }

    Color white( 255, 255, 255, 255 );
    Color black( 0, 0, 0, 255 );
    Color red( 255, 0, 0, 255 );
    Color green( 0, 255, 0, 255 );
    Color blue( 0, 0, 255, 255 );
    Color dwhite( 192, 192, 192, 255 );
    Color dblack( 0, 0, 0, 255 );
    Color dred( 192, 0, 0, 255 );
    Color dgreen( 0, 192, 0, 255 );
    Color dblue( 0, 0, 192, 255 );

    Board2D board;
    board << SetMode( domainKernel.className(), "Grid" )
          << domainKernel;

    for( int y = -1; y <= 1; ++y )
    {
        for( int x = -1; x <= 1; ++x )
        {
            RealPoint shiftPoint = rOrigin + RealPoint( x * h, y * h );
            KernelSupport kernelShifted( shiftPoint, radius );
            DigitalShape digCurrentKernel;
            digCurrentKernel.attach( kernelShifted );
            digCurrentKernel.init( kernel.getLowerBound(), kernel.getUpperBound(), h );

            DigitalSet setCurrentKernelB( domainKernel );
            Shapes< Domain >::digitalShaper ( setCurrentKernelB, digCurrentKernel );

            int offset = ( x + 1 ) + (( y + 1 ) * 3 );
            if( offset == 4 ) // Full kernel
            {

                DigitalSet setCurrentKernel( domainKernel );
                Shapes< Domain >::digitalShaper ( setCurrentKernel, digCurrentKernel );

                board.clear();
                board << SetMode( domainKernel.className(), "Grid" )
                      << domainKernel;
                board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
                      << pOrigin;
                /*for( auto it = setCurrentKernel.begin(), itend = setCurrentKernel.end();
                     it != itend;
                     ++it  )
                {
                    //                                        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) - Point( 2, 2 ) + shiftNewSpel );
                    board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
                    board << *it;
                }*/

                if (setCurrentKernel.find(Point(-47,-13)) == setCurrentKernel.end())
                    trace.error()<<" Absent - completA"<<std::endl;
                else
                    trace.error()<<" inside/on - completA"<<std::endl;


                //                board.saveSVG("testII_decallage_old.svg");

                int count2 = 0;
                /*for( auto it = setCurrentKernel.begin(), itend = setCurrentKernel.end();
                     it != itend;
                     ++it )
                {
                    //                    board << *it;
                    kernels[ offset ].insert( KSpaceKernel.sSpel( *it ));
                    ++count2;
                }*/
                continue;
            }
            else
            {
#ifdef EUCLIDEAN
                MinusShape current( kernel, kernelShifted );
                GaussDigitizer< Z2i::Space, MinusShape > digCurrentKernel;
                digCurrentKernel.attach( current );
                digCurrentKernel.init( kernel.getLowerBound(), kernel.getUpperBound(), h );
#else
                MinusShape current( digKernel, digCurrentKernel );
#endif

                DigitalSet setCurrentKernel( domainKernel );
#ifdef EUCLIDEAN
                Shapes< Domain >::digitalShaper ( setCurrentKernel, digCurrentKernel );
#else
                Shapes< Domain >::digitalShaper ( setCurrentKernel, current );
#endif

                if( offset == 8 )
                {

                    if (setCurrentKernelB.find(Point(-47,-13)) == setCurrentKernelB.end())
                        trace.error()<<" Absent - completB"<<std::endl;
                    else
                        trace.error()<<" inside/on - completB"<<std::endl;


                    if (setCurrentKernel.find(Point(-47,-13)) == setCurrentKernel.end())
                        trace.error()<<" Absent - completM"<<std::endl;
                    else
                        trace.error()<<" inside/on - completM"<<std::endl;


#ifdef EUCLIDEAN
                    RealPoint bug = RegularPointEmbedder_<Z2i::Space, Point>( Point(-47, -13), h);
                    RealPoint notbug1 = RegularPointEmbedder_<Z2i::Space, Point>( Point(-47, -14), h);
                    RealPoint notbug2 = RegularPointEmbedder_<Z2i::Space, Point>( Point(-48, -13), h);
                    std::cout << "Bug point mask:" << current.orientation( bug ) << std::endl;
                    std::cout << "Bug point kernel:" << kernel.orientation( bug ) << std::endl;
                    std::cout << "Bug point kernelShifted:" << kernelShifted.orientation( bug ) << std::endl;
                    std::cout << "Not Bug point1 mask:" << current.orientation( notbug1 ) << std::endl;
                    std::cout << "Bug point1 kernel:" << kernel.orientation( notbug1 ) << std::endl;
                    std::cout << "Bug point1 kernelShifted:" << kernelShifted.orientation( notbug1 ) << std::endl;
                    std::cout << "Not Bug point2 mask:" << current.orientation( notbug2 ) << std::endl;
                    std::cout << "Bug point2 kernel:" << kernel.orientation( notbug2 ) << std::endl;
                    std::cout << "Bug point2 kernelShifted:" << kernelShifted.orientation( notbug2 ) << std::endl;

#else
                    Point bug(-47,-13);
                    Point notbug1( -47, -14);
                    Point notbug2( -48, -13);
                    std::cout << "Bug point mask:" << current.orientation( bug ) << std::endl;
                    std::cout << "Bug point kernel:" << digKernel.orientation( bug ) << std::endl;
                    std::cout << "Bug point kernelShifted:" << digCurrentKernel.orientation( bug ) << std::endl;
                    std::cout << "Not Bug point1 mask:" << current.orientation( notbug1 ) << std::endl;
                    std::cout << "Bug point1 kernel:" << digKernel.orientation( notbug1 ) << std::endl;
                    std::cout << "Bug point1 kernelShifted:" << digCurrentKernel.orientation( notbug1 ) << std::endl;
                    std::cout << "Not Bug point2 mask:" << current.orientation( notbug2 ) << std::endl;
                    std::cout << "Bug point2 kernel:" << digKernel.orientation( notbug2 ) << std::endl;
                    std::cout << "Bug point2 kernelShifted:" << digCurrentKernel.orientation( notbug2 ) << std::endl;
#endif


                    //                                    if (setCurrentKernel.find(Point(-46,13)) == setCurrentKernel.end())
                    //                                        trace.error()<<" Absent"<<std::endl;

                    board.clear();
                    board << SetMode( domainKernel.className(), "Grid" )
                          << domainKernel;
                    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
                          << pOrigin;
                    /*for( auto it = setCurrentKernel.begin(), itend = setCurrentKernel.end();
                         it != itend;
                         ++it  )
                    {
                        //                                        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) - Point( 2, 2 ) + shiftNewSpel );
                        board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
                        board << *it;
                    }*/

                    //                    board.saveSVG("testII_decallage_minus.svg");
                }

                int count2 = 0;
                /*for( auto it = setCurrentKernel.begin(), itend = setCurrentKernel.end();
                     it != itend;
                     ++it )
                {
                    //                    board << *it;
                    kernels[ offset ].insert( KSpaceKernel.sSpel( *it ));
                    ++count2;
                }*/

                ASSERT(( count2 != 0 )); // Error when creating masks

            }

            //            std::stringstream ss;
            //            ss << "test_kernel_" << offset << ".svg";
            //            board.saveSVG(ss.str().c_str());
        }
    }

    /// Part to substract from previous result.

    //    typedef Z2i::KSpace::SCell Spel;
    //    Spel shiftedSpel;

    //    Point oldPos = Point( 1, 0 );
    //    Point newPos = Point( 2, 1 );

    //    Point shiftOldSpel = KSpaceKernel.sKCoords( KSpaceKernel.sSpel( oldPos ) ) - KSpaceKernel.sKCoords( KSpaceKernel.sSpel( pOrigin ) );
    //    Point shiftNewSpel = KSpaceKernel.sKCoords( KSpaceKernel.sSpel( newPos ) ) - KSpaceKernel.sKCoords( KSpaceKernel.sSpel( pOrigin ) );
    //    Point diffSpel = KSpaceKernel.sKCoords( KSpaceKernel.sSpel( newPos ) ) - KSpaceKernel.sKCoords( KSpaceKernel.sSpel( oldPos ) );

    //    int x = diffSpel[ 0 ];
    //    int y = diffSpel[ 1 ];

    //    int offset = (( x / 2 ) + 1 ) + ((( y / 2 ) + 1 ) * 3 );
    //    std::cout << offset << std::endl;

    //    board.clear();
    //    board << SetMode( domainKernel.className(), "Grid" )
    //          << domainKernel;
    //    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
    //          << pOrigin;

    //    for( SurfelSet::const_iterator current = kernels[ 4 ].begin(), end = kernels[ 4 ].end(); current != end; ++current )
    //    {
    //        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) + shiftOldSpel );
    //        board << CustomStyle( pOrigin.className(), new CustomColors( black, dblack ) );
    //        board << KSpaceKernel.sCoords( shiftedSpel );
    //    }
    //    board.saveSVG("testII_decallage_full_old.svg");

    //    board.clear();
    //    board << SetMode( domainKernel.className(), "Grid" )
    //          << domainKernel;
    //    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
    //          << pOrigin;
    //    for( SurfelSet::const_iterator current = kernels[ 4 ].begin(), end = kernels[ 4 ].end(); current != end; ++current )
    //    {
    //        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) + shiftNewSpel );
    //        board << CustomStyle( pOrigin.className(), new CustomColors( white, dwhite ) );
    //        board << KSpaceKernel.sCoords( shiftedSpel );
    //    }
    //    board.saveSVG("testII_decallage_full_new.svg");

    //    board.clear();
    //    board << SetMode( domainKernel.className(), "Grid" )
    //          << domainKernel;
    //    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
    //          << pOrigin;
    //    for( SurfelSet::const_iterator current = kernels[ offset ].begin(), end = kernels[ offset ].end(); current != end; ++current )
    //    {
    //        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) - Point( 2, 2 ) + shiftNewSpel );
    //        board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
    //        board << KSpaceKernel.sCoords( shiftedSpel );
    //    }

    //    board.saveSVG("testII_decallage_minus.svg");

    //    board.clear();
    //    board << SetMode( domainKernel.className(), "Grid" )
    //          << domainKernel;
    //    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
    //          << pOrigin;

    //    /// Part to add from previous result.
    //    for( SurfelSet::const_iterator current = kernels[ 8 - offset ].begin(), end = kernels[ 8 - offset ].end(); current != end; ++current )
    //    {
    //        KSpaceKernel.sSetKCoords( shiftedSpel, KSpaceKernel.sKCoords( *current ) + shiftNewSpel );
    //        board << CustomStyle( pOrigin.className(), new CustomColors( green, dgreen ) );
    //        board << KSpaceKernel.sCoords( shiftedSpel );
    //    }
    //    board.saveSVG("testII_decallage_plus.svg");

    //    board.clear();
    //    board << SetMode( domainKernel.className(), "Grid" )
    //          << domainKernel;
    //    board << CustomStyle( pOrigin.className(), new CustomColors( blue, dblue ) )
    //          << pOrigin;

    //    Point bug(-46, -13);
    //    Point notbug1( -46, -14);
    //    Point notbug2( -47, -13);
    //    board << CustomStyle( pOrigin.className(), new CustomColors( red, dred ) );
    //    board << bug;
    //    board << notbug1;
    //    board << notbug2;
    //    board.saveSVG("testII_decallage_bug.svg");
    //    //    board.saveSVG("testII_decallage.svg");
    //    //////////////////////////////

    return true;
}

int testII3D_kernels( int argc, char** argv )
{
    typedef Z3i::KSpace KSpace;
    typedef Z3i::Domain Domain;
    typedef KSpace::Space::RealPoint RealPoint;
    typedef ImplicitBall< Z3i::Space > KernelSupport;
    typedef KSpace::SurfelSet SurfelSet;
    typedef Z3i::DigitalSet DigitalSet;


    //////////////////////////////

    QApplication application( argc, argv );
    Viewer3D viewer;

    double radius = 5.0;
    double h = 0.1;
    RealPoint rOrigin = RealPoint ( 0.0, 0.0, 0.0 );
    KernelSupport kernel( rOrigin, radius );
    KSpace KSpaceKernel;

    std::vector< SurfelSet > kernels = std::vector< SurfelSet > ( 27 );

    typedef EuclideanShapesMinus< KernelSupport, KernelSupport > EuclideanShape;

    GaussDigitizer< Z3i::Space, KernelSupport > digKernel;
    digKernel.attach( kernel );
    digKernel.init( kernel.getLowerBound() - Domain::Point( 1, 1, 1 ), kernel.getUpperBound() + Domain::Point( 1, 1, 1 ), h );

    Domain domainKernel = digKernel.getDomain();
    DigitalSet setKernel( domainKernel );
    Shapes< Domain >::digitalShaper ( setKernel, digKernel );

    viewer.show();
    viewer << SetMode3D( domainKernel.className(), "BoundingBox" ) << domainKernel;

    bool space_ok = KSpaceKernel.init( domainKernel.lowerBound(), domainKernel.upperBound(), true );
    if( !space_ok )
    {
        trace.error() << "Error in the Khalimsky space construction." << std::endl;
        return false;
    }

    int count = 0;
    /*for( auto it = setKernel.begin(), itend = setKernel.end();
         it != itend;
         ++it )
    {
        kernels[ 13 ].insert( KSpaceKernel.sSpel( *it )); /// It's a trit ({0,1,2}) encoded array, and base10(11) is 4
        ++count;
    }*/

    ASSERT(( count != 0 )); // Error when creating full kernel

    for( int z = -1; z <= 1; ++z )
    {
        for( int y = -1; y <= 1; ++y )
        {
            for( int x = -1; x <= 1; ++x )
            {
                RealPoint shiftPoint = rOrigin + RealPoint( x * h, y * h, z + h );
                KernelSupport kernelShifted( shiftPoint, radius );
                int offset = ( x + 1 ) + (( y + 1 ) * 3 ) + (( z + 1 ) * 9 );
                if( offset == 13 ) // Full kernel
                {
                    continue;
                }
                else
                {
                    if( offset != 1 )//&& offset != 25 )
                        continue;

                    EuclideanShape current( kernel, kernelShifted );

                    GaussDigitizer< Z3i::Space, EuclideanShape > digCurrentKernel;
                    digCurrentKernel.attach( current );
                    digCurrentKernel.init( kernel.getLowerBound(), kernel.getUpperBound(), h );

                    DigitalSet setCurrentKernel( domainKernel );
                    Shapes< Domain >::digitalShaper ( setCurrentKernel, digCurrentKernel );

                    int count2 = 0;
                    /*for( auto it = setCurrentKernel.begin(), itend = setCurrentKernel.end();
                         it != itend;
                         ++it )
                    {
                        //                    viewer << CustomColors3D( Color::Black, cmap_grad( b ));
                        viewer << *it;

                        kernels[ offset ].insert( KSpaceKernel.sSpel( *it ));
                        ++count2;
                    }*/

                    ASSERT(( count2 != 0 )); // Error when creating masks

                }
            }
        }
    }


    //////////////////////////////

    viewer << Viewer3D::updateDisplay;

    return application.exec();
}

int testII2D_same_results( )
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

    typedef DigitalSetSelector< Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type MySet;
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

    typedef LightImplicitDigitalSurface< Z2i::KSpace, Digitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > DigSurface;
    typedef Z2i::KSpace::SCell SCell;
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
    typedef VisitorRange::ConstIterator It;
    VisitorRange range( new Visitor( digSurf, *digSurf.begin() ) );
    It ibegin = range.begin();
    It iend = range.end();

    std::vector< double > results;
    std::back_insert_iterator< std::vector< double > > insertResults( results );
    estimator.eval( ibegin, iend, insertResults, ball );

    //    std::vector< double > resultsShape;
    //    std::back_insert_iterator< std::vector< double > > insertResultsShape( resultsShape );
    //    estimator.eval( ibegin, iend, insertResultsShape, ball );

    int p = 0;
    //    std::cout << "here"<<std::endl;

    VisitorRange range2( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range2.begin();
    iend = range2.end();
    for ( ;
          ibegin != iend; ++ibegin, ++p)
    {
        //        std::cout << "p"<<std::endl;
        /*if ( p != 148 )
            continue;*/

        // double resultOneShape = estimator.eval( it, *ishape );
        double resultOne = estimator.eval( ibegin );

        if( results[ p ] != resultOne )
            std::cout << p << " error without shape " << results[p] << " vs " << resultOne << std::endl;

    }

    return 1;
}

bool testII2D()
{
    typedef ImplicitBall< Z2i::Space > Ball;
    typedef Z2i::RealPoint RealPoint;
    typedef Z2i::Domain Domain;

    /// Euclidean shape
    RealPoint rcenter( 0.0, 0.0 );
    Ball ball( rcenter, 5.0 );

    /// Digital shape
    typedef Z2i::Point Point;
    typedef GaussDigitizer< Z2i::Space, Ball > Digitizer;

    Point dcenter( 0, 0 );
    double h = 0.1;
    Digitizer* gauss = new Digitizer();
    gauss->attach( ball );
    gauss->init( RealPoint( -6.0, -6.0 ), RealPoint( 6.0, 6.0 ), h );

    Domain domain = gauss->getDomain();

    typedef PointFunctorFromPointPredicateAndDomain< Digitizer, Domain, unsigned int > MyPointFunctor;
    MyPointFunctor pointFunctor( gauss, domain, 1, 0 );

    typedef DigitalSetSelector< Domain, BIG_DS + HIGH_ITER_DS + HIGH_BEL_DS >::Type MySet;
    MySet set( domain );
    Shapes< Domain >::digitalShaper( set, *gauss );

    /// Khalimsky shape
    Z2i::KSpace k;
    k.init( domain.lowerBound(), domain.upperBound(), true );

    typedef FunctorOnCells< MyPointFunctor, Z2i::KSpace > MyCellFunctor;
    MyCellFunctor cellFunctor ( pointFunctor, k );

    typedef LightImplicitDigitalSurface< Z2i::KSpace, Digitizer > LightImplicitDigSurface;
    typedef DigitalSurface< LightImplicitDigSurface > DigSurface;
    typedef Z2i::KSpace::SCell SCell;
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
    typedef VisitorRange::ConstIterator It;

    /// Integral Invariant
    typedef IntegralInvariantGaussianCurvatureEstimator< Z2i::KSpace, MyCellFunctor > GaussianEstimator;
    typedef IntegralInvariantMeanCurvatureEstimator< Z2i::KSpace, MyCellFunctor > MeanEstimator;

    double re = 3.0;

    /////// Mean
    MeanEstimator meanEstimator( k, cellFunctor );
    meanEstimator.init( h, re );

    std::vector< double > resultsMean;
    std::back_insert_iterator< std::vector< double > > insertResultsMean( resultsMean );

    VisitorRange range( new Visitor( digSurf, *digSurf.begin() ) );
    It ibegin = range.begin();
    It iend = range.end();

    meanEstimator.eval( ibegin, iend, insertResultsMean, ball );

    /////// Gaussian
    GaussianEstimator gaussianEstimator( k, cellFunctor );
    gaussianEstimator.init( h, re );

    std::vector< double > resultsGaussian;
    std::back_insert_iterator< std::vector< double > > insertResultsGaussian( resultsGaussian );

    VisitorRange range2( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range2.begin();
    iend = range2.end();

    gaussianEstimator.eval( ibegin, iend, insertResultsGaussian );


    ASSERT( resultsMean.size() == resultsGaussian.size() );

    for( unsigned int i = 0; i < resultsMean.size(); ++i )
    {
        std::cout << i << " " << resultsMean[ i ] << " " << resultsGaussian[ i ] << std::endl;
    }

#ifdef EXPORT

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

    VisitorRange range4( new Visitor( digSurf, *digSurf.begin() ) );
    ibegin = range4.begin();
    iend = range4.end();
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
#endif

    return true;
}

int testII3D( int argc, char** argv )
{
    typedef Z3i::Space::RealPoint RealPoint;
    typedef RealPoint::Coordinate Ring;
    typedef MPolynomial<3, Ring> Polynomial3;
    typedef MPolynomialReader<3, Ring> Polynomial3Reader;
    typedef ImplicitPolynomial3Shape<Z3i::Space> ImplicitShape;
    typedef GaussDigitizer<Z3i::Space,ImplicitShape> DigitalShape;
    typedef DigitalShape::PointEmbedder DigitalEmbedder;

    double p1[ 3 ] = {-10.0,-10.0,-10.0};
    double p2[ 3 ] = {10.0,10.0,10.0};

    double step = 0.2;

    double re = 3;


    Polynomial3 P;
    Polynomial3Reader reader;
    std::string poly_str = "x^4 + y^4 + z^4 - 2500.0";//"x^2 + y^2 + z^2 - 25.0";
    std::string::const_iterator iter
            = reader.read( P, poly_str.begin(), poly_str.end() );
    if ( iter != poly_str.end() )
    {
        std::cerr << "ERROR: I read only <"
                  << poly_str.substr( 0, iter - poly_str.begin() )
                  << ">, and I built P=" << P << std::endl;
        return -1;
    }


    ImplicitShape* ishape = new ImplicitShape( P );
    DigitalShape* dshape = new DigitalShape();
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


    typedef Z3i::KSpace::Surfel Surfel;
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
    typedef IntegralInvariantGaussianCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator

    MyPointFunctor pointFunctor( dshape, domain, 1, 0 );
    MyCellFunctor functor ( pointFunctor, K );
    MyCurvatureEstimator iigaussest ( K, functor );
    iigaussest.init( step, re );


//    std::vector< double > resultTrue;
    std::vector< double > resultII;
//    std::vector< RealPoint > nearestPoints;
    //-----------------------------------------------------------------------
    // Looking for the min and max values

    double minCurv = 1;
    double maxCurv = 0;
    //  SCellToMidPoint< KSpace > midpoint( K );
//    NearestPointEmbedder< Z3i::KSpace, ImplicitShape > ScellToRealPoint;
//    ScellToRealPoint.init( K, step, *ishape );

    std::back_insert_iterator< std::vector< double > > resultIIIterator( resultII );
    iigaussest.eval( theSetOfSurfels.begin(), theSetOfSurfels.end(), resultIIIterator, *ishape );

    int p = 0;

    //    std::cout << "here"<<std::endl;
    /*for ( auto it = theSetOfSurfels.begin(), it_end = theSetOfSurfels.end();
          it != it_end; ++it, ++p)
    {
//        RealPoint A;// = ScellToRealPoint( *it );
//        double a = 0.04;//ishape->gaussianCurvature( A );
        double b = resultII[ p ];


//        //        if( resultIIShape[ p ] != resultOneShape )
//        //            std::cout << "error with shape " << std::endl;

//        //        std::cout << b << std::endl;
//        resultTrue.push_back( a );
//        nearestPoints.push_back( A );

        //      if ( boost::math::isnan( a ))
        //      {
        //        a = 0;
        //      }

//        double Linf = std::abs ( a - b );
        if ( b > maxCurv )
        {
            maxCurv = b;
        }
        else if ( b < minCurv )
        {
            minCurv = b;
        }

    }*/

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
    /*for ( auto it = theSetOfSurfels.begin(),
          it_end = theSetOfSurfels.end();
          it != it_end; ++it, ++nbSurfels, ++i )
    {
//        double a = resultTrue[ i ];
        double b = resultII[ i ];
//        double Linf = std::abs ( a - b );

//        std::cout << b << std::endl;

        viewer << CustomColors3D( Color::Black, cmap_grad( b ));
        viewer << *it;
    }*/

    viewer << Viewer3D::updateDisplay;

    return application.exec();
}

int testII3D_same_results( )
{
    typedef Z3i::Space::RealPoint RealPoint;
    typedef RealPoint::Coordinate Ring;
    typedef MPolynomial<3, Ring> Polynomial3;
    typedef MPolynomialReader<3, Ring> Polynomial3Reader;
    typedef ImplicitPolynomial3Shape<Z3i::Space> ImplicitShape;
    typedef GaussDigitizer<Z3i::Space,ImplicitShape> DigitalShape;
    typedef DigitalShape::PointEmbedder DigitalEmbedder;

    double p1[ 3 ] = {-10.0,-10.0,-10.0};
    double p2[ 3 ] = {10.0,10.0,10.0};

    double step = 0.2;

    double re = 3;


    Polynomial3 P;
    Polynomial3Reader reader;
    std::string poly_str = "x^2 + y^2 + z^2 - 25.0";
    std::string::const_iterator iter
            = reader.read( P, poly_str.begin(), poly_str.end() );
    if ( iter != poly_str.end() )
    {
        std::cerr << "ERROR: I read only <"
                  << poly_str.substr( 0, iter - poly_str.begin() )
                  << ">, and I built P=" << P << std::endl;
        return -1;
    }


    ImplicitShape* ishape = new ImplicitShape( P );
    DigitalShape* dshape = new DigitalShape();
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


    typedef Z3i::KSpace::Surfel Surfel;
    typedef Z3i::KSpace::SurfelSet SurfelSet;
    typedef SetOfSurfels< Z3i::KSpace, SurfelSet > MySetOfSurfels;
    typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;


    MySetOfSurfels theSetOfSurfels( K, surfAdj );
    Surfel bel = Surfaces< Z3i::KSpace >::findABel( K, *dshape, 100000 );
    Surfaces< Z3i::KSpace >::trackBoundary( theSetOfSurfels.surfelSet(),
                                            K, surfAdj,
                                            *dshape, bel );

    typedef PointFunctorFromPointPredicateAndDomain< DigitalShape, Z3i::Domain, unsigned int > MyPointFunctor;
    typedef FunctorOnCells< MyPointFunctor, Z3i::KSpace > MyCellFunctor;
    typedef IntegralInvariantGaussianCurvatureEstimator< Z3i::KSpace, MyCellFunctor > MyCurvatureEstimator; // Gaussian curvature estimator

    MyPointFunctor pointFunctor( dshape, domain, 1, 0 );
    MyCellFunctor functor ( pointFunctor, K );
    MyCurvatureEstimator iigaussest ( K, functor );
    iigaussest.init( step, re );

    std::vector< double > resultII;

    //    std::vector< double > resultIIShape;
    //    std::back_insert_iterator< std::vector< double > > resultIIShapeIterator( resultIIShape );
    //    iigaussest.eval( theSetOfSurfels.begin(), theSetOfSurfels.end(), resultIIShapeIterator, *ishape );

    std::back_insert_iterator< std::vector< double > > resultIIIterator( resultII );
    iigaussest.eval( theSetOfSurfels.begin(), theSetOfSurfels.end(), resultIIIterator );

    int p = 0;
    //    std::cout << "here"<<std::endl;
    /*for ( auto it = theSetOfSurfels.begin(), it_end = theSetOfSurfels.end();
          it != it_end; ++it, ++p)
    {

        //        std::cout << "p"<<std::endl;
        //if ( p != 148 )
        //    continue;

        // double resultOneShape = iigaussest.eval( it, *ishape );
        double resultOne = iigaussest.eval( it );

        if( resultII[ p ] != resultOne )
            std::cout << p << " error without shape " << resultII[p] << " vs " << resultOne << std::endl;
    }*/
    return 0;
}

int testII3D_noise( int argc, char** argv )
{
    QApplication application( argc, argv );
    Viewer3D viewer;

    typedef Z3i::Space::RealPoint RealPoint;
    typedef Z3i::Space::RealPoint::Coordinate Ring;
    typedef MPolynomial< 3, Ring > Polynomial3;
    typedef MPolynomialReader<3, Ring> Polynomial3Reader;
    typedef ImplicitPolynomial3Shape<Z3i::Space> Shape;
    typedef Z3i::Space Space;

    RealPoint border_min( -10, -10, -10 );
    RealPoint border_max( 10, 10, 10 );

    double h = 0.2;
    double noiseLevel = 0.5;
    double alpha = 0.333333;
    double radius_kernel = 3;
    bool lambda_optimized = false;

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

    Shape* aShape = new Shape( poly );

    typedef typename Space::RealPoint RealPoint;
    typedef GaussDigitizer< Z3i::Space, Shape > DigitalShape;
    typedef Z3i::KSpace KSpace;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::Surfel Surfel;

    ASSERT (( noiseLevel < 1.0 ));
    bool withNoise = ( noiseLevel <= 0.0 ) ? false : true;
    if( withNoise )
        noiseLevel *= h;

    // Digitizer
    DigitalShape* dshape = new DigitalShape();
    dshape->attach( *aShape );
    dshape->init( border_min, border_max, h );

    KSpace K;
    if ( ! K.init( dshape->getLowerBound(), dshape->getUpperBound(), true ) )
    {
        std::cerr << "[3dLocalEstimators_0memory] error in creating KSpace." << std::endl;
        return false;
    }

    typedef KanungoNoise< DigitalShape, Z3i::Domain > KanungoPredicate;
    typedef LightImplicitDigitalSurface< KSpace, KanungoPredicate > Boundary;
    typedef DigitalSurface< Boundary > MyDigitalSurface;
    typedef typename MyDigitalSurface::ConstIterator ConstIterator;

    typedef DepthFirstVisitor< MyDigitalSurface > Visitor;
    typedef GraphVisitorRange< Visitor > VisitorRange;
    typedef typename VisitorRange::ConstIterator VisitorConstIterator;

    typedef PointFunctorFromPointPredicateAndDomain< KanungoPredicate, Z3i::Domain, unsigned int > MyPointFunctor;
    typedef FunctorOnCells< MyPointFunctor, KSpace > MySpelFunctor;

    // Extracts shape boundary
    KanungoPredicate * noisifiedObject = new KanungoPredicate( *dshape, dshape->getDomain(), noiseLevel );
    SCell bel = Surfaces< KSpace >::findABel( K, *noisifiedObject, 10000 );
    Boundary * boundary = new Boundary( K, *noisifiedObject, SurfelAdjacency< KSpace::dimension >( true ), bel );
    MyDigitalSurface surf ( *boundary );

    double minsize = dshape->getUpperBound()[0] - dshape->getLowerBound()[0];
    unsigned int tries = 0;
    while( surf.size() < 2 * minsize || tries > 150 )
    {
        delete boundary;
        bel = Surfaces< KSpace >::findABel( K, *noisifiedObject, 10000 );
        boundary = new Boundary( K, *noisifiedObject, SurfelAdjacency< KSpace::dimension >( true ), bel );
        surf = MyDigitalSurface( *boundary );
        ++tries;
    }

    if( tries > 150 )
    {
        std::cerr << "Can't found a proper bel. So .... I ... just ... kill myself." << std::endl;
        return false;
    }

    VisitorRange * range;
    VisitorConstIterator ibegin;
    VisitorConstIterator iend;

    range = new VisitorRange( new Visitor( surf, *surf.begin() ));
    ibegin = range->begin();
    iend = range->end();

    viewer.show();
    viewer << SetMode3D( dshape->getDomain().className(), "BoundingBox" ) << dshape->getDomain();

    for( ; ibegin != iend; ++ibegin )
    {
        //                    viewer << CustomColors3D( Color::Black, cmap_grad( b ));
        viewer << *ibegin;
    }

    delete range;

    viewer << Viewer3D::updateDisplay;
    return application.exec();
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

    testII3D_noise( argc, argv );
//    test2DTopology();
//    testII2D( );
//    testII2D_kernels();
//    testII2D_kernels_2();
//    testII2D_same_results();
//    testII3D_same_results();
//    testII3D_kernels(argc, argv);
//    testII3D( argc, argv );
//    testII3D();

    trace.endBlock();
    return 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
