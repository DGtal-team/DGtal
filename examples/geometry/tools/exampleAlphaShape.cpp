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
 * @file exampleAlphaShape.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/12/17
 *
 * An example file named exampleAlphaShape.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/IteratorCirculatorTraits.h"
#include "DGtal/helpers/StdDefs.h"

//! [Hull2D-Include]
#include "DGtal/geometry/tools/Hull2DHelpers.h"
//! [Hull2D-Include]
#include "DGtal/geometry/tools/PolarPointComparatorBy2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/AvnaimEtAl2x2DetSignComputer.h"
#include "DGtal/geometry/tools/determinant/InHalfPlaneBySimpleMatrix.h"
#include "DGtal/geometry/tools/determinant/InGeneralizedDiskOfGivenRadius.h"

#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"

#include "DGtal/topology/DigitalSetBoundary.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/DepthFirstVisitor.h"

#include "DGtal/io/boards/Board2D.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
/*
 * @brief Procedure that draws on a board a polygon given by a range of vertices. 
 * @param itb begin iterator
 * @param ite end iterator
 * @param aBoard any board
 * @param isClosed bool equal to 'true' for closed polygon, 'false' otherwise. 
 * @tparam ForwardIterator at least a readable and forward iterator
 * @tparam Board equivalent to Board2D
 */
template <typename ForwardIterator, typename Board>
void drawPolygon(const ForwardIterator& itb, const ForwardIterator& ite, 
		 Board& aBoard, bool isClosed = true)
{
  BOOST_CONCEPT_ASSERT(( boost_concepts::ForwardTraversalConcept<ForwardIterator> )); 
  BOOST_CONCEPT_ASSERT(( boost_concepts::ReadableIteratorConcept<ForwardIterator> )); 
  typedef typename IteratorCirculatorTraits<ForwardIterator>::Value Point; 

  ForwardIterator it = itb; 
  if (it != ite)
    {//for the first point
      Point p1 = *it;
      Point p = p1; 
      //display
      aBoard << SetMode( p.className(), "Grid" )
	     << CustomStyle( p.className()+"/Grid", new CustomPenColor( Color::Red ) )
	     << p 
	     << CustomStyle( p.className()+"/Grid", new CustomPenColor( Color::Black ) ); 
       
      Point prev = p; 
      for (++it; it != ite; ++it, prev = p)
	{//for all other points
	  //conversion
	  p = *it; 
	  //display
	  aBoard << p; 
	  if (prev != p)
	    aBoard.drawArrow(prev[0], prev[1], p[0], p[1]); 
	}

      //display the last edge too
      if (isClosed)
	{
	  if (prev != p1)
	    aBoard.drawArrow(prev[0], prev[1], p1[0], p1[1]); 
	}
    }
}


/**
 * Algorithms that computes the convex hull
 * of a point set
 */
void convexHull()
{
  //Digitization of a disk of radius 6
  Ball2D<Z2i::Space> ball(Z2i::Point(0,0), 6);
  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
  Z2i::DigitalSet pointSet(domain);   
  Shapes<Z2i::Domain>::euclideanShaper(pointSet, ball);

  //! [Hull2D-Namespace]
  using namespace Hull2D; 
  //! [Hull2D-Namespace]

  //! [Hull2D-Functor]
  typedef InHalfPlaneBySimpleMatrix<Z2i::Point, DGtal::int64_t> Functor;  
  Functor functor; 
  //! [Hull2D-Functor]

  { //convex hull in counter-clockwise order
    vector<Z2i::Point> res; 

    //! [Hull2D-StrictPredicateCCW]
    typedef PredicateFromOrientationFunctor2<Functor, false, false> StrictPredicate; 
    StrictPredicate predicate( functor ); 
    //! [Hull2D-StrictPredicateCCW]
    //according to the last two template arguments, neither strictly negative values, nor zeros are accepted: 
    //the predicate returns 'true' only for strictly positive values returned by the underlying functor. 

    //! [Hull2D-AndrewAlgo]
    andrewConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    //! [Hull2D-AndrewAlgo]

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "ConvexHullCCW.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("ConvexHullCCW.png", Board2D::CairoPNG);
#endif
  }

  { //convex hull in counter-clockwise order with all the points lying on the edges
    vector<Z2i::Point> res; 

    //! [Hull2D-LargePredicateCCW]
    typedef PredicateFromOrientationFunctor2<Functor, false, true> LargePredicate; 
    LargePredicate predicate( functor ); 
    //! [Hull2D-LargePredicateCCW]
    //according to the last template argument, zeros are accepted so that  
    //the predicate returns 'true' for all the positive values returned by the underlying functor. 

    //andrew algorithm
    andrewConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "ConvexHullCCWWithPointsOnEdges.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("ConvexHullCCWWithPointsOnEdges.png", Board2D::CairoPNG);
#endif

  }

  { //convex hull in clockwise order
    vector<Z2i::Point> res; 

    //! [Hull2D-StrictPredicateCW]
    typedef PredicateFromOrientationFunctor2<Functor, true, false> StrictPredicate; 
    StrictPredicate predicate( functor );
    //! [Hull2D-StrictPredicateCW]
    //according to the last two argument template, 
    //the predicate returns 'true' only for strictly negative values returned by the underlying functor. 

    //andrew algorithm
    andrewConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "ConvexHullCW.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("ConvexHullCW.png", Board2D::CairoPNG);
#endif
  }

  { //convex hull in counter-clockwise order
    vector<Z2i::Point> res; 

    //geometric predicate
    typedef PredicateFromOrientationFunctor2<Functor, false, false> StrictPredicate; 
    StrictPredicate predicate( functor ); 

    //! [Hull2D-GrahamAlgo]
    grahamConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate ); 
    //! [Hull2D-GrahamAlgo]

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "ConvexHullCCWbis.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("ConvexHullCCWbis.png", Board2D::CairoPNG);
#endif
  }

  { //order of the points for andrew algorithm
    vector<Z2i::Point> res; 
    std::copy( pointSet.begin(), pointSet.end(), back_inserter( res ) ); 

    std::sort( res.begin(), res.end() ); 

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board, false ); 
    board.saveSVG( "AndrewWEVP.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AndrewWEVP.png", Board2D::CairoPNG);
#endif
  }

  { //order of the points for graham algorithm
    vector<Z2i::Point> res; 
    std::copy( pointSet.begin(), pointSet.end(), back_inserter( res ) ); 

    //find an extremal point
    //NB: we choose the point of greatest x-coordinate
    //so that the sort step (by a polar comparator) 
    //returns a weakly externally visible polygon
    typename std::vector<Z2i::Point>::iterator itMax 
      = std::max_element( res.begin(), res.end() ); 

    //sort around this point with a polar comparator
    PolarPointComparatorBy2x2DetComputer<Z2i::Point> comparator;  
    comparator.setPole( *itMax );
    std::sort( res.begin(), res.end(), comparator ); 

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board, false ); 
    board.saveSVG( "GrahamWEVP.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("GrahamWEVP.png", Board2D::CairoPNG);
#endif
  }


}

/**
 * Algorithms that computes the alpha-shape
 * of a point set
 */
void alphaShape()
{

  //Digitization of a disk of radius 8
  Ball2D<Z2i::Space> ball(Z2i::Point(0,0), 8);
  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
  Z2i::DigitalSet digitalSet(domain);   
  Shapes<Z2i::Domain>::euclideanShaper(digitalSet, ball);
  
  //Contour of the digital set
  Z2i::KSpace kspace; 
  kspace.init(domain.lowerBound()-Z2i::Point(1,1), domain.upperBound()+Z2i::Point(1,1), true); 
  typedef DigitalSetBoundary<Z2i::KSpace, Z2i::DigitalSet> DigitalSurfaceContainer;
  typedef DigitalSurface<DigitalSurfaceContainer> CustomDigitalSurface; 
  DigitalSurfaceContainer digitalSurfaceContainer( kspace, digitalSet ); 
  CustomDigitalSurface digitalSurface( digitalSurfaceContainer ); 

  //Grid curve
  Z2i::Curve gridCurve; 
  typedef DepthFirstVisitor<DigitalSurface<DigitalSurfaceContainer> > CustomVisitor;
  CustomVisitor visitor( digitalSurface, *digitalSurface.begin() );
  while ( ! visitor.finished() )
    {
      gridCurve.pushBack( visitor.current().first );
      visitor.expand();
    }

  //Point set defined as the set of (not duplicated) inner points  
  typedef Z2i::Curve::InnerPointsRange PointRange; 
  PointRange pointsRange = gridCurve.getInnerPointsRange();
  vector<Z2i::Point> border; 
  unique_copy( pointsRange.begin(), pointsRange.end(), back_inserter( border ) ); 

  //namespace for hull functions
  using namespace Hull2D; 

  { //alpha = 0
    trace.info() << " alpha == 0 " << endl; 
    vector<Z2i::Point> res; 
    
    //! [Hull2D-RadiusPredicateInf]
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    Functor functor(true, 1, 0); //alpha = 0; 1/alpha -> +inf 
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 
    //! [Hull2D-RadiusPredicateInf]

    //! [Hull2D-ClosedGrahamScan]
    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    //! [Hull2D-ClosedGrahamScan]

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShape0.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShape0.png", Board2D::CairoPNG);
#endif

   }

  { //alpha = 0
    trace.info() << " alpha == 0 " << endl; 
    vector<Z2i::Point> res; 
    
    //comparator and functor
    typedef InHalfPlaneBySimpleMatrix<Z2i::Point, DGtal::int64_t> Functor;  
    Functor functor; 
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShape0bis.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShape0bis.png", Board2D::CairoPNG);
#endif

   }

  //negative alpha shape
  { //alpha = -1
    trace.info() << " alpha == -1 " << endl; 
    vector<Z2i::Point> res; 

    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    //! [Hull2D-RadiusPredicateM1]
    Functor functor(false, 1, 1); //1/alpha = -sqrt(1/1) = -1 
    //! [Hull2D-RadiusPredicateM1]
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeM1.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeM1.png", Board2D::CairoPNG);
#endif

   }

  { //alpha = -sqrt(5)
    trace.info() << " alpha == -sqrt(5) " << endl; 
    vector<Z2i::Point> res; 

    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    //! [Hull2D-RadiusPredicateMsqrt5]
    Functor functor(false, 5, 1); //1/alpha = -sqrt(5) 
    //! [Hull2D-RadiusPredicateMsqrt5]
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeMSqrt5.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeMSqrt5.png", Board2D::CairoPNG);
#endif

  }

  { //alpha = -5
    trace.info() << " alpha == -5 " << endl; 
    vector<Z2i::Point> res; 

    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    //! [Hull2D-RadiusPredicateM5]
    Functor functor(false, 25, 1); //1/alpha = -sqrt(25/1) = -5 
    //! [Hull2D-RadiusPredicateM5]
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeM5.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeM5.png", Board2D::CairoPNG);
#endif

  }

  //positive alpha shape
  { 
    trace.info() << " alpha == 8 " << endl; 
    vector<Z2i::Point> res; 

    //! [Hull2D-RadiusPredicateP8]
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    Functor functor(true, 64, 1); //1/alpha = sqrt(64/1) = 8 
    typedef PredicateFromOrientationFunctor2<Functor, false, true> Predicate; 
    Predicate predicate( functor ); 
    //! [Hull2D-RadiusPredicateP8]

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeP8.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeP8.png", Board2D::CairoPNG);
#endif

  }

  //positive alpha shape
  { 
    trace.info() << " alpha == 9 " << endl; 
    vector<Z2i::Point> res; 

    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef InGeneralizedDiskOfGivenRadius<Z2i::Point, DetComputer> Functor; 
    //! [Hull2D-RadiusPredicateP9]
    Functor functor(true, 81, 1); //1/alpha = sqrt(81/1) = 9 
    //! [Hull2D-RadiusPredicateP9]
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( border.begin(), border.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeP9.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeP9.png", Board2D::CairoPNG);
#endif

  }

}

/**
 * Main procedure. 
 * Draw the convex hull and alpha-shape (for several values of alpha)
 * of a point set coming from the digitization of a disk. 
 * @param argc number of arguments
 * @param argv array of arguments
 */
int main( int argc, char** argv )
{
  trace.beginBlock ( "Example exampleAlphaShape" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  convexHull(); 

  alphaShape(); 

  trace.endBlock(); 

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
