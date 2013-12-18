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

#include "DGtal/geometry/tools/Hull2DHelpers.h"
#include "DGtal/geometry/tools/PolarPointComparatorBy2x2DetComputer.h"
#include "DGtal/geometry/tools/determinant/AvnaimEtAl2x2DetSignComputer.h"
#include "DGtal/geometry/tools/determinant/InHalfPlaneBySimpleMatrix.h"
#include "DGtal/geometry/tools/determinant/RadiusFunctor.h"

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
  //Digitization of a disk of radius 12
  Ball2D<Z2i::Space> ball(Z2i::Point(0,0), 12);
  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
  Z2i::DigitalSet pointSet(domain);   
  Shapes<Z2i::Domain>::euclideanShaper(pointSet, ball);

  //namespace for hull functions
  using namespace Hull2D; 

  //orientation functor
  typedef InHalfPlaneBySimpleMatrix<Z2i::Point, DGtal::int64_t> Functor;  
  Functor functor; 

  { //convex hull in counter-clockwise order
    vector<Z2i::Point> res; 

    //geometric predicate
    typedef PredicateFromOrientationFunctor2<Functor, false, false> StrictPredicate; 
    StrictPredicate predicate( functor ); 

    //andrew algorithm
    andrewConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   

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

    typedef PredicateFromOrientationFunctor2<Functor, false, true> LargePredicate; 
    LargePredicate predicate( functor ); 

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

    //geometric predicate
    typedef PredicateFromOrientationFunctor2<Functor, true, false> StrictPredicate; 
    StrictPredicate predicate( functor ); 

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

    //graham algorithm
    PolarPointComparatorBy2x2DetComputer<Z2i::Point> comparator;  
    grahamConvexHullAlgorithm( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate, comparator ); 

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
    board.saveSVG( "pointorder.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("pointorder.png", Board2D::CairoPNG);
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
    board.saveSVG( "pointorderbis.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("pointorderbis.png", Board2D::CairoPNG);
#endif
  }


}

/**
 * Algorithms that computes the alpha-shape
 * of a point set
 */
void alphaShape()
{

  //Digitization of a disk of radius 12
  Ball2D<Z2i::Space> ball(Z2i::Point(0,0), 12);
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
  vector<Z2i::Point> pointSet; 
  unique_copy( pointsRange.begin(), pointsRange.end(), back_inserter( pointSet ) ); 

  //namespace for hull functions
  using namespace Hull2D; 

  { //alpha = 0
    trace.info() << " alpha == 0 " << endl; 
    vector<Z2i::Point> res; 
    
    //geometric predicate
    typedef AvnaimEtAl2x2DetSignComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(true, 1, 0);  
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    //generalized graham algorithm with a radius functor (1/alpha -> +inf)
    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   

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
    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(true, 1, 0);  

    typedef InHalfPlaneBySimpleMatrix<Z2i::Point, DGtal::int64_t> Functor;  
    Functor functor; 
    typedef PredicateFromOrientationFunctor2<Functor> Predicate; 
    Predicate predicate( functor ); 

    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    
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

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(false, 1, 1); //1/alpha = -sqrt(1/1) = -1 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    
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

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(false, 5, 1); //1/alpha = -sqrt(5) 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    
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

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(false, 25, 1); //1/alpha = -sqrt(25/1) = -5 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeM5.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeM5.png", Board2D::CairoPNG);
#endif

  }


  { //alpha = -20
    trace.info() << " alpha == -20 " << endl; 
    vector<Z2i::Point> res; 

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(false, 400, 1); //1/alpha = -sqrt(400/1) = -20 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeM20.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeM20.png", Board2D::CairoPNG);
#endif

  }

  //positive alpha shape
  { //alpha = 12
    trace.info() << " alpha == 12 " << endl; 
    vector<Z2i::Point> res; 

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(true, 144, 1); //1/alpha = sqrt(144/1) = 12 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor, false, true> Predicate; 
    Predicate predicate( rFunctor ); 

    //    closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    PolarPointComparatorBy2x2DetComputer<Z2i::Point> comparator;  
    generalizedGrahamAlgorithm( digitalSet.begin(), digitalSet.end(), back_inserter( res ), predicate, comparator );

    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeP12.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeP12.png", Board2D::CairoPNG);
#endif

  }

  //positive alpha shape
  { //alpha = 14
    trace.info() << " alpha == 14 " << endl; 
    vector<Z2i::Point> res; 

    typedef Simple2x2DetComputer<DGtal::int64_t> DetComputer; 
    typedef RadiusFunctor<Z2i::Point, DetComputer> RadiusFunctor; 
    RadiusFunctor rFunctor(true, 196, 1); //1/alpha = sqrt(196/1) = 14 
    typedef PredicateFromOrientationFunctor2<RadiusFunctor> Predicate; 
    Predicate predicate( rFunctor ); 

    //closedGrahamScanFromAnyPoint( pointSet.begin(), pointSet.end(), back_inserter( res ), predicate );   
    PolarPointComparatorBy2x2DetComputer<Z2i::Point> comparator;  
    generalizedGrahamAlgorithm( digitalSet.begin(), digitalSet.end(), back_inserter( res ), predicate, comparator );
    
    //display
    Board2D board;
    drawPolygon( res.begin(), res.end(), board ); 
    board.saveSVG( "AlphaShapeP14.svg" );  
#ifdef WITH_CAIRO
    board.saveCairo("AlphaShapeP14.png", Board2D::CairoPNG);
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
