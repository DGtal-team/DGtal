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
 * @file shapeGridCurveEstimator.cpp
 * @ingroup tutorial-examples
 * @author Tristan Roussillon (tristan.roussillon@liris.cnrs.fr)
 *
 *
 * @date 2010/10/17
 * 
 * @brief An example of generating a grid curve from a parametric shape
 * and estimating its length. 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////

//! [shapeGridCurveEstimator-includes]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "ConfigExamples.h"

//shape and digitizer
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/geometry/nd/GaussDigitizer.h"

//tracking grid curve
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/2d/GridCurve.h"

//estimation
#include "DGtal/geometry/2d/estimators/DSSLengthEstimator.h"
#include "DGtal/geometry/2d/estimators/TrueGlobalEstimatorOnPoints.h"
#include "DGtal/geometry/2d/estimators/ParametricShapeArcLengthFunctor.h"

//! [shapeGridCurveEstimator-includes]

#include "DGtal/geometry/2d/GreedySegmentation.h"

///////////////////////////////////////////////////////////////////////////////

int main()
{
  //shape
  typedef Flower2D<Z2i::Space> Flower; 
  Flower2D<Z2i::Space> flower(Z2i::Point(0,0), 20, 5, 5, 0);
  
  //! [shapeGridCurveEstimator-dig]
  //digitization of a shape of type Flower 
  //into a digital space of type Space
  GaussDigitizer<Z2i::Space,Flower> dig;  
  dig.attach( flower );
  dig.init( flower.getLowerBound()+Z2i::Vector(-1,-1),
               flower.getUpperBound()+Z2i::Vector(1,1), 1 ); 
  //! [shapeGridCurveEstimator-dig]
  
  //! [shapeGridCurveEstimator-prepareTracking]
  //Khalimsky space
  Z2i::KSpace ks;
  ks.init( dig.getLowerBound(), dig.getUpperBound(), true );
  //adjacency (4-connectivity)
  SurfelAdjacency<2> sAdj( true );
  //! [shapeGridCurveEstimator-prepareTracking]

  //! [shapeGridCurveEstimator-tracking]
  //searching for one boundary element
  Z2i::SCell bel = Surfaces<Z2i::KSpace>::findABel( ks, dig, 1000 );
  //tracking
  vector<Z2i::Point> boundaryPoints;
  Surfaces<Z2i::KSpace>
    ::track2DBoundaryPoints( boundaryPoints, ks, sAdj, dig, bel );
  //! [shapeGridCurveEstimator-tracking]

  //! [shapeGridCurveEstimator-instantiation]
  Z2i::Curve c;
  c.initFromVector( boundaryPoints );  
  //! [shapeGridCurveEstimator-instantiation]
  
  //! [shapeGridCurveEstimator-lengthEstimation]
  typedef Z2i::Curve::PointsRange Range; 
  Range r = c.getPointsRange(); 
  DSSLengthEstimator< Range::ConstIterator > DSSlength;
  DSSlength.init( 1, r.begin(), r.end(), c.isClosed() );
  double length1 = DSSlength.eval();
  cout << "Length (h=1): " << length1 << endl; 
  //! [shapeGridCurveEstimator-lengthEstimation]

//@TODO correct init method of trueLengthEstimator (remove &flower)
  //! [shapeGridCurveEstimator-trueEstimation]
  typedef ParametricShapeArcLengthFunctor< Flower > Length;
  TrueGlobalEstimatorOnPoints< 
    Range::ConstIterator, 
    Flower, 
    Length  >  trueLengthEstimator;
  trueLengthEstimator.init( 1, r.begin(), r.end(), &flower, c.isClosed());
  double trueLength = trueLengthEstimator.eval(); 
  cout << "ground truth: " << trueLength << endl; 
  //! [shapeGridCurveEstimator-trueEstimation]

  //! [shapeGridCurveEstimator-higher]
  //digitization at higher resolution
  dig.init( flower.getLowerBound()+Z2i::Vector(-1,-1),
               flower.getUpperBound()+Z2i::Vector(1,1), 0.5 ); 
  //searching for one boundary element
  bel = Surfaces<Z2i::KSpace>::findABel( ks, dig, 1000 );
  //tracking
  Surfaces<Z2i::KSpace>
    ::track2DBoundaryPoints( boundaryPoints, ks, sAdj, dig, bel );
  //reset grid curve and its points range
  c.initFromVector( boundaryPoints );
  r = c.getPointsRange(); 
  //estimate length
  DSSlength.init( 0.5, r.begin(), r.end(), c.isClosed() );
  double length2 = DSSlength.eval();
  cout << "Length (h=0.5): " << length2 << endl;  
  //! [shapeGridCurveEstimator-higher]
  //@TODO bug in the length ?
  
  
/*  typedef Z2i::Curve::PointsRange::ConstIterator ConstIterator; 
  typedef ArithmeticalDSS<ConstIterator,int,4> SegmentComputer;
  typedef GreedySegmentation<SegmentComputer> Segmentation;

  Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer() );
  Segmentation::SegmentComputerIterator i = theSegmentation.begin();
  Segmentation::SegmentComputerIterator end = theSegmentation.end();
  
  DGtal::Board2D aBoard;
  aBoard << SetMode("PointVector", "Grid");
  for ( ; i != end; ++i) {
    aBoard << SetMode(i->styleName(), "Points") << *i; 
    aBoard << SetMode(i->styleName(), "BoundingBox") << *i; 
  } 
  aBoard.saveEPS("DisplayGridCurveSegmentationTuto.eps");
*/
  
  return 0;

}

///////////////////////////////////////////////////////////////////////////////
