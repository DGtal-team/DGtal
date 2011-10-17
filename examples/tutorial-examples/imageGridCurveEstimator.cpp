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
 * @file imageGridCurveEstimator.cpp
 * @ingroup tutorial-examples
 * @author Tristan Roussillon (tristan.roussillon@liris.cnrs.fr)
 *
 *
 * @date 2010/10/17
 * 
 * @brief An example of extracting a grid curve from an image iso-contour
 * and estimating its length. 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <algorithm>
///////////////////////////////////////////////////////////////////////////////

//! [imageGridCurveEstimator-includes]
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "ConfigExamples.h"

//images
#include "DGtal/io/readers/PNMReader.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/imagesSetsUtils/IntervalForegroundPredicate.h"

//tracking grid curve
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/geometry/2d/GridCurve.h"

//estimation
#include "DGtal/geometry/2d/estimators/DSSLengthEstimator.h"

//display
#include "DGtal/io/boards/Board2D.h"
//! [imageGridCurveEstimator-includes]

#include "DGtal/geometry/2d/GreedySegmentation.h"

///////////////////////////////////////////////////////////////////////////////

int main()
{
  //image import
  typedef DGtal::ImageContainerBySTLVector< Z2i::Domain, int> Image;
  std::string filename =  examplesPath + "samples/contourS.pgm";
  Image image = DGtal::PNMReader<Image>::importPGMImage(filename); 
  
  //! [imageGridCurveEstimator-prepareTracking]
  Z2i::KSpace ks;                                                                           //Khalimsky space 
  ks.init( image.lowerBound(), image.upperBound(), true );
  SurfelAdjacency<2> sAdj( true );                                              //adjacency
  IntervalForegroundPredicate<Image> predicate(image,0,135); //predicate from the image
  //! [imageGridCurveEstimator-prepareTracking]

  //! [imageGridCurveEstimator-tracking]
  Z2i::SCell bel = Surfaces<Z2i::KSpace>::findABel( ks, predicate, 1000 );
  vector<Z2i::Point> boundaryPoints;
  Surfaces<Z2i::KSpace>::track2DBoundaryPoints( boundaryPoints, ks, sAdj, predicate, bel );
  //! [imageGridCurveEstimator-tracking]

  //! [imageGridCurveEstimator-instanciation]
  Z2i::Curve c;
  c.initFromVector( boundaryPoints );  
  //! [imageGridCurveEstimator-instanciation]
  
  //! [imageGridCurveEstimator-lengthEstimation]
  Z2i::Curve::PointsRange r = c.getPointsRange(); 
  DSSLengthEstimator< Z2i::Curve::PointsRange::ConstIterator > DSSlength;
  DSSlength.init(1, r.begin(), r.end(), c.isClosed());
  double length = DSSlength.eval();
  cout << "Length: " << length << endl; 
  //! [imageGridCurveEstimator-lengthEstimation]
  
  //segmentation
  typedef Z2i::Curve::PointsRange::ConstIterator ConstIterator; 
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
  
  
  return 0;

}

///////////////////////////////////////////////////////////////////////////////
