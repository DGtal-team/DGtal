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
 * @file greedyAlphaThickDecomposition.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/02/01
 *
 * An example file named greedyAlphaThickDecomposition
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/AlphaThickSegmentComputer.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/geometry/curves/GreedySegmentation.h"
#include "DGtal/io/readers/GenericReader.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main(  )
{
  trace.beginBlock ( "Example of greedy alpha thick segment decomposition" );
  
  typedef  std::vector<Z2i::RealPoint>::const_iterator ConstIterator;
  typedef  AlphaThickSegmentComputer<Z2i::RealPoint, ConstIterator > AlphaThickSegmentComputer2D;
  
  Board2D aBoard;
  std::string file = examplesPath + "samples/contourSnoisy.sdp";
  std::vector<Z2i::RealPoint> aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile (file);  

  // displaying contour
  aBoard << SetMode(aContour[0].className(), "Grid"); 
  std::vector<LibBoard::Point> poly;
  for (unsigned int i = 0; i< aContour.size(); i++)  poly.push_back(LibBoard::Point(aContour[i][0], aContour[i][1]));
  aBoard.setPenColor(DGtal::Color::Gray);
  aBoard.fillPolyline(poly);

  // Computing greedy Alpha Thick decomposition.
  //! [greedyAlphaThickDecompositionModeDisplay]
  aBoard << SetMode("AlphaThickSegment", "BoundingBox");
  //! [greedyAlphaThickDecompositionModeDisplay]

  //! [greedyAlphaThickDecompositionAlgo]
  ConstIterator contourIt = aContour.begin();
  while(contourIt != aContour.end()){
    AlphaThickSegmentComputer2D aComputer;
    aComputer.init(4);  
    while(contourIt != aContour.end() && aComputer.extendFront(*contourIt)){
      contourIt++;
    }
    aBoard<< aComputer;
    if(contourIt!=aContour.end()){
      contourIt--;
    }
  }
  //! [greedyAlphaThickDecompositionAlgo]  
  
  aBoard.saveEPS("greedyAlphaThickDecomposition.eps"); 
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
