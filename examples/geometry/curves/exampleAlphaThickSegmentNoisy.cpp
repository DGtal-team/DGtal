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
 * @file exampleAlphaThickSegmentNoisy.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/01/30
 *
 * An example file named exampleAlphaThickSegmentNoisy.
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
#include <DGtal/io/readers/GenericReader.h>
#include <DGtal/io/readers/PointListReader.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main(  )
{
  trace.beginBlock ( "Example exampleAlphaThickSegment" );
  
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::RealPoint, double  > AlphaThickSegmentComputer2D;
  Board2D aBoard;
  std::string file = examplesPath + "samples/contourSnoisy.sdp";
  std::vector<Z2i::RealPoint> aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile (file);


  // displaying contour
  aBoard << SetMode(aContour[0].className(), "Grid"); 
  for (unsigned int i = 0; i< aContour.size(); i++){
      aBoard << aContour[i];
      aBoard.drawLine(aContour[i][0], aContour[i][1], 
                  aContour[(i+1)%aContour.size()][0], aContour[(i+1)%aContour.size()][1]);
  }

    
  //construction of an AlphaThickSegmentComputer2D from the freemanchain iterator
  AlphaThickSegmentComputer2D anAlphaSegment, anAlphaSegment2, anAlphaSegment3;
  std::vector<Z2i::RealPoint>::const_iterator it2 =  aContour.begin();  
  
  anAlphaSegment.init(10);                           
  while (anAlphaSegment.extendFront(*it2)) {
    it2++;
  }

  aBoard << anAlphaSegment;  
  
  anAlphaSegment2.init(aContour.begin(), 5);
  while (anAlphaSegment2.extendFront()) {
  }
  aBoard  << CustomStyle( anAlphaSegment2.className(), new CustomColors( DGtal::Color::Blue, DGtal::Color::None ) );  
  aBoard << anAlphaSegment2;  

  anAlphaSegment3.init(aContour.begin(), 1);
  while (anAlphaSegment3.extendFront()) {
  }
  aBoard  << CustomStyle( anAlphaSegment3.className(), new CustomColors( DGtal::Color::Green, DGtal::Color::None ) );  
  aBoard << anAlphaSegment3;
  

  aBoard.saveEPS("exampleAlphaThickSegmentNoisy.eps");

  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
