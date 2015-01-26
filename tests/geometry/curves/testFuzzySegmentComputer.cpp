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
 * @file testFuzzySegmentComputer.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/01/07
 *
 * Functions for testing class FuzzySegmentComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FuzzySegmentComputer.h"
#include "DGtal/io/readers/PointListReader.h"

#include <DGtal/io/boards/Board2D.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FuzzySegmentComputer.
///////////////////////////////////////////////////////////////////////////////


/**
 * Example of a test. To be completed.
 *
 */
bool testFuzzySegmentComputerFloatingPointContour()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  FuzzySegmentComputer<Z2i::Space, Z2i::RealPoint, double> FuzzySegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment on contour composed of floating coords ..." );
  std::vector<Z2i::RealPoint> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample2.sdp";
  aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile(fileContour);
  bool res = true;
  FuzzySegmentComputer2D aFuzzySegmentComp;
  aFuzzySegmentComp.init(4);
  
  unsigned int indexStart = 10;
  bool isExtending = true;
  isExtending &= aFuzzySegmentComp.extend(aContour[indexStart]);
  unsigned int i = 1;
  while (isExtending){
    isExtending &= aFuzzySegmentComp.extend(aContour[indexStart+i]);
    i++;
  }

  
  
  // Display boundingbox
  Z2i::RealPoint pt1, pt2, pt3, pt4;
  aBoard.setLineWidth(1);
  aFuzzySegmentComp.getRealBoundingBox(pt1,pt2,pt3,pt4);
  aBoard.setPenColor(DGtal::Color::Blue);
  aBoard.drawLine(pt1[0],pt1[1], pt2[0], pt2[1]);
  aBoard.drawLine(pt2[0],pt2[1], pt3[0], pt3[1]);
  aBoard.drawLine(pt3[0],pt3[1], pt4[0], pt4[1]);  
  aBoard.drawLine(pt4[0],pt4[1], pt1[0], pt1[1]);  
 
  
  // Display the input curve
  aBoard << SetMode((*aContour.begin()).className(), "Grid");
  for (std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); 
       it != aContour.end(); it++){
    aBoard << *it;
    if (it+1 != aContour.end()){
      Z2i::RealPoint next = *(it+1);
      aBoard.setPenColor(DGtal::Color::Gray);
      aBoard.drawLine((*it)[0], (*it)[1], next[0], next[1]);
    }
  }

  
  
  aBoard.saveEPS("testFuzzySegmentComputer_FloatingPt.eps"); 
  trace.info() << aFuzzySegmentComp;
  
  nbok += res ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}



/**
 * Example of a test. To be completed.
 *
 */
bool testFuzzySegmentComputer()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  FuzzySegmentComputer<Z2i::Space, Z2i::RealPoint, double> FuzzySegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment ..." );
  std::vector<Z2i::RealPoint> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample.sdp";
  aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile(fileContour);
  bool res = true;
  FuzzySegmentComputer2D aFuzzySegmentComp;
  aFuzzySegmentComp.init(4);
  FuzzySegmentComputer2D aFuzzySegmentComp2;
  aFuzzySegmentComp2.init(6);
  FuzzySegmentComputer2D aFuzzySegmentComp3;
  aFuzzySegmentComp3.init(1);
  FuzzySegmentComputer2D aFuzzySegmentComp4;
  aFuzzySegmentComp4.init(0.5);
  
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp.extend(*it);
    if (!res){
      break;
    }
  }
  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp2.extend(*it);
    if (!res){
      break;
    }
  }
  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp3.extend(*it);
    if (!res){
      break;
    }
  }
  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp4.extend(*it);
    if (!res){
      break;
    }
  }

  
  // Display convexhull
  std::vector<Z2i::RealPoint> aVect = aFuzzySegmentComp.getConvexHull();
  aBoard.setLineWidth(5);
  aBoard.setPenColor(DGtal::Color::Red);
  for (unsigned int i = 0; i < aVect.size(); i++){
    aBoard.drawLine(aVect.at(i)[0], aVect.at(i)[1], 
                    aVect.at((i+1)%aVect.size())[0],
                    aVect.at((i+1)%aVect.size())[1]);    
  } 
  
  // Display boundingbox
  Z2i::RealPoint pt1, pt2, pt3, pt4;
  aBoard.setLineWidth(1);
  aFuzzySegmentComp.getRealBoundingBox(pt1,pt2,pt3,pt4);
  aBoard.setPenColor(DGtal::Color::Blue);
  trace.info() << "Bounding box of the segment:" << std::endl;
  trace.info() << "pt1 :" << pt1 << std::endl;
  trace.info() << "pt2 :" << pt2 << std::endl;
  trace.info() << "pt3 :" << pt3 << std::endl;
  trace.info() << "pt4 :" << pt4 << std::endl;
  aBoard.drawLine(pt1[0],pt1[1], pt2[0], pt2[1]);
  aBoard.drawLine(pt2[0],pt2[1], pt3[0], pt3[1]);
  aBoard.drawLine(pt3[0],pt3[1], pt4[0], pt4[1]);  
  aBoard.drawLine(pt4[0],pt4[1], pt1[0], pt1[1]);  
 
  aFuzzySegmentComp2.getRealBoundingBox(pt1,pt2,pt3,pt4);
  aBoard.drawLine(pt1[0],pt1[1], pt2[0], pt2[1]);
  aBoard.drawLine(pt2[0],pt2[1], pt3[0], pt3[1]);
  aBoard.drawLine(pt3[0],pt3[1], pt4[0], pt4[1]);  
  aBoard.drawLine(pt4[0],pt4[1], pt1[0], pt1[1]);  
  
  aFuzzySegmentComp3.getRealBoundingBox(pt1,pt2,pt3,pt4);
  aBoard.drawLine(pt1[0],pt1[1], pt2[0], pt2[1]);
  aBoard.drawLine(pt2[0],pt2[1], pt3[0], pt3[1]);
  aBoard.drawLine(pt3[0],pt3[1], pt4[0], pt4[1]);  
  aBoard.drawLine(pt4[0],pt4[1], pt1[0], pt1[1]);  

  aFuzzySegmentComp4.getRealBoundingBox(pt1,pt2,pt3,pt4);
  aBoard.drawLine(pt1[0],pt1[1], pt2[0], pt2[1]);
  aBoard.drawLine(pt2[0],pt2[1], pt3[0], pt3[1]);
  aBoard.drawLine(pt3[0],pt3[1], pt4[0], pt4[1]);  
  aBoard.drawLine(pt4[0],pt4[1], pt1[0], pt1[1]);  
  
  
  // Display the input curve
  aBoard << SetMode((*aContour.begin()).className(), "Grid");
  for (std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); 
       it != aContour.end(); it++){
    aBoard << *it;
    if (it+1 != aContour.end()){
      Z2i::RealPoint next = *(it+1);
      aBoard.setPenColor(DGtal::Color::Gray);
      aBoard.drawLine((*it)[0], (*it)[1], next[0], next[1]);
    }
  }

  // Display the Fuzzy segement
  aBoard << SetMode((*aContour.begin()).className(), "Grid");
  for (FuzzySegmentComputer2D::ConstIterator it = aFuzzySegmentComp.begin(); 
       it != aFuzzySegmentComp.end(); it++){
      aBoard.setPenColor(DGtal::Color::Gray);
      aBoard.drawCircle((*it)[0], (*it)[1], 0.02);
  }
  
  
  aBoard.saveEPS("testFuzzySegmentComputer_Convexhull.eps"); 
  trace.info() << aFuzzySegmentComp;
  res = aFuzzySegmentComp.size()==45;
  
  nbok += res ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
               << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FuzzySegmentComputer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testFuzzySegmentComputer() && testFuzzySegmentComputerFloatingPointContour() ; // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
