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
 * @file testAlphaThickSegmentComputer.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/01/07
 *
 * Functions for testing class AlphaThickSegmentComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/AlphaThickSegmentComputer.h"
#include "DGtal/io/readers/PointListReader.h"

#include <DGtal/io/boards/Board2D.h>
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class AlphaThickSegmentComputer.
///////////////////////////////////////////////////////////////////////////////


/**
 * Example of a test. To be completed.
 *
 */
bool testAlphaThickSegmentComputerFloatingPointContour()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::RealPoint, double> AlphaThickSegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment on contour composed of floating coords ..." );
  std::vector<Z2i::RealPoint> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample2.sdp";
  aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile(fileContour);
  
  AlphaThickSegmentComputer2D aFuzzySegmentComp;
  aFuzzySegmentComp.init(aContour.begin()+10, 1);
  
  bool isExtending = true;
  while (isExtending){
    isExtending &= aFuzzySegmentComp.extendFront();
  }

    
 
  
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
  // Display segment 
  aBoard << SetMode((*aFuzzySegmentComp.begin()).className(), "Grid"); 
  aBoard << aFuzzySegmentComp;



  trace.info() << "Segment size: " << aFuzzySegmentComp.getNumberSegmentPoints() << std::endl;  
  aBoard.saveEPS("testAlphaThickSegmentComputer_FloatingPt.eps"); 
  
  nbok += aFuzzySegmentComp.getNumberSegmentPoints()==30 ? 1 : 0; 
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
bool testAlphaThickSegmentComputer()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::RealPoint, double > AlphaThickSegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment ..." );
  std::vector<Z2i::RealPoint> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample.sdp";
  aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile(fileContour);
  bool res = true;
  AlphaThickSegmentComputer2D aFuzzySegmentComp;
  aFuzzySegmentComp.init(aContour.begin(),4);
  AlphaThickSegmentComputer2D aFuzzySegmentComp2;
  aFuzzySegmentComp2.init(6);
  AlphaThickSegmentComputer2D aFuzzySegmentComp3;
  aFuzzySegmentComp3.init(aContour.begin(),1);
  AlphaThickSegmentComputer2D aFuzzySegmentComp4;
  aFuzzySegmentComp4.init(aContour.begin(), 0.5);
  
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp.extendFront();
    if (!res){
      break;
    }
  }
  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp2.extendFront(*it);
    if (!res){
      break;
    }
  }


  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp3.extendFront();
    if (!res){
      break;
    }
  }
  res=true;
  for (  std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); it!= aContour.end();  it++){ 
    res &= aFuzzySegmentComp4.extendFront();
    if (!res){
      break;
    }
  }


  
  // Display convexhull
  std::vector<Z2i::RealPoint> aVect = aFuzzySegmentComp.getConvexHull();
  aBoard.setLineWidth(5);
  aBoard.setPenColor(DGtal::Color::Blue);
  for (unsigned int i = 0; i < aVect.size(); i++){
    aBoard.drawLine(aVect.at(i)[0], aVect.at(i)[1], 
                    aVect.at((i+1)%aVect.size())[0],
                    aVect.at((i+1)%aVect.size())[1]);    
  } 





  
  // Display boundingbox
  // aBoard << SetMode(aFuzzySegmentComp.className(), "BoundingBox"); 
 aBoard  << CustomStyle( aFuzzySegmentComp.className(), new CustomColors( DGtal::Color::Green, DGtal::Color::None ) );  
  aBoard << aFuzzySegmentComp2;
 
  aBoard  << CustomStyle( aFuzzySegmentComp.className(), new CustomColors( DGtal::Color::Blue, DGtal::Color::None ) );      
  aBoard << aFuzzySegmentComp;
  aBoard  << CustomStyle( aFuzzySegmentComp.className(), new CustomColors( DGtal::Color::Yellow, DGtal::Color::None ) );  
  aBoard << aFuzzySegmentComp3;
  aBoard  << CustomStyle( aFuzzySegmentComp.className(), new CustomColors( DGtal::Color::Magenta, DGtal::Color::None ) );  
  aBoard << aFuzzySegmentComp4;
  
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

  
  aBoard.saveEPS("testAlphaThickSegmentComputer_Convexhull.eps"); 
  trace.info() << aFuzzySegmentComp << "size:"<< aFuzzySegmentComp.getNumberSegmentPoints();
  res = aFuzzySegmentComp.getNumberSegmentPoints()==45;
  
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
bool testAlphaThickSegmentInt()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::Point, int> AlphaThickSegmentComputer2D;
  trace.beginBlock ( "Testing Fuzzy segment on contour composed of int coords ..." );
  std::vector<Z2i::Point> aContour;
  std::string fileContour = testPath + "samples/klokan.sdp";
  aContour = PointListReader<Z2i::Point>::getPointsFromFile(fileContour);
  
  AlphaThickSegmentComputer2D aFuzzySegmentComp;
  aFuzzySegmentComp.init(aContour.begin()+1340,  10.0);
  

  while (aFuzzySegmentComp.extendFront()){

  }

  trace.info() << aFuzzySegmentComp;
  // Display the input curve
  aBoard << SetMode((*aContour.begin()).className(), "Grid");
  for (std::vector<Z2i::Point>::const_iterator it = aContour.begin(); 
       it != aContour.end(); it++){
    aBoard << *it;
    if (it+1 != aContour.end()){
      Z2i::Point next = *(it+1);
      aBoard.setPenColor(DGtal::Color::Gray);
      aBoard.drawLine((*it)[0], (*it)[1], next[0], next[1]);
    }
  }
    
  // Display segment 
  aBoard << SetMode((*aFuzzySegmentComp.begin()).className(), "Grid"); 
  aBoard << aFuzzySegmentComp;
 
  
  trace.info() << "Segment size: " << aFuzzySegmentComp.getNumberSegmentPoints() << std::endl;  
  aBoard.saveEPS("testAlphaThickSegmentComputer_Int.eps"); 
  
  nbok += aFuzzySegmentComp.getNumberSegmentPoints()==153 ? 1 : 0; 
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
  trace.beginBlock ( "Testing class AlphaThickSegmentComputer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testAlphaThickSegmentComputer() && testAlphaThickSegmentComputerFloatingPointContour() 
    && testAlphaThickSegmentInt(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;

  
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
