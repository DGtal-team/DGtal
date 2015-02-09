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
 * Test alpha thick segment with floating points.
 */
bool testAlphaThickSegmentComputerFloatingPointContour()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  Board2D aBoard;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::RealPoint, double> AlphaThickSegmentComputer2D;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::RealPoint, double>::Primitive Primitive;

  trace.beginBlock ( "Testing alpha thick segment on contour composed of floating coords ..." );
  std::vector<Z2i::RealPoint> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample2.sdp";
  aContour = PointListReader<Z2i::RealPoint>::getPointsFromFile(fileContour);
  
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer;
  anAlphaThickSegmentComputer.init(aContour.begin()+10, 1);
  
  while (anAlphaThickSegmentComputer.extendFront()){
  }

  nbok += anAlphaThickSegmentComputer.getNumberSegmentPoints()==30 ? 1 : 0; 
  nb++;
  
  // Display alpha thick segment 
  aBoard << SetMode((*anAlphaThickSegmentComputer.begin()).className(), "Grid"); 
  aBoard << anAlphaThickSegmentComputer;
  
   
  // Test primitive of parallelStrip and display the input curve
  Primitive pStrip = anAlphaThickSegmentComputer.primitive();
  unsigned int nbInStrip = 0;
  
  aBoard << SetMode((*aContour.begin()).className(), "Grid");
  for (std::vector<Z2i::RealPoint>::const_iterator it = aContour.begin(); 
       it != aContour.end(); it++){
    if (it+1 != aContour.end()){
      Z2i::RealPoint next = *(it+1);
      aBoard.setLineWidth(2);
      aBoard.setPenColor(DGtal::Color::Gray);
      aBoard.drawLine((*it)[0], (*it)[1], next[0], next[1]);
    }
    if (pStrip(*it)){
      nbInStrip++;
      aBoard << *it; 
    }    
  }
  trace.info() << "Nb contour points in the segment parallel strip (awaited 31)  = " << nbInStrip << std::endl;

  nbok += nbInStrip==31;
  nb++;

  trace.info() << "Segment size (awaited 30): " << anAlphaThickSegmentComputer.getNumberSegmentPoints() << std::endl;  
  aBoard.saveEPS("testAlphaThickSegmentComputer_FloatingPt.eps"); 
  
  nbok += anAlphaThickSegmentComputer.getNumberSegmentPoints()==30 ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
  trace.endBlock(); 
  return nbok == nb;
}




/**
 * Test the convexhull and box of a the alpha thick segment computer.
 */
bool testAlphaThickSegmentConvexHullAndBox()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  Board2D aBoard;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::Point, int > AlphaThickSegmentComputer2D;
  trace.beginBlock ( "Testing convexhull and boxes of alphaThick segment on noisy discrete contour." );
  std::vector<Z2i::Point> aContour;
  std::string fileContour = testPath + "samples/contourNoiseSample.sdp";
  aContour = PointListReader<Z2i::Point>::getPointsFromFile(fileContour);
  bool res = true;
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer;
  anAlphaThickSegmentComputer.init(aContour.begin(),4);
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer2;
  anAlphaThickSegmentComputer2.init(aContour.begin(),6);
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer3;
  anAlphaThickSegmentComputer3.init(aContour.begin(),1);
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer4;
  anAlphaThickSegmentComputer4.init(aContour.begin(), 0.5);
  
  unsigned i=0;
  while (anAlphaThickSegmentComputer.extendFront() && i<aContour.size()-1){i++;}
  i=0;
  while (anAlphaThickSegmentComputer2.extendFront() && i<aContour.size()-1){i++;}
  i=0;
  while (anAlphaThickSegmentComputer3.extendFront() && i<aContour.size()-1){i++;}
  i=0;
  while (anAlphaThickSegmentComputer4.extendFront() && i<aContour.size()-1){i++;}
    
  // Display convexhull
  std::vector<Z2i::Point> aVect = anAlphaThickSegmentComputer.getConvexHull();
  aBoard.setLineWidth(5);
  aBoard.setPenColor(DGtal::Color::Purple);
  for (unsigned int i = 0; i < aVect.size(); i++){
    aBoard.drawLine(aVect.at(i)[0], aVect.at(i)[1], 
                    aVect.at((i+1)%aVect.size())[0],
                    aVect.at((i+1)%aVect.size())[1]);    
  } 
  
  // Display boundingbox
  aBoard << SetMode((*anAlphaThickSegmentComputer.begin()).className(), "Grid"); 
  aBoard << CustomStyle( anAlphaThickSegmentComputer2.className(),
                         new CustomColors( DGtal::Color::Green, DGtal::Color::None ) );  
  aBoard << anAlphaThickSegmentComputer2; 
  aBoard << CustomStyle( anAlphaThickSegmentComputer.className(), 
                         new CustomColors( DGtal::Color::Blue, DGtal::Color::None ) );      
  aBoard << anAlphaThickSegmentComputer;
  aBoard << CustomStyle( anAlphaThickSegmentComputer3.className(), 
                         new CustomColors( DGtal::Color::Yellow, DGtal::Color::None ) );  
  aBoard << anAlphaThickSegmentComputer3;
  aBoard << CustomStyle( anAlphaThickSegmentComputer4.className(), 
                         new CustomColors( DGtal::Color::Magenta, DGtal::Color::None ) );  
  aBoard << anAlphaThickSegmentComputer4;
  

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
  
  aBoard.saveEPS("testAlphaThickSegmentComputer_Convexhull.eps"); 
  trace.info() << " Alpha Thick with alpha 4, size (awaited be 45) = " << anAlphaThickSegmentComputer.getNumberSegmentPoints();
  res = anAlphaThickSegmentComputer.getNumberSegmentPoints()==45;
  
  nbok += res ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
  trace.endBlock();
  return nbok == nb;
}



/**
 * Test  alpha thick segment computer on Freeman Chain.
 */
bool testAlphaThickSegmentFreeman()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef FreemanChain<Z2i::Space::Integer>::ConstIterator FCConstIterator;
  typedef  AlphaThickSegmentComputer<Z2i::Space, Z2i::Point, int, FCConstIterator > AlphaThickSegmentComputer2D;

  Board2D aBoard;

  trace.beginBlock ( "Testing AlphaThickSegmentComputer2D on Freeman chain ..." );  

  // Reading input contour
  std::string freemanChainFilename = testPath + "samples/klokan.fc";
  fstream fst;
  fst.open (freemanChainFilename.c_str(), ios::in);
  FreemanChain<Z2i::Space::Integer> fc(fst);
  fst.close();
  aBoard << fc;
  
  
  AlphaThickSegmentComputer2D anAlphaThickSegmentComputer;
  anAlphaThickSegmentComputer.init(fc.begin(),  40.0);
  while (anAlphaThickSegmentComputer.extendFront()){
  }

  
  // Display segment 
  aBoard << SetMode((*anAlphaThickSegmentComputer.begin()).className(), "Grid"); 
  aBoard << anAlphaThickSegmentComputer;
  trace.info() << "Segment size (awaited 638): " << anAlphaThickSegmentComputer.getNumberSegmentPoints() << std::endl;  
  nbok += anAlphaThickSegmentComputer.getNumberSegmentPoints()==638 ? 1 : 0; 
  nb++;
  
  
  // Display points inside the parallel strip
  unsigned int nbInStrip = 0;
  AlphaThickSegmentComputer2D::Primitive parallelStrip = anAlphaThickSegmentComputer.primitive();

  for(FCConstIterator it  = fc.begin(); it != fc.end(); it++){
    if(parallelStrip(*it)){
      aBoard.setPenColor(DGtal::Color::Blue);
      aBoard.drawCircle( (*it)[0],(*it)[1], 1.0) ;
      nbInStrip++;
    }
  }
  aBoard.setPenColor(DGtal::Color::Green);
  aBoard.fillCircle( (*(fc.begin()))[0],(*(fc.begin()))[1], 1.0) ;
      
  trace.info() << "Nb contour points in the segment parallel strip  (awaited 758)  = " << nbInStrip << std::endl;
  nbok +=  nbInStrip==758 ? 1 : 0; 
  nb++;
  
  trace.info() << "(" << nbok << "/" << nb << ") " << std::endl;
  trace.endBlock();
  aBoard.saveEPS("testAlphaThickSegmentComputer_Freeman.eps"); 
  
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

  bool res = testAlphaThickSegmentConvexHullAndBox() && testAlphaThickSegmentComputerFloatingPointContour() 
    && testAlphaThickSegmentFreeman();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;

  
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
