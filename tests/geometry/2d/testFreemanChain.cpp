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
 * @file testFreemanChain.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/07/05
 *
 * Functions for testing class FreemanChain.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/program_options.hpp>
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/base/Common.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/io/boards/Board2D.h"
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FreemanChain.
///////////////////////////////////////////////////////////////////////////////
typedef SpaceND<2> Space2Type;
typedef HyperRectDomain<Space2Type> Domain2D;
typedef Space2Type::Point Point;

/**
 * test reverse iterator
 *
 */
bool testFreemanChainIterator(const std::string& code)
{


  typedef int Coordinate;
	typedef FreemanChain<Coordinate> Sequence;
  typedef Sequence::ConstIterator SequenceIterator;
	typedef std::reverse_iterator<SequenceIterator> ReverseIterator;
  
  trace.beginBlock ( "Testing FreemanChain Iterator" );
  
  std::stringstream ss;
  ss << code << std::endl;
  Sequence seq(ss);

  trace.info()<< "Freeman chain set to " << code << endl;   
 
trace.info()<< "<" << endl;  
  for (SequenceIterator i = seq.begin(); i != seq.end(); ++i) {
		trace.info()<< *i << " "  << i.getPosition() << " "; 
	}
		trace.info()<< endl; 

trace.info()<< ">" << endl;  
  

  for (ReverseIterator ri(seq.end()); ri != ReverseIterator(seq.begin()); ++ri) {
		trace.info()<< *ri << " "; 
	}
		trace.info()<< endl; 

  trace.endBlock();
	return true;
}


/**
 * Example of a test. To be completed.
 *
 */
bool testFreemanChain(const string& code)
{
  unsigned int nbok = 0;
  unsigned int nb = 4;
  
  
  trace.beginBlock ( "Testing FreemanChain " );
  
  std::stringstream ss;
  ss << code << std::endl;
  FreemanChain<int> fc(ss);  

  nbok += 1;   
  trace.info()<< "Freeman chain set to " << code << endl; 
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Reading FreemanChain" << std::endl;
  
  
  trace.info() << "isClosed():" << fc.isClosed()<< endl;
  nbok += 1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test is Closed" << std::endl;
  
  
  int minX, maxX, minY, maxY;
  fc.computeBoundingBox(minX, minY, maxX, maxY);  
  trace.info()<< "Freeman chain bounding box: " << minX << " " << minY << " " << maxX << " " << maxY << endl ; 
  nbok += 1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test chain bounding box" << std::endl;
  
  
  vector<FreemanChain<int>::PointI2> aContourPointVector; 
  fc.getContourPoints(fc, aContourPointVector);
  trace.info() << "List of point: ";
  for (unsigned int i =0; i < aContourPointVector.size(); i++){
    trace.info()<< "(" << aContourPointVector.at(i).at(0) << "," << aContourPointVector.at(i).at(1) << ")";
  }
  
  trace.info()<< endl;
  nbok+=1;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Test extracting list of contour point" << std::endl;
      
  trace.endBlock();

  return nbok == nb;
}







/**
 * Example of a test. To be completed.
 *
 */
bool testDisplayFreemanChain(const string &file)
{
  Board2D aBoard;
  aBoard.setUnit(Board::UCentimeter);
  
  fstream fst;
  fst.open (file.c_str(), ios::in);
  FreemanChain<int> fc(fst);  
  aBoard.setPenColor(Color::Red);
  
  //aBoard << DrawPavingPixel();
  
  aBoard << fc;
  fst.close();
  
  std::string filenameImage = testPath + "samples/contourS.png"; // ! only PNG with Cairo for the moment !
  LibBoard::Image image(0,84, 185, 85, filenameImage, 20); 
  image.shiftDepth(1);
  LibBoard::Board & board = aBoard;
  board << image;
  
  aBoard.saveSVG( "testDisplayFC.svg", Board::BoundingBox, 5000);
  aBoard.saveEPS( "testDisplayFC.eps", Board::BoundingBox, 5000 );
  aBoard.saveFIG( "testDisplayFC.fig", Board::BoundingBox, 5000 );
  
#ifdef WITH_CAIRO
  aBoard.saveCairo("testDisplayFC-cairo.pdf", Board2D::CairoPDF, Board::BoundingBox, 5000);
  aBoard.saveCairo("testDisplayFC-cairo.png", Board2D::CairoPNG, Board::BoundingBox, 5000);
  aBoard.saveCairo("testDisplayFC-cairo.ps", Board2D::CairoPS, Board::BoundingBox, 5000);
  aBoard.saveCairo("testDisplayFC-cairo.svg", Board2D::CairoSVG, Board::BoundingBox, 5000);
#endif
  
  return true;
}





///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FreemanChain" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;
  std::string chain = "0 0 0000111122223333";
  

  bool res = testFreemanChainIterator(chain)
							&& testFreemanChain(chain);
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;


  std::string filename = testPath + "samples/contourS.fc";
  std::cout << filename << std::endl;
  testDisplayFreemanChain(filename);
  trace.endBlock();
  
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
