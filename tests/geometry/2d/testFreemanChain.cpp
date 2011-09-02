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
 * Test Constructors
 */
bool testConstructors() 
{
  typedef FreemanChain<int> FreemanChain;
  typedef FreemanChain::ConstIterator Iterator;

  trace.beginBlock ( "Testing FreemanChain constructors" );

  trace.info() << "Constructor from string and coordintes" << endl;
  std::string s = "00001030003222321222";
  FreemanChain c1(s, -42, 12);

  trace.info() << "Constructor from point vector" << endl;
  std::vector<Z2i::Point> myVector;
  for (Iterator i = c1.begin(); i != c1.end(); i++)
    myVector.push_back(*i);
//  myVector.push_back(*c1.end());
  FreemanChain c2(myVector);

  trace.info() << "Constructor from input stream" << endl;
  std::stringstream ss;
  ss << "-42 12 " << s << "\n";
  FreemanChain c3(ss);

  trace.info() << "Copy constructor" << endl;
  FreemanChain c4(c1);

  trace.info() << "Copy operator" << endl;
  FreemanChain c5("0123" , 0, 0);
  FreemanChain c6(c5);

  c5 = c1;

  cout << "c1 " << c1 << endl;
  cout << "c2 " << c2 << endl;
  cout << "c3 " << c3 << endl;
  cout << "c4 " << c4 << endl;
  cout << "c5 " << c5 << endl;
  cout << "c6 " << c6 << endl;

  bool res = (
         (c1 == c2) && (c1 == c3) && (c1 == c4)// && (c1 == c5) && (c1 != c6)
//      && (c2 == c1) && (c2 == c3) && (c2 == c4) && (c2 == c5) && (c2 != c6)
//      && (c3 == c1) && (c3 == c2) && (c3 == c4) && (c3 == c5) && (c3 != c6)
//      && (c4 == c1) && (c4 == c2) && (c4 == c3) && (c4 == c5) && (c4 != c6)
//      && (c5 == c1) && (c5 == c2) && (c5 == c3) && (c5 == c4) && (c4 != c6)
//      && (c6 != c1) && (c6 != c2) && (c6 != c3) && (c6 != c4) && (c6 != c5)
      );
  cout << "yep" << endl;
  trace.endBlock();
  return res;
}





   








/**
 * test reverse iterator
 */
bool testFreemanChainIterator(const std::string& code)
{
  typedef int Coordinate;
  typedef FreemanChain<Coordinate> Sequence;
  typedef Sequence::PointI2 Point;
  typedef Sequence::ConstIterator SequenceIterator;
  typedef Sequence::ConstCharIterator CharIterator;
  typedef std::reverse_iterator<SequenceIterator> ReverseIterator;
   
  trace.beginBlock ( "Testing FreemanChain Iterator" );

  std::stringstream ss;
  ss << code << std::endl;
  Sequence seq(ss);

  trace.info()<< "Freeman chain set to " << code << endl;   
  trace.info()<< seq << endl;
  trace.info()<< "Iterates on points." << endl;
  std::stack<Point> myStack;

  int nbPts = 0;
  for (SequenceIterator i = seq.begin(); i != seq.end(); ++i) 
  {
    myStack.push(*i);
    nbPts++;
  }

  trace.info()<< "Test reverse iterator." << endl;
  bool samePoints = true;
  for (ReverseIterator ri(seq.end()); 
      ri != ReverseIterator(seq.begin());
      ++ri) 
  {
    if ( !myStack.empty() && ( *ri == myStack.top() ) )
    {
      myStack.pop();
    } 
    else
    {
      samePoints = false;
      break;
    }
  }
  trace.endBlock();
  return myStack.empty() && samePoints && ( nbPts == seq.size() + 1);
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
  std::string chain = "0 0 000011112222333";
  
  bool res = testConstructors();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;


  res = testFreemanChainIterator(chain) && testFreemanChain(chain);
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;

  std::string filename = testPath + "samples/contourS.fc";
  std::cout << filename << std::endl;
  testDisplayFreemanChain(filename);
  trace.endBlock();
  
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
