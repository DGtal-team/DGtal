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
 * @file testFrechetShortcut.cpp
 * @ingroup Tests
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/03/26
 *
 * Functions for testing class FrechetShortcut.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iterator>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <vector>

#include "DGtal/base/Common.h"
#include "DGtal/base/Exceptions.h"
#include "DGtal/kernel/SpaceND.h"

#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/curves/FrechetShortcut.h"
#include "DGtal/io/boards/Board2D.h"

#include "DGtal/io/boards/CDrawableWithBoard2D.h"
#include "DGtal/geometry/curves/CForwardSegmentComputer.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/geometry/curves/GreedySegmentation.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace LibBoard;
using namespace Z2i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class FrechetShortcut.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testFrechetShortcut()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  typedef PointVector<2,int> Point;
  typedef std::vector<Point>::iterator Iterator;
  typedef FrechetShortcut<Iterator,int> Shortcut;

  std::vector<Point> contour;
  contour.push_back(Point(0,0));
  contour.push_back(Point(1,0));
  contour.push_back(Point(2,0));
  contour.push_back(Point(3,0));
  contour.push_back(Point(4,0));
  contour.push_back(Point(5,0));
  contour.push_back(Point(6,0));
  contour.push_back(Point(7,0));
  contour.push_back(Point(7,1));
  contour.push_back(Point(6,1));
  contour.push_back(Point(5,1));
  contour.push_back(Point(4,1));
  contour.push_back(Point(3,1));
  contour.push_back(Point(2,1));
  contour.push_back(Point(3,2));
  contour.push_back(Point(4,2));
  contour.push_back(Point(5,2));
  contour.push_back(Point(6,2));
  contour.push_back(Point(7,2));
  contour.push_back(Point(8,2));
  contour.push_back(Point(9,2));

  trace.beginBlock ( "Testing block ..." );

  Shortcut s(5);
  s.init(contour.begin());

  trace.info() << s << std::endl;

  while ( (s.end() != contour.end())
    &&(s.extendFront()) ) {}

  trace.info() << s << std::endl;


  trace.endBlock();

  return nbok == nb;
}



void testFrechetShortcutConceptChecking()
{
  typedef PointVector<2,int> Point;
  typedef std::vector<Point>::const_iterator ConstIterator;
  typedef FrechetShortcut<ConstIterator,int> Shortcut;
  BOOST_CONCEPT_ASSERT(( concepts::CDrawableWithBoard2D<Shortcut> ));
  BOOST_CONCEPT_ASSERT(( concepts::CForwardSegmentComputer<Shortcut> ));
}

bool testSegmentation()
{
  typedef PointVector<2,int> Point;

  std::vector<Point> contour;
  contour.push_back(Point(0,0));
  contour.push_back(Point(1,0));
  contour.push_back(Point(2,0));
  contour.push_back(Point(3,0));
  contour.push_back(Point(4,0));
  contour.push_back(Point(5,0));
  contour.push_back(Point(6,0));
  contour.push_back(Point(7,0));
  contour.push_back(Point(7,1));
  contour.push_back(Point(6,1));
  contour.push_back(Point(5,1));
  contour.push_back(Point(4,1));
  contour.push_back(Point(3,1));
  contour.push_back(Point(2,1));
  contour.push_back(Point(2,2));
  contour.push_back(Point(3,2));
  contour.push_back(Point(4,2));
  contour.push_back(Point(5,2));
  contour.push_back(Point(6,2));
  contour.push_back(Point(7,2));
  contour.push_back(Point(8,2));
  contour.push_back(Point(9,2));

  trace.beginBlock ( "Testing block ..." );

  typedef Curve::PointsRange::ConstIterator Iterator;
  typedef FrechetShortcut<Iterator,int> SegmentComputer;

  Curve aCurve; //grid curve
  aCurve.initFromPointsVector(contour);

  typedef Curve::PointsRange Range; //range
  Range r = aCurve.getPointsRange(); //range

  Board2D board;
  board << r;
  board << aCurve.getArrowsRange();

  // Test when error = 3


  int nbok = 3;
  int nb=0;
  trace.beginBlock ( "Greedy segmentation" );
  {
    typedef GreedySegmentation<SegmentComputer> Segmentation;
    Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer(3) );

    Segmentation::SegmentComputerIterator it = theSegmentation.begin();
    Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();

    for ( ; it != itEnd; ++it) {
      SegmentComputer s(*it);
      trace.info() << s << std::endl;
      board << (*it);
      nb++;
    }
    trace.info() << theSegmentation << std::endl;
    board.saveEPS("FrechetShortcutGreedySegmentationTest.eps", Board2D::BoundingBox, 5000 );
  }

  // test when error = 0


  int nbok2 = 5;
  int nb2=0;
  trace.beginBlock ( "Greedy segmentation" );
  {
    typedef GreedySegmentation<SegmentComputer> Segmentation;
    Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer(0) );

    Segmentation::SegmentComputerIterator it = theSegmentation.begin();
    Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();

    for ( ; it != itEnd; ++it) {
      SegmentComputer s(*it);
      trace.info() << s << std::endl;
      board << (*it);
      nb2++;
    }
    trace.info() << theSegmentation << std::endl;
    board.saveEPS("FrechetShortcutGreedySegmentationTest.eps", Board2D::BoundingBox, 5000 );
  }


  /* Saturated segmentation does not work for FrechetShortcut
     computer. Indeed, given two maximal Frechet shortcuts s1(begin, end) et
     s2(begin, end),  we can have s1.begin < s2.begin < s2.end <
     s1.end. */


  trace.endBlock();

  return (nbok == nb) && (nbok2==nb2);
}



bool testSegmentationLarger(const string& filename, int min, int max, double delta)
{
  // Build output filename

  std::size_t pos1 = filename.find("samples/");
  std::size_t init = pos1+8;
  std::size_t pos2 = filename.find(".dat");
  std::size_t end = pos2-init;
  std::string output = filename.substr(init,end);

  trace.beginBlock ( "Testing block ..." );

  trace.beginBlock ( "Greedy segmentation on larger contours" );

  trace.info() << "Reading input curve" << filename << std::endl;

  typedef Curve::PointsRange::ConstIterator Iterator;

  Curve aCurve; //grid curve

  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  aCurve.initFromVectorStream(instream);


  typedef Curve::PointsRange Range; //range
  Range r = aCurve.getPointsRange(); //range

  Board2D board;
  board << r;
  board << aCurve.getArrowsRange();

  trace.info() << "Size of input curve = " << aCurve.size() << std::endl;

  typedef Curve::PointsRange::ConstIterator Iterator;
  typedef FrechetShortcut<Iterator,int> SegmentComputer;
  typedef GreedySegmentation<SegmentComputer> Segmentation;

  for(double error = min; error <= max; error+=delta)
    {
      trace.info() << "error = " << error << "\t";
      Segmentation theSegmentation( r.begin(), r.end(), SegmentComputer(error) );

      Segmentation::SegmentComputerIterator it = theSegmentation.begin();
      Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();

      int size = 0;
      for ( ; it != itEnd; ++it)
	size++;

      trace.info() << "size of simplified curve = " << size << std::endl;

      it = theSegmentation.begin();
      for ( ; it != itEnd; ++it)
	{
	  SegmentComputer s(*it);
	  board << (*it);
	}
      // save simplified curves in eps file
      string outputFilename = "FrechetShortcut-"+output+".eps";
      board.saveEPS(outputFilename.c_str(), Board2D::BoundingBox, 5000 );
    }

  trace.endBlock();
  trace.endBlock();

  return 1;
}




///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class FrechetShortcut" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  testFrechetShortcutConceptChecking();


  std::string Plant054 = testPath + "samples/Plant054.dat";
  std::string beetle = testPath + "samples/beetle-1.dat";


  bool res = testFrechetShortcut() && testSegmentation() && testSegmentationLarger(Plant054,0,20,0.5) && testSegmentationLarger(beetle,0,30,5); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
