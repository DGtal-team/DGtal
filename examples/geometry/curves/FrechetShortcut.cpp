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
 * @file FrechetShortcut.cpp
 * @ingroup Examples
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/03/26
 *
 * An example file named FrechetShortcut.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "ConfigExamples.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/geometry/curves/FrechetShortcut.h"
#include "DGtal/geometry/curves/GreedySegmentation.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;




///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example FrechetShortcut" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  std::string filename = examplesPath + "samples/plant-frechet.dat";
  ifstream instream; // input stream
  instream.open (filename.c_str(), ifstream::in);

  
  double error = atof(argv[1]);
  trace.info() << error << endl;
  
  Curve c; //grid curve
  c.initFromVectorStream(instream);
  
  Board2D board;
  
  board << c.getArrowsRange();

  trace.beginBlock("Simple example");

  //! [FrechetShortcutUsage]
  Curve::PointsRange r = c.getPointsRange(); 
  
  typedef FrechetShortcut<Curve::PointsRange::ConstIterator,int> Shortcut;
  
  typedef GreedySegmentation<Shortcut> Segmentation;
  Segmentation theSegmentation( r.begin(), r.end(), Shortcut(error) );
  
  Segmentation::SegmentComputerIterator it = theSegmentation.begin();
  Segmentation::SegmentComputerIterator itEnd = theSegmentation.end();

  Shortcut s;
  
  for ( ; it != itEnd; ++it) {
    s=Shortcut(*it);
    trace.info() << s << std::endl;
    board << s; 
    //   //   for (Iterator i = it->begin(); i != it->end(); ++i)
  }
  
  board.saveEPS("FrechetShortcutExample.eps", Board2D::BoundingBox, 5000 ); 

  //extension
  //  s.init( r.begin() );
  //while ( ( s.end() != itEnd )
  //    &&( s.extendForward() ) ) {}
  //! [FrechetShortcutUsage]


  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
