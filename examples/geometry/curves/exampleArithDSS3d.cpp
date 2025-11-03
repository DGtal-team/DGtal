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
 * @file geometry/curves/exampleArithDSS3d.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/06/01
 *
 * This file is part of the DGtal library
 */

/**
   This example outputs a greedy segmentation of an open and 6-connected 3d digital curve into DSSs. 
   
@verbatim
$ ./examples/geometry/curves/exampleArithDSS3d
@endverbatim

@image html exampleArithDSS3d.png "Greedy segmentation of a 3d digital curve into DSSs"
@image latex exampleArithDSS3d.png "Greedy segmentation of a 3d digital curve into DSSs"


@see \ref moduleGridCurveAnalysis

\example geometry/curves/exampleArithDSS3d.cpp
*/




#include <iostream>

#include "DGtal/io/viewers/PolyscopeViewer.h"

#include "DGtal/io/readers/PointListReader.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/StandardDSS6Computer.h"
#include "DGtal/geometry/curves/GreedySegmentation.h"


using namespace std;
using namespace DGtal;
using namespace Z3i;


// Standard services - public :

int main()
{

  typedef PointVector<3,int> Point;
  typedef std::vector<Point>::iterator Iterator;
  typedef StandardDSS6Computer<Iterator,int,4> SegmentComputer;  
  typedef GreedySegmentation<SegmentComputer> Decomposition;

  string inputFilename = examplesPath + "samples/sinus.dat"; 
  vector<Point> sequence = PointListReader<Point>::getPointsFromFile(inputFilename); 


  SegmentComputer algo;
  Decomposition theDecomposition(sequence.begin(), sequence.end(), algo);
  
  ///////////////////////////////////
  //display  
  Point p;

#ifdef DGTAL_WITH_POLYSCOPE_VIEWER

  PolyscopeViewer viewer;
  viewer.allowReuseList = true; // groups segments
  viewer.drawAsGrid();
#endif

  unsigned int c = 0;
  Decomposition::SegmentComputerIterator i = theDecomposition.begin();
  for ( ; i != theDecomposition.end(); ++i) {
    SegmentComputer currentSegmentComputer(*i);
     #ifdef DGTAL_WITH_POLYSCOPE_VIEWER
       viewer.drawAsBalls();
       viewer << currentSegmentComputer;  
       viewer.defaultStyle();
       viewer << currentSegmentComputer;  
    #endif
    c++;
  } 
  
  
  #ifdef DGTAL_WITH_POLYSCOPE_VIEWER
    viewer.show();
  #endif
    
}

