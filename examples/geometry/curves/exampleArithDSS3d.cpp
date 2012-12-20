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
 * @file testArithDSS3dViewer.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 *
 * @date 2011/06/01
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testArithDSS3dViewer <p>
 * Aim: simple test of \ref ArithmeticalDSS3d
 */




#include <iostream>

#include <QtGui/qapplication.h>
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/CDrawableWithDisplay3D.h"

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/geometry/curves/SaturatedSegmentation.h"


using namespace std;
using namespace DGtal;
using namespace Z3i;


// Standard services - public :

int main( int argc, char** argv )
{


  typedef PointVector<3,int> Point;
  typedef std::vector<Point>::iterator Iterator;
  typedef ArithmeticalDSS3d<Iterator,int,4> SegmentComputer;  
  typedef SaturatedSegmentation<SegmentComputer> Decomposition;

  string inputFilename = examplesPath + "samples/sinus.dat"; 
  vector<Point> sequence = PointListReader<Point>::getPointsFromFile(inputFilename); 


  SegmentComputer algo;
  Decomposition theDecomposition(sequence.begin(), sequence.end(), algo);
  
  ///////////////////////////////////
  //display  
  bool flag = true;    
  #ifdef WITH_VISU3D_QGLVIEWER

  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();

  Point p;
  viewer  << SetMode3D(p.className(), "Grid");

    unsigned int c = 0;
    Decomposition::SegmentComputerIterator i = theDecomposition.begin();
    for ( ; i != theDecomposition.end(); ++i) {
      SegmentComputer currentSegmentComputer(*i);
       viewer << SetMode3D(currentSegmentComputer.className(), "Points"); 
      viewer << currentSegmentComputer;  
       viewer << SetMode3D(currentSegmentComputer.className(), "BoundingBox"); 
      viewer << currentSegmentComputer;  
      //cerr << currentSegmentComputer << endl;
      c++;
    } 
 
  viewer << Viewer3D::updateDisplay;
  flag = application.exec();
   #endif
  return flag;
}

