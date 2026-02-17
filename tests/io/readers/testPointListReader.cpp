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
 * @file testPointListReader.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/04/01
 *
 * Functions for testing class PointListReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/curves/FreemanChain.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PointListReader.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testPointListReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing reading point list ..." );
  std::string filename = testPath + "samples/pointList1.pl";
  std::vector<unsigned int> vectPos;
  vectPos.push_back(1);
  vectPos.push_back(2);
  vector<Z2i::Point> vectPoints = PointListReader<Z2i::Point>::getPointsFromFile(filename,
                     vectPos);
  for(unsigned int k=0;k < vectPoints.size(); k++){
    trace.info() << " pt: "<< vectPoints.at(k)<< endl;
  }
  nbok += (vectPoints.size()==4) ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "<< std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing reading freemanchain ..." );
  std::string filenameFC = testPath + "samples/freemanChainSample.fc";
  std::vector< FreemanChain< int > > vectFC = PointListReader< Z2i::Point>:: getFreemanChainsFromFile<int> (filenameFC);
  for(unsigned int i=0; i< vectFC.size(); i++){
    FreemanChain<int> fc = vectFC.at(i);
    trace.info() << "Freeman chain " << i << ": " << fc.x0 << " " << fc.y0 << " " << fc.chain << endl;
  }
  nbok += (vectFC.size()==5) ? 1 : 0;
  nb++;
  trace.endBlock();

  trace.beginBlock ( "Testing reading 3D polygons (one each line)..." );
  std::string filenamePoly = testPath + "samples/polygons.dat";
  std::vector< std::vector< Z3i::Point > > vectPolygons = PointListReader< Z3i::Point >::getPolygonsFromFile(filenamePoly);
  for(unsigned int i=0; i< vectPolygons.size(); i++){
    std::vector< Z3i::Point > aPolygon  = vectPolygons.at(i);
    trace.info() << "Polygon " << i << ": " ;
    for(unsigned int j =0; j <aPolygon.size(); j++){
      trace.info()<< "Point :" << aPolygon.at(j);
    }
    trace.info()<< std::endl;
  }
  trace.beginBlock ( "Testing reading special 1 dimension elements (polygon of 1 dimension point)..." );

  typedef  PointVector<1, int> Point1D;
  std::vector< std::vector< Point1D > > vectPt1D = PointListReader< Point1D >::getPolygonsFromFile(filenamePoly);
  for(unsigned int i=0; i< vectPt1D.size(); i++){
    std::vector< Point1D > aPolygon  = vectPt1D.at(i);
    trace.info() << "Polygon " << i << ": " ;
    for(unsigned int j =0; j <aPolygon.size(); j++){
      trace.info()<< "Point :" << aPolygon.at(j);
    }
    trace.info()<< std::endl;
  }
  Point1D p1, p2, p3, p4, p5, p6; p1[0]=1; p2[0]=2; p3[0]=3; p4[0]=4; p5[0]=5; p6[0]=6;
  Point1D p7, p8, p9, p10, p11, p12; p7[0]=0; p8[0]=0; p9[0]=10; p10[0]=0; p11[0]=5; p12[0]=5;

    nbok += (vectPt1D.at(0).size()==6 && vectPt1D.at(1).size()==6 && vectPt1D.at(0).at(0) == p1 &&
          vectPt1D.at(0).at(1)==p2 && vectPt1D.at(0).at(2) == p3 &&  vectPt1D.at(0).at(3) == p4
       && vectPt1D.at(0).at(4)==p5 && vectPt1D.at(0).at(5)==p6
       && vectPt1D.at(1).at(0)==p7
       && vectPt1D.at(1).at(1)==p8 && vectPt1D.at(1).at(2)==p9 &&  vectPt1D.at(1).at(3)==p10
       && vectPt1D.at(1).at(4)==p11 && vectPt1D.at(1).at(5)==p12);


  nb++;
  trace.endBlock();

 trace.beginBlock ( "Testing reading 2D polygons (one each line)..." );

  std::vector< std::vector< Z2i::Point > > vectPolygons2D = PointListReader< Z2i::Point >::getPolygonsFromFile(filenamePoly);
  for(unsigned int i=0; i< vectPolygons2D.size(); i++){
    std::vector< Z2i::Point > aPolygon2D  = vectPolygons2D.at(i);
    trace.info() << "Polygon " << i << ": " ;
    for(unsigned int j =0; j <aPolygon2D.size(); j++){
      trace.info()<< "Point :" << aPolygon2D.at(j);
    }
    trace.info()<< std::endl;
  }
  nbok += (vectPolygons2D.at(0).size()==3 && vectPolygons2D.at(1).size()==3 && vectPolygons2D.at(0).at(0)==Z2i::Point(1,2)
           && vectPolygons2D.at(0).at(1)==Z2i::Point(3,4) && vectPolygons2D.at(0).at(2)==Z2i::Point(5,6)
           && vectPolygons2D.at(1).at(0)==Z2i::Point(0,0) && vectPolygons2D.at(1).at(1)==Z2i::Point(10, 0)
           && vectPolygons2D.at(1).at(2)==Z2i::Point(5,5)) ? 1 : 0;

  nb++;
  trace.endBlock();

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class PointListReader" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;


  bool res = testPointListReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
