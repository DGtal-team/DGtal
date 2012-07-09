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
 * @file testOFFWriter.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2012/07/08
 *
 * Functions for testing class OFFWriter.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/Display3D.h"
#include "DGtal/io/writers/OFFWriter.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class OFFWriter.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testOFFWriter()
{
  unsigned int nbok = 1;
  unsigned int nb = 0;
  // Constructing the object to export in OFF format
  vector<Display3D::pointD3D > vectVertex;
  Display3D::pointD3D p1,p2,p3,p4;
  p1.x=0.0; p1.y=0.0; p1.z=0.0;
  p2.x=1.0; p2.y=0.0; p2.z=0.0;
  p3.x=1.0; p3.y=1.0; p3.z=0.0;
  p4.x=0.0; p4.y=1.0; p4.z=0.0;
  
  vectVertex.push_back(p1);
  vectVertex.push_back(p2);
  vectVertex.push_back(p3);
  vectVertex.push_back(p4);
  
  
  vector<unsigned int> vectFaces;
  vectFaces.push_back(4);
  vectFaces.push_back(0);
  vectFaces.push_back(1);
  vectFaces.push_back(2);
  vectFaces.push_back(3);

  
  // Adding color to faces
  vector<DGtal::Color> vectColor;
  DGtal::Color col (25,0,0, 200);
  vectColor.push_back(col);

  bool isOK = OFFWriter<Display3D::pointD3D>::export2OFF("test.off",vectVertex, vectFaces,1, vectColor);
  
  
  
  trace.beginBlock ( "Testing block ..." );
  nbok += isOK ? 1 : 0; 
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
  trace.beginBlock ( "Testing class OFFWriter" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testOFFWriter(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
