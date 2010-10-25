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
//LICENSE-END
/**
 * @file testGenericWriterReader.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/25
 *
 * Functions for testing class VolReader.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/io/writers/GenericWriter.h"
#include "DGtal/io/readers/GenericReader.h"

#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Generic.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testGenericWriterReader()
{
  unsigned int nbok = 0;
  unsigned int nb = 1;
  
  trace.beginBlock ( "Testing Generic ..." );

  typedef SpaceND<2> TSpace;
  typedef TSpace::Point Point;
  typedef TSpace::Vector Vector;
  typedef HyperRectDomain<TSpace> Domain;
 
  Point a ( 1, 1);
  Point b ( 16, 16);
 
  typedef ImageSelector<Domain, unsigned char>::Type Image;
  Image image(a,b);
  for(unsigned int i=0 ; i < 256; i++)
    image[i] = i;
  
  GenericWriter<Point>::exportTXT("export-generic.txt",a);
  GenericWriter<Point>::exportBIN("export-generic.bin",a);
  GenericWriter<Point>::exportXML("export-generic.xml",a);

	
  /* Point c = GenericReader<Point>::importTXT("export-generic.txt");
  Point d = GenericReader<Point>::importBIN("export-generic.bin");
  Point e = GenericReader<Point>::importXML("export-generic.xml");
 
  //We check the result
  if ((a == c) && (a == d))// && (a == e))
  */
  nbok++;

  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class Generic" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testGenericWriterReader(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
