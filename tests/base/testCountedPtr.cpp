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
 * @file testCountedPtr.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/05/31
 *
 * Functions for testing class CountedPtr.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CountedPtr.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testCountedPtr()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing CountedPtr ..." );

  int *value=new int(5);
  CountedPtr<int> p( value );
  nbok += p.unique() ? 1 : 0;
  nb++;
  trace.info() << p << " value=" << *p<< std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "unique" << std::endl;

  *p = 6;
  trace.info() << p << " value=" << *p<< std::endl;
  nbok += p.unique() ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "unique" << std::endl;

  trace.endBlock();

  return nbok == nb;
}


bool testCountedPtrCopy()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing CountedPtr copy..." );

  int *value= new int(5);
  CountedPtr<int> p( value );
  nbok += p.unique() ? 1 : 0;
  nb++;
  trace.info() << p <<std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "unique" << std::endl;

  CountedPtr<int> q ( p );

  nbok += p.unique() ? 0: 1;
  nb++;
  trace.info() << p <<std::endl;
  trace.info() << q<<std::endl;
  trace.info() << "(" << nbok << "/" << nb << ") "
         << "not unique anymore" << std::endl;


  trace.endBlock();

  return nbok == nb;
}


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CountedPtr" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testCountedPtr() && testCountedPtrCopy(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
