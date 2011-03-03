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
 * @file testDomainRangeBench.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/02
 *
 * Functions for testing class DomainRangeBench.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DomainRangeBench.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDomainRangeBench()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  

  Z3i::Point a(0,0,0);
  Z3i::Point b(150000,1500000,1500000);
  Z3i::Point start(250,0,0);
  Z3i::Domain domain(a,b);
  
  long int cpt=0;

  typedef Z3i::Domain::ConstIterator CIt;

  trace.beginBlock ( "Testing 1D Range Iterator z ..." );
  for(CIt it = domain.subRange(2,start).begin(),
	itend = domain.subRange(2,start).end();
      it != itend;
      ++it)
    cpt += (*it)[2];
  trace.endBlock(); 
 

  long int cpt2=0;
  trace.beginBlock ( "Testing 1D naive Iterator ..." );
  for( Z3i::Point upper = domain.upperBound();
       start[2] <= upper[2];
       start[2]++)
    cpt2 += start[2];
  trace.endBlock();
  
  

  nbok += (cpt==cpt2) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DomainRangeBench" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDomainRangeBench(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
