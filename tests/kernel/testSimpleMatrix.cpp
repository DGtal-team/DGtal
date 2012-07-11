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
 * @file testSimpleMatrix.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/07/10
 *
 * Functions for testing class SimpleMatrix.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SimpleMatrix.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SimpleMatrix.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleMatrix()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing create ..." );
  
  typedef SimpleMatrix<double,3,4> M34d;
  
  M34d m34d;
  trace.info() << m34d<<std::endl;
  
  m34d.setComponent(1,2, 0.5);
  trace.info() << m34d<<std::endl;
  
  nbok += (m34d(1,2) == 0.5) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

bool testArithm()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  
  typedef SimpleMatrix<double,3,4> M34d;
  
  M34d m34d;
  M34d m34dbis, resadd, ressub;
  for(DGtal::Dimension i = 0; i< 3; ++i)
    for(DGtal::Dimension j = 0; j< 4; ++j)
      {
        m34d.setComponent(i,j,i*j);
        m34dbis.setComponent(i,j,i+j);
        resadd.setComponent(i,j,i*j+i+j);
        ressub.setComponent(i,j,i*j-(double)i-(double)j);
      }
  
  trace.info() << m34d <<std::endl;
  trace.info() << m34dbis<<std::endl;
 
  trace.beginBlock ( "Testing add ..." ); 
  nbok += ((m34d + m34dbis) == resadd) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok" << std::endl;
  nbok += ((m34dbis + m34d) == resadd) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok commutative" << std::endl;
  
  M34d other;
  other += m34d;
  nbok += (other == m34d) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok +=" << std::endl;
  
  trace.endBlock();
  
  trace.beginBlock ( "Testing substraction ..." ); 
  nbok += ((m34d - m34dbis) == ressub) ? 1 : 0; 
  nb++;
  trace.info()<<ressub<<std::endl;
  trace.info()<<m34d - m34dbis<<std::endl;
  
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok simple" << std::endl;
  trace.endBlock();
  
  trace.beginBlock ( "Testing product ..." ); 
  nbok += ((m34d - m34dbis) == ressub) ? 1 : 0; 
  nb++;
  trace.info()<<ressub<<std::endl;
  trace.info()<<m34d - m34dbis<<std::endl;
  
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "ok simple" << std::endl;
  trace.endBlock();
  
  return nbok == nb;

}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SimpleMatrix" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSimpleMatrix() && testArithm(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
