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
 * @file testArithmeticalDSS.cpp
 * @ingroup Tests
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/06/28
 *
 * Functions for testing class ArithmeticalDSS.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/curves/ArithmeticalDSS.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ArithmeticalDSS.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename DSS>
bool mainTest()
{
  typedef typename DSS::Point Point; 

  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );

  //operateur constructeur, copie, affectation
  trace.info() << "constructor, copy, assignement, equality" << std::endl; 

  DSS dss(0, 1, 0, 1, 
	  Point(0,0), Point(1,0), 
	  Point(0,0), Point(1,0),
	  Point(0,0), Point(1,0) ); 
  DSS dss2 = dss; 
  DSS dss3(1, 1, 0, 2, 
	  Point(0,0), Point(1,1), 
	  Point(0,0), Point(1,1),
	  Point(0,0), Point(1,1) );
  DSS dss4 = dss3; 
  dss3 = dss2 = dss; 

  //egalite, difference
  DSS dss5(0, -1, 0, 1, 
	  Point(1,0), Point(0,0), 
	  Point(1,0), Point(0,0),
	  Point(1,0), Point(0,0) ); 
  
  if ( (dss == dss2)
       &&(dss == dss3)
       &&(dss != dss4)
       &&(dss == dss5) )
    nbok++; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  //sortie standard
  trace.info() << dss << std::endl; 

  //validite
  trace.info() << "valid dss" << std::endl; 
  if ( dss.isValid() && dss3.isValid() && dss5.isValid() ) 
    nbok++; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  DSS dss6(0, 1, 0, 1, 
	  Point(1,0), Point(0,0), 
	  Point(1,0), Point(0,0),
	  Point(1,0), Point(0,0) ); 

  trace.info() << "not valid dss" << std::endl; 
  if (!dss6.isValid()) 
    nbok++; 
  nb++;

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  //accessors
  trace.info() << "a,b,mu,omega accessors" << std::endl; 

  if ( (dss.a() == 0)&&(dss.b() == 1)&&(dss.mu() == 0)&&(dss.omega() == 1) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
	       << std::endl;

  trace.info() << "points accessors" << std::endl; 
  if ( (dss.front() == Point(1,0))&&(dss.back() == Point(0,0)) )
    nbok++; 
  nb++; 
  if ( (dss.Ul() == Point(1,0))&&(dss.Uf() == Point(0,0)) )
    nbok++; 
  nb++; 
  if ( (dss.Ll() == Point(1,0))&&(dss.Lf() == Point(0,0)) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  DSS dss7(5, 8, 0, 13, 
	  Point(0,0), Point(8,5), 
	  Point(0,0), Point(8,5),
	  Point(4,1), Point(4,1) ); 
  
  trace.info() << "remainder, position, tests" << std::endl; 
  if ( (dss7.isValid()) 
       && (dss7.r( Point(8,5) ) == 0)
       &&(dss7.r( Point(16,10) ) == 0)
       &&(dss7.r( Point(3,2) ) == -1)
       &&(dss7.r( Point(5,3) ) == 1) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
 
  if ( (dss7.orthogonalPosition( Point(0,0) ) == 0)
       &&(dss7.orthogonalPosition( Point(8,5) ) == 89)
       &&(dss7.orthogonalPosition( Point(1,0) ) == 8)
       &&(dss7.orthogonalPosition( Point(-1,0) ) == -8) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  if ( (dss7.isInDSL( Point(0,0) ))
       &&(dss7.isInDSL( Point(16,10) ))
       &&(dss7.isInDSL( Point(5,3) ))
       &&(!dss7.isInDSL( Point(3,2) )) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;

  if ( (dss7( Point(0,0) ))
       &&(!dss7( Point(16,10) ))
       &&(dss7( Point(5,3) ))
       &&(!dss7( Point(3,2) ))
       &&(!dss7( Point(-1,0) )) )
    nbok++; 
  nb++; 

  trace.info() << "(" << nbok << "/" << nb << ") "
  	       << std::endl;
            
  trace.endBlock();
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class ArithmeticalDSS" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t> >()
    && mainTest<DGtal::ArithmeticalDSS<DGtal::int32_t, DGtal::BigInteger, 4> >()
    ;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
