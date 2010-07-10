/**
 * @file testModuloComputer.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/07
 *
 * Functions for testing class ModuloComputer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/ModuloComputer.h"
#include "DGtal/kernel/IntegerTraits.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ModuloComputer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testModuloComputer()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );

  //Construct an arithmetic modulo 15

  
  ModuloComputer< int > modular(15);
  typedef IntegerTraits< int >::UnsignedVersion myUnsignedInteger;
  myUnsignedInteger a;
  
  a = modular.cast( 2 );   //a contains the value 2
  nbok += (a == 2) ? 1 : 0;
  nb++;

  trace.info() << "a= "<<a<<std::endl;
  a = modular.cast( -1 );  //a contains the value 14
  nbok += (a== 14) ? 1 : 0;
  nb++;

  trace.info() << "a= "<<a<<std::endl;
  modular.increment( a ); //a contains the value 0
  nbok += (a== 0) ? 1 : 0;
  nb++;  

  trace.info() << "a= "<<a<<std::endl;
  nbok += true ? 1 : 0; 
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
  trace.beginBlock ( "Testing class ModuloComputer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testModuloComputer(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
