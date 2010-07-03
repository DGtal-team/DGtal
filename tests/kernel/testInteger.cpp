/**
 * @file testInteger.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/02
 *
 * Functions for testing class Integer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CInteger.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Integer.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 * @ingroup Tests
 */
bool testInteger()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  BOOST_CONCEPT_ASSERT(( CInteger<int> ));
  BOOST_CONCEPT_ASSERT(( CInteger<unsigned int> ));
  BOOST_CONCEPT_ASSERT(( CInteger<long long int> ));
  // These tests fail : bool is not a model of CInteger.
  // BOOST_CONCEPT_ASSERT(( CInteger<std::string> ));
  // BOOST_CONCEPT_ASSERT(( CInteger<bool> ));
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
  trace.beginBlock ( "Testing class Integer" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testInteger(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
