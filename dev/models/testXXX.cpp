/**
 * @file testXXX.cpp
 * @ingroup Tests
 * @author AUTHOR (\c EMAIL )
 * INSTITUTION
 *
 * @date 2000/??/??
 *
 * Functions for testing class XXX.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "YYY/ZZZ/XXX.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class XXX.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 * @ingroup Tests
 */
bool testXXX()
{
}

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class XXX" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testXXX(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();

  return res ? 0 : 1;
}
