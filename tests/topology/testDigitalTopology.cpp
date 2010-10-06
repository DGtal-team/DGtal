/**
 * @file testDigitalTopology.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/07/07
 *
 * Functions for testing class DigitalTopology.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/topology/MetricAdjacency.h"
#include "DGtal/topology/DigitalTopology.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalTopology.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDigitalTopologyZ2()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  typedef SpaceND< 2 > Zi2;
  typedef MetricAdjacency< Zi2, 1 > Adj4;
  typedef MetricAdjacency< Zi2, 2 > Adj8;
  Adj4 adj4;
  Adj8 adj8;

  trace.beginBlock ( "Testing Digital topology (4,8) in Z2." );
  typedef DigitalTopology< Adj4, Adj8 > DT48;
  DT48 dt( adj4, adj8, JORDAN_DT );
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << dt << std::endl;
  typedef DT48::ReverseTopology DT84;
  DT84 opp_dt = dt.reverseTopology();
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << opp_dt << std::endl;
  trace.endBlock();

  // should not compile
  // typedef DigitalTopology< Adj4, bool > DTimpossible;
  // DTimpossible dti( adj4, true );

  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DigitalTopology" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDigitalTopologyZ2(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
