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
 * @file testSignal.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/07/06
 *
 * Functions for testing class Signal.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/RealPointVector.h"
#include "DGtal/math/Signal.h"
#include "DGtal/geometry/2d/BinomialConvolver.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Signal.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSignal()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  //double values[ 8 ] = { 1.0, 5.0, 3.0, 4.3, 6.2, 7.1, 5.7, 8.4 };
  double values[ ] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  Signal<double> signal( values, 8, 2, false, 0.0 );
  trace.info() << "Unperiodic signal" << endl;
  trace.info() << signal << endl;
  for ( unsigned int n = 1; n < 10; ++n )
    {
      Signal<double> Gauss = Signal<double>::G2n( n );
      Signal<double> conv_signal = signal * Gauss;
      trace.info() << conv_signal << endl;
    }
  Signal<double> signal2( values, 8, 2, true, 0.0 );
  trace.info() << "Periodic signal" << endl;
  trace.info() << signal2 << endl;
  for ( unsigned int n = 1; n < 10; ++n )
    {
      Signal<double> Gauss = Signal<double>::G2n( n );
      Signal<double> conv_signal = signal2 * Gauss;
      trace.info() << conv_signal << endl;
    }
  nbok += true ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "true == true" << std::endl;
  trace.endBlock();
  
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testBinomialConvolver()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ..." );
  typedef RealPointVector<2> RealPoint;
  std::vector< RealPoint > points;
  points.push_back( RealPoint( { 0.0, 0.0 } ) ); 
  points.push_back( RealPoint( { 1.0, 0.0 } ) ); 
  points.push_back( RealPoint( { 2.0, 0.0 } ) ); 
  points.push_back( RealPoint( { 2.0, 1.0 } ) ); 
  points.push_back( RealPoint( { 2.0, 2.0 } ) ); 
  points.push_back( RealPoint( { 1.0, 2.0 } ) ); 
  points.push_back( RealPoint( { 0.0, 2.0 } ) ); 
  points.push_back( RealPoint( { 0.0, 1.0 } ) ); 
  typedef std::vector< RealPoint >::const_iterator ConstIteratorOnPoints;
  for ( unsigned int n = 1; n < 10; ++n )
    {
      trace.info() << "Binomial convolver n=" << n << std::endl;
      BinomialConvolver< ConstIteratorOnPoints, double > bcc( n );
      bcc.init( 1.0, points.begin(), points.end(), true );
      for ( unsigned int i = 0; i < 8; ++i )
      	std::cout << i
      		  << " " << bcc.x( i ).first
      		  << " " << bcc.x( i ).second
      		  << " " << bcc.tangent( i ).first
      		  << " " << bcc.tangent( i ).second
      		  << " " << bcc.curvature( i )
      		  << std::endl;
    }
  trace.info() << "Binomial convolver suggested n=" 
	       << BinomialConvolver< ConstIteratorOnPoints, double >::suggestedSize( 1.0, points.begin(), points.end() )
	       << std::endl;

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
  trace.beginBlock ( "Testing class Signal" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSignal() && testBinomialConvolver(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
