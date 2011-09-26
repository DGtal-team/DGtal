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
 * @file testDigitalSurface.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2011/09/01
 *
 * Functions for testing class DigitalSurface.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/DigitalSetBoundary.h"
#include "DGtal/shapes/Shapes.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class DigitalSurface.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testDigitalSurface()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block ... DigitalSetBoundary" );
  using namespace Z2i;
  typedef DigitalSetBoundary<KSpace,DigitalSet> Boundary;
  typedef Boundary::SurfelConstIterator ConstIterator;
  typedef Boundary::Tracker Tracker;
  typedef Boundary::Surfel Surfel;
  Point p1( -10, -10 );
  Point p2( 10, 10 );
  Domain domain( p1, p2 );
  DigitalSet dig_set( domain );
  Shapes<Domain>::addNorm2Ball( dig_set, Point( 0, 0 ), 5 );
  Shapes<Domain>::removeNorm2Ball( dig_set, Point( 0, 0 ), 1 );
  KSpace K;
  nbok += K.init( domain.lowerBound(), domain.upperBound(), true ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "K.init() is ok" << std::endl;
  Boundary boundary( K, dig_set );
  unsigned int nbsurfels = 0;
  for ( ConstIterator it = boundary.begin(), it_end = boundary.end();
        it != it_end; ++it )
    {
      ++nbsurfels;
    }
  trace.info() << nbsurfels << " surfels found." << std::endl;
  nb++, nbok += nbsurfels == ( 12 + 44 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "nbsurfels == (12 + 44 )" << std::endl;
  for ( ConstIterator it = boundary.begin(), it_end = boundary.end();
        it != it_end; ++it )
    {
      Tracker* ptrTracker = boundary.newTracker( *it );
      Surfel s = ptrTracker->current();
      Dimension trackDir = * K.sDirs( s );
      Surfel s1, s2;
      unsigned int m1 = ptrTracker->adjacent( s1, trackDir, true ); 
      unsigned int m2 = ptrTracker->adjacent( s2, trackDir, false ); 
      trace.info() << "s = " << s << std::endl;
      trace.info() << "s1 = " << s1 << " m1 = " << m1 << std::endl;
      trace.info() << "s2 = " << s2 << " m2 = " << m2 << std::endl;
      nb++, nbok += boundary.isInside( s1 ) ? 1 : 0;
      trace.info() << "(" << nbok << "/" << nb << ") "
                   << "boundary.isInside( s1 )" << std::endl;
      nb++, nbok += boundary.isInside( s2 ) ? 1 : 0;
      trace.info() << "(" << nbok << "/" << nb << ") "
                   << "boundary.isInside( s2 )" << std::endl;
      delete ptrTracker;
    }
  trace.endBlock();

  trace.beginBlock ( "Testing block ... DigitalSurface" );
  typedef DigitalSurface<Boundary> MyDS;
  MyDS digsurf( boundary ); // duplicated.
  nb++, nbok += digsurf.size() == nbsurfels ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "digsurf.size() == nbsurfels" << std::endl;
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class DigitalSurface" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testDigitalSurface(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
