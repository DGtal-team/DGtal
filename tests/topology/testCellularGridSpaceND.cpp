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
 * @file testCellularGridSpaceND.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/02/08
 *
 * Functions for testing class CellularGridSpaceND.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/KhalimskySpaceND.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class CellularGridSpaceND.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
template <typename KSpace>
bool testCellularGridSpaceND()
{
  typedef typename KSpace::Cell Cell;
  typedef typename KSpace::SCell SCell;
  typedef typename KSpace::Point Point;
  typedef typename KSpace::DirIterator DirIterator;
  typedef typename KSpace::Cells Cells;
  typedef typename KSpace::SCells SCells;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  
  trace.beginBlock ( "Testing block KSpace instantiation and scan ..." );
  KSpace K;
  int xlow[ 4 ] = { -3, -2, -2, -1 };
  int xhigh[ 4 ] = { 5, 3, 2, 3 };
  Point low( xlow );
  Point high( xhigh ); 
  bool space_ok = K.init( low, high, true );
  nbok += space_ok ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "K.init( low, high )" << std::endl;
  trace.info() << "K.dim()=" << K.dimension << endl;
  int spel[ 4 ] = { 1, 1, 1, 1 }; // pixel
  Point kp( spel );
  Cell center = K.uCell( kp );
  Cell c1 = K.uCell( kp );
  Cell clow = K.uCell( low, kp );
  Cell chigh = K.uCell( high, kp );
  trace.info() << c1 << clow << chigh 
	       << " topo(c1)=" << K.uTopology( c1 ) << " dirs=";
  for ( DirIterator q = K.uDirs( clow ); q != 0; ++q )
    trace.info() << " " << *q;
  trace.info() << endl;
  Cell f = K.uFirst( c1 );
  Cell l = K.uLast( c1 );
  trace.info() << "Loop in " << clow << chigh << endl;
  c1 = f;
  unsigned int nbelems = 0;
  do {
    ++nbelems;
    // trace.info() << c1;
  } while ( K.uNext( c1, f, l ) );
  trace.info() << " -> " << nbelems << " elements." << endl;
  unsigned int exp_nbelems = 1;
  for ( Dimension i = 0; i < K.dimension; ++i )
    exp_nbelems *= K.size( i );
  nbok += nbelems == exp_nbelems ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << nbelems << " scanned elements == "
	       << exp_nbelems << " space size."
	       << std::endl;
  trace.endBlock();
  trace.beginBlock ( "Testing neighborhoods in KSpace..." );
  Cells N = K.uNeighborhood( center );
  nbok += N.size() == ( K.dimension*2 + 1 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << N.size() << "(neighborhood size) == "
	       << ( K.dimension*2 + 1 ) << "(2*dim()+1)" << endl;
  Cells Np = K.uProperNeighborhood( center );
  nbok += Np.size() == ( K.dimension*2 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << Np.size() << "(proper neighborhood size) == "
	       << ( K.dimension*2 ) << "(2*dim())" << endl;
  trace.endBlock();

  trace.beginBlock ( "Testing faces in KSpace..." );
  Cells Nf = K.uFaces( center );
  nbok += Nf.size() == round( pow( 3, K.dimension ) - 1 ) ? 1 : 0; 
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << Nf.size() << "(faces size) == "
	       << round( pow( 3, K.dimension ) - 1 ) << "(3^dim()-1)" << endl;
  trace.endBlock();
  
  trace.beginBlock ( "Testing block Incidence in KSpace..." );
  SCell sspel = K.sCell( kp, K.POS );
  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
	if ( *q1 != *q2 )
	  {
	    SCell s0 = K.sIncident( sspel, *q1, true );
	    SCell s1 = K.sIncident( sspel, *q2, true );
	    SCell l10 = K.sIncident( s0, *q2, true );
	    SCell l01 = K.sIncident( s1, *q1, true );
	    trace.info() << "D+_" << *q2 << "(D+_" << *q1 << "(V))=" << l10 
			 << " D+_" << *q1 << "(D+_" << *q2 << "(V))=" << l01
			 << endl;
	    nbok += l10 == K.sOpp( l01 ) ? 1 : 0; 
	    nb++;
	  }
      }
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "anti-commutativity of incidence operators." << std::endl;
  trace.endBlock();

  trace.beginBlock ( "Testing direct Incidence in KSpace..." );
  for ( DirIterator q1 = K.sDirs( sspel ); q1 != 0; ++q1 )
    for ( DirIterator q2 = K.sDirs( sspel ); q2 != 0; ++q2 )
      {
	if ( *q1 != *q2 )
	  {
	    SCell s0 = K.sDirectIncident( sspel, *q1 );
	    SCell l10 = K.sDirectIncident( s0, *q2 );
	    SCell s1 = K.sDirectIncident( sspel, *q2 );
	    SCell l01 = K.sDirectIncident( s1, *q1 );
	    trace.info() << "Dd_" << *q2 << "(Dd_" << *q1 << "(V))=" << l10 
			 << " Dd_" << *q1 << "(Dd_" << *q2 << "(V))=" << l01
			 << endl;
	    nbok += l10 != l01 ? 1 : 0; 
	    nbok += K.sSign( s0 ) == K.POS ? 1 : 0;
	    nbok += K.sSign( s1 ) == K.POS ? 1 : 0;
	    nbok += K.sSign( l10 ) == K.POS ? 1 : 0;
	    nbok += K.sSign( l01 ) == K.POS ? 1 : 0;
	    nbok += s0 == K.sIncident( sspel, *q1, K.sDirect( sspel, *q1 ) )
	      ? 1 : 0;
	    nbok += s1 == K.sIncident( sspel, *q2, K.sDirect( sspel, *q2 ) )
	      ? 1 : 0;
	    nbok += l10 == K.sIncident( s0, *q2, K.sDirect( s0, *q2 ) )
	      ? 1 : 0;
	    nbok += l01 == K.sIncident( s1, *q1, K.sDirect( s1, *q1 ) )
	      ? 1 : 0;
	    nb += 9;
	  }
      }
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "correctness of direct and indirect orientations." << std::endl;
  
  trace.endBlock();
  
  
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class CellularGridSpaceND" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  typedef KhalimskySpaceND<2> K2;
  typedef KhalimskySpaceND<3> K3;
  typedef KhalimskySpaceND<4> K4;
  bool res = testCellularGridSpaceND<K2>()
   && testCellularGridSpaceND<K3>()
   && testCellularGridSpaceND<K4>();
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
