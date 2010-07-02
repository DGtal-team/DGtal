/**
 * @file FreemanChain.cpp
 *
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/07/01
 *
 * Implementation of methods defined in FreemanChain.h
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/math/Mathutils.h"
#include <string>
#include <sstream>
#include <vector>
#include <deque>


// Includes inline functions/methods if necessary.
#if !defined(INLINE)
#include "DGtal/geomeetry/2d/FreemanChain.ih"
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;

using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// class FreemanChain
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

/**
 * Destructor.
 */
DGtal::FreemanChain::~FreemanChain()
{
}







/**
 * Outputs the chain [c] to the stream [out].
 * @param out any output stream,
 * @param c a Freeman chain.
 */
void
DGtal::FreemanChain::write( std::ostream & out, const FreemanChain & c )
{
  out << c.x0 << " " << c.y0 << " " << c.chain << endl;
}


/**
 * Reads a chain from the stream [in] and updates [c].
 * @param in any input stream,
 * @param c (returns) the Freeman chain.
 */
void 
DGtal::FreemanChain::read( std::istream & in, FreemanChain & c )
{
  string str;
  while ( true )
    {
      getline( in, str );
      if ( ! in.good() ) return;
      if ( ( str.size() > 0 ) && ( str[ 0 ] != '#' ) )
	{
	  istringstream str_in( str );
	  str_in >> c.x0 >> c.y0 >> c.chain;
	  return;
	}
    }
}



// /**
//  * Creates a Freeman chaincode [chain] and a chain coding the
//  * quadrant of each step [qchain], given an iterator on a
//  * 4-connected path [it], a number of step [nb], the first step
//  * [freeman_code] and the first quadrant [quadrant].
//  *
//  * @param chain (returns) a string of '0', '1', '2', or '3'
//  * (Freeman chaincode)
//  *
//  * @param qchain (returns) a string of '0', '1', '2', or '3'
//  * giving the quadrants.
//  *
//  * @param it any iterator 
//  *
//  * @param nb the number of 'next' performed on a copy of [it].
//  *
//  * @param freeman the first code or step.
//  *
//  * @param quadrant the first quadrant (equal to freeman_code or one below).
//  *
//  * @param start_index the starting index in [chain] and [qchain], default is 0.
//  */
// void
// DGtal::FreemanChain::create( std::string & chain,
// 			       std::string & qchain,
// 			       const C4CIterator & it, 
// 			       unsigned int nb, unsigned int freeman, unsigned int quadrant,
// 			       unsigned int start_index )
// {
//   // Computes Freeman chain code and quadrants.
//   Mathutils::ModuloComputer mf( 4 );
//   Mathutils::ModuloComputer msu( nb );
//   chain.resize( nb );
//   qchain.resize( nb );
//   Proxy<C4CIterator> it2( it.clone() );

//   for ( unsigned int i = 0; i != nb; ++i )
//     {
//       chain[ start_index ] = '0' + freeman;
//       qchain[ start_index ] = '0' + quadrant;
//       unsigned int code = it2->next();
//       switch ( code )
// 	{
// 	case 0: cerr << "[DGtal::FreemanChain::create] "
// 		     << " Contour is too short." << endl; 
// 	  break;
// 	case 1: mf.increment( freeman );
// 	  break;
// 	case 2: break;
// 	case 3: mf.decrement( freeman );
// 	  break;
// 	}
//       if ( ( quadrant != freeman ) 
// 	   && ( mf.next( quadrant ) != freeman ) )
// 	{ // Change of quadrant.
// 	  if ( mf.previous( quadrant ) == freeman )
// 	    mf.decrement( quadrant );
// 	  else if ( mf.next( quadrant ) == mf.previous( freeman ) )
// 	    mf.increment( quadrant );
// 	}
//       msu.increment( start_index );
//     }
// }




/**
 * @param aZero (returns) the '0' or 'x' letter for quadrant [quadrant].
 * @param aOne (returns) the '1' or 'y' letter for quadrant [quadrant].
 * @param aQuadrant the quadrant as any of '0', '1', '2', or '3'.
 */
void
DGtal::FreemanChain::alphabet( char & aZero, char & aOne, 
			       char aQuadrant )
{
  switch ( aQuadrant )
    {
    case '0': aZero = '0'; aOne = '1'; break;
    case '1': aZero = '1'; aOne = '2'; break;
    case '2': aZero = '2'; aOne = '3'; break;
    case '3': aZero = '3'; aOne = '0'; break;
    }
}







/**
 * Given two consecutive moves on a Freeman chain code, this
 * method returns the type of movement: 0: return move, 1: turning
 * toward the interior, 2: going straight, 3: turning toward
 * exterior. Interior/exterior is specified by [ccw].
 *
 * @param aCode1 the code of the first step as an integer in 0..3.
 * @param aCode2 the code of the second step as an integer in 0..3.
 * @param ccw 'true' if the contour is seen counterclockwise with
 * its inside to the left.
 */
unsigned int
DGtal::FreemanChain::movement( unsigned int aCode1, unsigned int aCode2, bool ccw )
{
  unsigned int cfg = ( ccw ? 0 : 16 ) + ( aCode1 << 2 ) + aCode2;
  static const unsigned int tbl[ 32 ] = { 
    2, 1, 0, 3, 3, 2, 1, 0,
    0, 3, 2, 1, 1, 0, 3, 2, 
    2, 3, 0, 1, 1, 2, 3, 0,
    0, 1, 2, 3, 3, 0, 1, 2 };
  return tbl[ cfg ];
}


/**
 * From the Freeman chain [aPl_chain] representing a pointel
 * 4-connected contour, constructs the Freeman chain [aPix_chain]
 * that represents its inner 4-connected border of pixels. The
 * Freeman chain [aPl_chain] has its inside to the left (ie. ccw).
 * 
 * Note that chain codes going back and forth are @b never considered
 * useless: it means that the chain is always supposed to have its
 * interior to the left (ccw) or right (cw) even at configurations
 * "02", "13", "20", "31".
 * 
 * @param aPix_chain (output) the code of the 4-connected inner border. 
 *
 * @param aPl2pix (output) the mapping associating pointels to
 * pixels as indices in their respective Freeman chain.
 *
 * @param aPix2pl (output) the inverse mapping associating pixels to
 * pointels as indices in their respective Freeman chain.
 *
 * @param aPl_chain the input code of the 4-connected pointel contour.
 */
void
DGtal::FreemanChain::pointel2pixel( FreemanChain & aPixChain,
				    vector<unsigned int> & aPl2pix,
				    vector<unsigned int> & aPix2pl,
				    const FreemanChain & aPlChain )
{
  innerContour( aPixChain, aPl2pix, aPix2pl, aPlChain, true );
}

/**
 * From the Freeman chain [outer_chain] representing a 4-connected
 * contour, constructs the Freeman chain [inner_chain] that
 * represents its inner 4-connected contour (which lies in its
 * interpixel space). The boolean [ccw] specifies if the inside is
 * to the left (ccw) or to the right (cw).
 *
 * Note that chain codes going back and forth are @b never considered
 * useless: it means that the chain is always supposed to have its
 * interior to the left (ccw) or right (cw) even at configurations
 * "02", "13", "20", "31".
 * 
 * @param aInner_chain (output) the code of the 4-connected inner
 * border, with starting coordinates that are floored to the closest
 * integer.
 *
 * @param aOuter2inner (output) the mapping associating outer to
 * inner elements as indices in their respective Freeman chain.
 *
 * @param aInner2outer (output) the mapping associating inner to
 * outer elements as indices in their respective Freeman chain.
 *
 * @param aOuter_chain the input code of the 4-connected contour.
 *
 * @param ccw 'true' if the contour is seen counterclockwise with its
 * inside to the left.
 */
void 
DGtal::FreemanChain::innerContour
( FreemanChain & aInnerChain,
  vector<unsigned int> & aOuter2inner,
  vector<unsigned int> & aInner2outer,
  const FreemanChain & aOuterChain,
  bool ccw )
{
  uint nb = aOuterChain.chain.size();
  uint j = 0;
  aOuter2inner.clear();
  aOuter2inner.reserve( nb );
  // aInnerChain.chain.reserve( nb + 4 );
  aInnerChain.chain = "";
  aInner2outer.clear();
  aInner2outer.reserve( nb + ( ccw ? 4 : -4 ) );
  int dx0, dy0;
  int dx1, dy1;
  FreemanChain::displacement( dx0, dy0, aOuterChain.code( 0 ) );
  int turn = ccw ? 1 : 3;
  FreemanChain::displacement( dx1, dy1, ( aOuterChain.code( 0 ) + turn ) % 4 );
  dx0 += dx1;
  dy0 += dy1;
  aInnerChain.x0 = dx0 > 0 ? aOuterChain.x0 : aOuterChain.x0 - 1;
  aInnerChain.y0 = dy0 > 0 ? aOuterChain.y0 : aOuterChain.y0 - 1;

  FreemanChain::const_iterator it_begin = aOuterChain.begin();
  FreemanChain::const_iterator it = it_begin;
  it.next();
  for ( uint i = 0; i < nb; ++i )
    {
      // Check if contour is open.
      // cerr << "i=" << i << " code=" << aOuterChain.code( i ) << endl;
      switch ( movement( aOuterChain.code( i ), 
			 aOuterChain.code( ( i + 1 ) % nb ),
			 ccw ) ) 
	{
	case 0:
	// contour going in then out.
	  aInnerChain.chain += aOuterChain.chain[ i ];
	  aInnerChain.chain += ( ( ( (uint) ( aOuterChain.chain[ i ] - '0' ) 
				     + ( ccw ? 3 : 1 ) ) )
				 % 4 ) + '0';
	  aInnerChain.chain += aOuterChain.chain[ ( i + 1 ) % nb ];
	  aOuter2inner.push_back( j );
	  aInner2outer.push_back( i );
	  aInner2outer.push_back( i + 1 );
	  aInner2outer.push_back( i + 1 );
	  j += 3;
	  break;

	case 1:
	  // contour turning toward its inside.
	  aOuter2inner.push_back( j );
	  break;

	case 2:
	  // contour going straight ahead
	  aInnerChain.chain += aOuterChain.chain[ i ];
	  aOuter2inner.push_back( j );
	  aInner2outer.push_back( i );
	  ++j;
	  break;

	case 3:
	  // contour turning toward its outside.
	  aInnerChain.chain += aOuterChain.chain[ i ];
	  aInnerChain.chain += aOuterChain.chain[ ( i + 1 ) % nb ];
	  aOuter2inner.push_back( j );
	  aInner2outer.push_back( i );
	  aInner2outer.push_back( i + 1 );
	  j += 2;
	  break;
	}	
      

      // Advances along contour and check if it is a closed contour.
      it.next();
      if ( ( i == nb - 1 )
	   && ( *it_begin != *it ) ) 
	// freeman chain is *not* a closed loop.
	{
	  aInnerChain.chain += aOuterChain.chain[ i ];
	  aOuter2inner.push_back( j );
	  aInner2outer.push_back( i );
	  ++i; ++j;
	  break;
	}
    }

 
}
    

/**
 * Reads the 4-connected contour [c] so that meaningless back and
 * forth steps are removed. These operations may create one or
 * several 4-connected contours (stored in [clean_cs]), whether
 * these removals cuts the contour in several loops. Because of
 * that, the mappings are more complex.

 * @param clean_cs (output) the array of cleaned 4-connected contours.
 *
 * @param c2clean (output) the mapping associating an element to
 * its clean element as a pair (n,i) where n is the index of the
 * cleaned contour and i the indice of the element in this Freeman
 * chain.
 *
 * @param clean2c (output) the array of mapping associating a
 * clean element to its non-clean element. clean2c[n][j] gives the
 * index of the non-clean element on c corresponding to the clean
 * element of index j in the n-th contour.
 *
 * @param c the input code of the 4-connected contour.
 *
 * @param ccw 'true' if the contour is seen counterclockwise with
 * its inside to the left.
 */
void
DGtal::FreemanChain::cleanContour( vector<FreemanChain> & aCleanCs,
				     vector< pair<unsigned int,unsigned int> > & aC2clean,
				     vector< vector<unsigned int> > & aClean2c,
				     const FreemanChain & c,
				     bool ccw )
{
  
}


/**
 * Removes outer spikes along a 4-connected contour, meaning steps
 * "02", "13", "20" or "31", which point outside the shape. The
 * inside is given by parameter [ccw]. Note that 4-connected
 * pointel contours should not have any outer spikes, while
 * 4-connected pixel contours should not have any inner spikes.
 *
 * @param clean_c (output) the cleaned 4-connected contour.
 *
 * @param c2clean (output) the mapping associating an element to
 * its clean element.
 *
 * @param clean2c (output) the inverse mapping associating a
 * clean element to its non-clean element. 
 *
 * @param c the input code of the 4-connected contour.
 *
 * @param ccw 'true' if the contour is seen counterclockwise with
 * its inside to the left.
 *
 * @return 'true' if the contour add an interior, 'false' otherwise.
 */
bool
DGtal::FreemanChain::cleanOuterSpikes( FreemanChain & aCleanC,
					 std::vector<unsigned int> & aC2clean,
					 std::vector<unsigned int> & aClean2c,
					 const FreemanChain & c,
					 bool ccw )
{
  unsigned int nb = c.chain.size();
  if ( nb == 0 ) 
    {
      cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
	   << " cleanOuterSpikes: Empty input chain"
	   << endl;
      return false;
    }
  Mathutils::ModuloComputer mc( nb );
  unsigned int i = 0;
  unsigned int j = 0;
  vector<unsigned int> c2cleanTMP;
  aCleanC.chain.reserve( nb );
  aCleanC.chain = "";
  aC2clean.clear();
  aClean2c.clear();
  aC2clean.reserve( nb );
  aClean2c.reserve( nb );
  c2cleanTMP.reserve( nb );
  FreemanChain::const_iterator it = c.begin();
  FreemanChain::const_iterator itn = c.begin(); itn.nextInLoop();
  // Find a consistent starting point.
  unsigned int n;
  unsigned int size_spike = 0;
  for ( n = 0; n < nb; ++n )
    {
      size_spike = 0;
      while ( movement( it.getCode(), itn.getCode(), ccw ) == 0 )
	{
	  it.previousInLoop(); 
	  itn.nextInLoop(); 
	  mc.increment( i );
	  size_spike += 2;
	  if ( size_spike >= nb )
	    {
	      cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
		   << " Spike is longer than contour !" 
		   << " size_spike=" << size_spike
		   << " nb=" << nb
		   << endl;
	      return false;
	    }
	}
      mc.increment( i );
      it = itn;
      itn.nextInLoop();
      if ( size_spike > 0 ) break;
    }
  unsigned int start_idx = it.getPosition();
  i = start_idx;
  // JOL : 2009/07/7, added starting coordinates
  PointI2D P = *it;
  aCleanC.x0 = P.at(0);
  aCleanC.y0 = P.at(1);

  // cerr << "Starting point is " << i << endl;
  ASSERT( ( n < nb ) || ( i == 0 ) );
  if ( ( n == nb ) )
    { // do nothing
      aCleanC.chain = c.chain;
      for ( unsigned int n = 0; n < nb; ++n )
	{
	  aC2clean.push_back( n );
	  aClean2c.push_back( n );
	}
      if ( size_spike != 0 )
	cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
	     << "No starting point found (only spikes !)" << endl;
      
      return size_spike == 0;
    }
  // Loops over all letters.
  FreemanChain::const_iterator it_begin = it;
  deque<unsigned int> clean_code;
  deque<unsigned int> clean_idx;
  vector<unsigned int> begin_outer_spike;
  vector<unsigned int> end_outer_spike;
  // i follows iterator it.
  do
    {
      clean_code.push_back( it.getCode() );
      clean_idx.push_back( i );
      itn = it;
      it.nextInLoop();
      mc.increment( i );
      // cerr << "- i=" << i << " (" << clean_code.back() 
      // 	   << it.getCode() << ") ";
      unsigned int size_spike = 0;
      unsigned int last_spike_idx = end_outer_spike.empty() ?
	start_idx :
	end_outer_spike.back();
      j = i;
      while ( ( ! clean_code.empty() )
	      && ( j != last_spike_idx ) 
	      && ( movement( clean_code.back(), it.getCode(), ccw ) == 0 )
	      && ( it != it_begin ) )
	{
	  clean_code.pop_back(); 
	  clean_idx.pop_back();
	  mc.increment( i );
	  mc.decrement( j );
	  it.nextInLoop();
	  itn.previousInLoop();
	  size_spike += 2;
	}
      // cerr << "i=" << i << " size_spike=" << size_spike
      // 	   << " last_spike_idx=" << last_spike_idx
      // 	   << endl;
      if ( size_spike != 0 )
	{
	  // There is a spike. Is it an outer one ?  
	  unsigned int previous_code = itn.getCode();
	  unsigned int previous_idx = itn.getPosition();
	  // JOL : do not
	  // consider any more "cleaned contour" since we cannot go
	  // further than the last spike.
	  // unsigned int previous_code = 
	  //   clean_code.empty() ? itn.getCode() : clean_code.back();
	  // unsigned int previous_idx = 
	  //   clean_code.empty() ? itn.getPosition() : clean_idx.back();
	  itn = it; 
	  itn.previousInLoop();
	  unsigned int move1 = movement( previous_code,
				 ( itn.getCode() + 2 ) % 4, ccw );
	  unsigned int move2 = movement( itn.getCode(), it.getCode() , ccw );
	  bool return_spike = ( move1 == 0 ) || ( move2 == 0 );
	  bool outer_spike = ( move1 == 3 ) || ( move2 == 3 );
// 	  if ( return_spike )
// 	    cerr << "[DGtal::FreemanChain::cleanOuterSpikes] return spike."
// 		 << endl;
// 	  if ( ! ( ( outer_spike && ( move1 != 1 ) && ( move2 != 1 ) )
// 		   || ( ! outer_spike && ( move1 != 3 ) && ( move2 != 3 ) ) ) )
// 	    cerr << "[DGtal::FreemanChain::cleanOuterSpikes] "
// 		 << "Weird spike. Invalid contour (expected 3 3) ?" 
// 		 << " move1=" << move1
// 		 << " move2=" << move2
// 		 << " ccw=" << ccw
// 		 << " start_idx=" << start_idx
// 		 << " size_spike=" << size_spike
// 		 << " it=" << it.getPosition()
// 		 << " itp=" << previous_idx
// 		 << endl
// 		 << c.chain << endl;
	  // Process waiting steps.
	  if ( outer_spike || return_spike ) 
	    {
	      begin_outer_spike.push_back( mc.next( previous_idx ) );
	      end_outer_spike.push_back( i );
	      // cout << " outer spike [" << begin_outer_spike.back()
	      // 	   << "," << end_outer_spike.back() << "[  " << endl;
	    }
	}
    }
  while ( it != it_begin );

  // Once outer spikes are known, we can create the new contour.
  aC2clean.resize( nb );
  i = start_idx % nb;
  j = 0;
  unsigned int nb_spikes = begin_outer_spike.size();
  unsigned int k = 0;
  n = 0;
  while ( n < nb )
    {
      if ( ( k == nb_spikes ) || ( i != begin_outer_spike[ k ] ) )
	{
	  aCleanC.chain.push_back( c.chain[ i ] );
	  aC2clean[ i ] = j;
	  aClean2c.push_back( i );
	  mc.increment( i );
	  ++j;
	  ++n;
	}
      else 
	{
	  while ( i != end_outer_spike[ k ] )
	    {
	      aC2clean[ i ] = j;
	      mc.increment( i );
	      ++n;
	    }
	  ++k;
	}
    }
  for ( unsigned int i = 0; i < nb; ++i )
    if ( aC2clean[ i ] >= aCleanC.chain.size() )
      if ( aC2clean[ i ] == aCleanC.chain.size() )
	aC2clean[ i ] = 0;
      else 
	{
	  cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
	       << "Bad correspondence for aC2clean[" << i<< "]"
	       << " = " << aC2clean[ i ] << " >= " << aCleanC.chain.size()
	       << endl;
	  aC2clean[ i ] = aC2clean[ i ] % aCleanC.chain.size();
	}

  for ( unsigned int j = 0; j < aCleanC.chain.size(); ++j )
    if ( aClean2c[ j ] >= nb )
      {
	cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
	     << "Bad correspondence for aClean2c[" << j << "]"
	     << " = " << aClean2c[ j ] << " >= " << nb
	     << endl;
	aClean2c[ j ] = aClean2c[ j ] % nb;
      }

  // for( int i= 0; i < c2cleanTMP.size(); i++ ) {
  //   aC2clean.push_back
  //     ( c2cleanTMP.at( ( i + (c2cleanTMP.size()-start_idx) ) 
  // 		       % c2cleanTMP.size() ) );
  // }

  // i = start_idx;
  // j = 0;
  // unsigned int nb_spikes = begin_outer_spike.size();
  // unsigned int k = 0;
  // n = 0;
  // while ( n < nb )
  //   {
  //     if ( ( k == nb_spikes ) || ( i != begin_outer_spike[ k ] ) )
  // 	{
  // 	  aCleanC.chain.push_back( c.chain[ i ] );
  // 	  c2cleanTMP.push_back( j );
  // 	  aClean2c.push_back( i );
  // 	  mc.increment( i );
  // 	  ++j;
  // 	  ++n;
  // 	}
  //     else 
  // 	{
  // 	  while ( i != end_outer_spike[ k ] )
  // 	    {
  // 	      c2cleanTMP.push_back( j );
  // 	      mc.increment( i );
  // 	      ++n;
  // 	    }
  // 	  ++k;
  // 	}
  //   }

  // for( int i= 0; i < c2cleanTMP.size(); i++ ) {
  //   aC2clean.push_back
  //     ( c2cleanTMP.at( ( i + (c2cleanTMP.size()-start_idx) ) 
  // 		       % c2cleanTMP.size() ) );
  // }

  // while ( true )
  //   {
  //     cout << "- i=" << i;
  //     unsigned int size_spike = 0;
  //     while ( movement( it.getCode(), itn.getCode(), ccw ) == 0 )
  // 	{
  // 	  it.previousInLoop(); 
  // 	  itn.nextInLoop(); 
  // 	  mc.decrement( i );
  // 	  size_spike += 2;
  // 	}
  //     cout << " with spike of size " << size_spike << endl;
  //     if ( size_spike == 0 )
  // 	{
  // 	  fc_to_process.push_back( c.chain[ i ] );
  // 	  idx_to_process.push_back( i );
  // 	  it.nextInLoop();
  // 	  itn.nextInLoop();
  // 	  mc.increment( i );
  // 	}
  //     else
  // 	{
  // 	  // There is a spike. Is it an outer one ?
  // 	  unsigned int code11 = it.getCode();
  // 	  it.nextInLoop();
  // 	  unsigned int code12 = it.getCode();
  // 	  mc.increment( i );
  // 	  unsigned int move1 = movement( code11, code12, ccw );
  // 	  unsigned int code22 = itn.getCode();
  // 	  itn.previousInLoop();
  // 	  unsigned int code21 = itn.getCode();
  // 	  unsigned int move2 = movement( code21, code22, ccw );
  // 	  bool outer_spike = ( move1 == 3 ) || ( move2 == 3 );
  // 	  ASSERT_FreemanChain
  // 	    ( ( ( outer_spike && ( move1 != 1 ) && ( move2 != 1 ) )
  // 		|| ( ! outer_spike && ( move1 != 3 ) && ( move2 != 3 ) ) )
  // 	      && "[DGtal::FreemanChain::cleanOuterSpikes] Weird spike. Invalid contour ?" );
  // 	  // Process waiting steps.
  // 	  while ( ! fc_to_process.empty() )
  // 	    {
  // 	      aCleanC.chain += fc_to_process.front();
  // 	      fc_to_process.pop_front();
  // 	      aC2clean.push_back( j ) ; 
  // 	      aClean2c.push_back( idx_to_process.front() );
  // 	      // aC2clean[ idx_to_process.front() ] = j;
  // 	      // aClean2c[ j ] = idx_to_process.front();
  // 	      idx_to_process.pop_front();
  // 	      ++j;
  // 	    }
  // 	  if ( outer_spike ) 
  // 	    {
  // 	      // Outer spike. Remove it.
  // 	      for ( unsigned int k = 0; k < size_spike; ++k )
  // 		{
  // 		  // aC2clean[ i ] = j;
  // 		  aC2clean.push_back( j ) ; 
  // 		  mc.increment( i );
  // 		  it.nextInLoop();
  // 		}
  // 	    }
  // 	  else
  // 	    {
  // 	      // Inner spike. Keep it.
  // 	      for ( unsigned int k = 0; k < size_spike; ++k )
  // 		{
  // 		  aC2clean.push_back( j ) ; 
  // 		  aClean2c.push_back( i );
  // 		  // aC2clean[ i ] = j;
  // 		  // aClean2c[ j ] = i;
  // 		  aCleanC.chain += c.chain[ i ];
  // 		  ++j;
  // 		  mc.increment( i );
  // 		  it.nextInLoop();
  // 		}
  // 	    }
  // 	  itn = it;
  // 	  itn.nextInLoop();
  // 	}
  //     if ( it == it_begin ) break;
  //   } //   while ( true )
  // // Process waiting steps.
  // while ( ! fc_to_process.empty() )
  //   {
  //     cout << fc_to_process.front() << " at " << idx_to_process.front() << endl;
  //     aCleanC.chain += fc_to_process.front();
  //     fc_to_process.pop_front();
  //     aC2clean.push_back( j ) ; 
  //     aClean2c.push_back( idx_to_process.front() );
  //     // aC2clean[ idx_to_process.front() ] = j;
  //     // aClean2c[ j ] = idx_to_process.front();
  //     idx_to_process.pop_front();
  //     ++j;
  //   }


  return true;
}

/**
 * Given a Freeman chain [c] coding a 4-connected pixel loop, computes
 * its subsampling by the transformation:
 * X = ( x - x0 ) div h, 
 * Y = ( y - y0 ) div v.
 *
 * @param subc (output) the subsampled Freeman chain code (may contain spikes)
 * 
 * @param c2subc (output) the mapping associating an element to
 * its subsampled element.
 *
 * @param subc2c (output) the inverse mapping associating a
 * subsampled element to its element.
 *
 * @param c the input chain code.
 *
 * @param h the subsampling along x
 * @param v the subsampling along y
 * @param x0 the x-origin of the frame (X,Y) in (x,y)
 * @param y0 the y-origin of the frame (X,Y) in (x,y)
 *
 * @return 'false' if initial contour was empty or if [subc] is empty,
 * 'true' otherwise.
 */
bool
DGtal::FreemanChain::subsample( FreemanChain & aSubc,
				  std::vector<unsigned int> & aC2subc,
				  std::vector<unsigned int> & aSubc2c,
				  const FreemanChain & c,
				  unsigned int h, unsigned int v,
				  int x0, int y0 )
{
  if ( ( h == 0 ) || ( v == 0 ) ) return false;
  FreemanChain::const_iterator it = c.begin();
  unsigned int j = 0;
  unsigned int nb = c.chain.size();
  if ( nb == 0 ) return false;

  PointI2D fxy( it.get() );
  PointI2D fXY;
  fXY.at(0)= ( fxy.at(0) - x0 ) / h; 
  fXY.at(1)= ( fxy.at(1) - y0 ) / v; 
  
  
  aSubc.x0 = fXY.at(0);
  aSubc.y0 = fXY.at(1);
  aC2subc.clear();
  aC2subc.reserve( nb );
  aSubc2c.clear();
  aSubc2c.reserve( nb );

  for ( unsigned int i = 0; i < nb; ++i )
    {
      aC2subc.push_back( j );
      it.nextInLoop();
      PointI2D nxy( it.get() );
      PointI2D nXY;
      nXY.at(0)= ( nxy.at(0) - x0 ) / h;
      nXY.at(1)= ( nxy.at(1) - y0 ) / v;

      
      if ( nXY != fXY )
	{
	  aSubc2c.push_back( i );
	  char code;
	  if ( nXY.at(0) > fXY.at(0) )       code = '0';
	  else if ( nXY.at(0) < fXY.at(0) )  code = '2';
	  else if ( nXY.at(1) > fXY.at(1) )  code = '1';
	  else                           code = '3';
	  aSubc.chain += code;
	  ++j;
	  fXY = nXY;
	}
    }
//   aC2subc.push_back( j );
//   it.nextInLoop();
//   for ( unsigned int i = 1; i <= nb; ++i )
//     {
//       // JOL 
//       //aC2subc.push_back( j );
//       Vector2i nxy( it.get() );
//       Vector2i nXY( ( nxy.x() - x0 ) / h, ( nxy.y() - y0 ) / v );
//       if ( nXY != fXY )
// 	{
// 	  char code;
// 	  if ( nXY.x() > fXY.x() )       code = '0';
// 	  else if ( nXY.x() < fXY.x() )  code = '2';
// 	  else if ( nXY.y() > fXY.y() )  code = '1';
// 	  else                           code = '3';
// 	  aSubc.chain += code;
// 	  aSubc2c.push_back( i - 1 );
// 	  ++j;
// 	  fXY = nXY;
// 	}
//       if ( i != nb ) aC2subc.push_back( j );
//       it.nextInLoop();
//     }
//   // TODO : enhance this.
  unsigned int nbsub =  aSubc.chain.size();
  // Last correspondence may be in fact to 0 instead of nbsub.
  if ( nbsub != 0 )
    for ( unsigned int i = 0; i < nb; ++i )
      if ( aC2subc[ i ] >= nbsub ) aC2subc[ i ] -= nbsub;


  ASSERT( aC2subc.size() == nb );
  return nbsub != 0;
}


// /**
//  * Computes the Minimum Length Polygon (MLP) of the Freeman chain [fc].
//  *
//  * @param vx (returns) the x-coordinates of the MLP vertices.
//  * @param vy (returns) the y-coordinates of the MLP vertices.
//  * @param vi (returns) the indices of the MLP vertices in [fc].
//  * @param fc the input Freeman chain.
//  *
//  * @return 'true' if the MLP has made a correct loop, 'false'
//  * otherwise (error ?).
//  */
// bool
// DGtal::FreemanChain::computeMLP( std::vector<int> & vx,
// 				   std::vector<int> & vy,
// 				   std::vector<unsigned int> & vi,
// 				   const FreemanChain & fc )
// {
//   bool cvx = true;
//   OrderedAlphabet A( '0', 4 );
//   FreemanChain::const_iterator it = fc.findQuadrantChange4( A );
//   // char start_char = fc.chain[ it.getPosition() ];

//   unsigned int j = it.getPosition();
//   unsigned int end = j;
//   // cerr << "end=" << end << endl; 
//   unsigned int nb_a1;
//   unsigned int nb_a2;
//   string w = fc.chain;
//   // cerr << w << endl;
//   PointI2D  xy = *it;
//   do 
//     {
//       vx.push_back( xy.x() );
//       vy.push_back( xy.y() );  
//       vi.push_back( j );
// //       cerr << "- (" << xy.x() << "," << xy.y() << ") at " << j << endl;
//       unsigned int l = A.nextEdge( nb_a1, nb_a2, w, j, cvx );
// //       cerr << "  nb_a1=" << nb_a1 << " nb_a2=" << nb_a2 << endl;
//       int dx;
//       int dy;
//       displacement( dx, dy, A.letter( 1 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a1;
//       else if ( dx < 0 ) xy.x() -= nb_a1;
//       else if ( dy > 0 ) xy.y() += nb_a1;
//       else if ( dy < 0 ) xy.y() -= nb_a1;
//       displacement( dx, dy, A.letter( 2 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a2;
//       else if ( dx < 0 ) xy.x() -= nb_a2;
//       else if ( dy > 0 ) xy.y() += nb_a2;
//       else if ( dy < 0 ) xy.y() -= nb_a2;
//     }
//   while ( j != end );
//   // cerr << w << endl;
//   bool loop_ok = ( ( xy.x() == vx[ 0 ] ) && ( xy.y() == vy[ 0 ] ) );
// //   if ( loop_ok )
// //     cerr << "Loop OK" << endl;
// //   else 
// //     cerr << "Loop ERROR" << endl;
//   return loop_ok;
// }

// /**
//  * Computes the Minimum Length Polygon (MLP) of the Freeman chain
//  * [fc]. Tells also if vertices are inside or outside.
//  *
//  * @param vx (returns) the x-coordinates of the MLP vertices.
//  * @param vy (returns) the y-coordinates of the MLP vertices.
//  * @param vi (returns) the indices of the MLP vertices in [fc].
//  *
//  * @param vt (returns) the type (inside=true, outside=false) of the
//  * MLP vertices in [fc].
//  *
//  * @param fc the input Freeman chain.
//  *
//  * @param cw when 'true', the inside is considered to be to the right
//  * of the contour (the contour turns clockwise around the inside),
//  * when 'false' the inside is considered to be to the left of the
//  * contour (the contour turns counterclockwise around the inside).
//  *
//  * @return 'true' if the MLP has made a correct loop, 'false'
//  * otherwise (error ?).
//  */
// bool
// DGtal::FreemanChain::computeMLP( std::vector<int> & vx,
// 				   std::vector<int> & vy,
// 				   std::vector<unsigned int> & vi,
// 				   std::vector<bool> & vt,
// 				   const FreemanChain & fc,
// 				   bool cw )
// {
//   OrderedAlphabet A( '0', 4 );
//   if ( cw ) A.reverseAround12();
//   FreemanChain::const_iterator it = fc.findQuadrantChange4( A );
//   FreemanChain::const_iterator itprev = it;
//   itprev.previousInLoop();
//   unsigned int mvt = FreemanChain::movement( itprev.getCode(), it.getCode(), ! cw );
  

//   //BK
//   //TODO
//   //ASSERT_FreemanChain( ( ( mvt == 1 ) || ( mvt == 3 ) )
//   //		       && "[DGtal::FreemanChain::computeMLP] Invalid start point." );
  

//   bool cvx = mvt == 1;

//   unsigned int j = it.getPosition();
//   unsigned int end = j;
//   // cerr << "end=" << end << endl; 
//   unsigned int nb_a1;
//   unsigned int nb_a2;
//   string w = fc.chain;
//   // cerr << w << endl;
//   PointI2D xy = *it;
//   do 
//     {
//       vx.push_back( xy.x() );
//       vy.push_back( xy.y() );  
//       vi.push_back( j );
//       vt.push_back( cvx );
//       // cerr << "(" << xy.x() << "," << xy.y() << ") at " << j;
//       unsigned int l = A.nextEdge( nb_a1, nb_a2, w, j, cvx );
//       // cerr << " nb_a1=" << nb_a1 << " nb_a2=" << nb_a2 << endl;
//       int dx;
//       int dy;
//       displacement( dx, dy, A.letter( 1 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a1;
//       else if ( dx < 0 ) xy.x() -= nb_a1;
//       else if ( dy > 0 ) xy.y() += nb_a1;
//       else if ( dy < 0 ) xy.y() -= nb_a1;
//       displacement( dx, dy, A.letter( 2 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a2;
//       else if ( dx < 0 ) xy.x() -= nb_a2;
//       else if ( dy > 0 ) xy.y() += nb_a2;
//       else if ( dy < 0 ) xy.y() -= nb_a2;
//     }
//   while ( j != end );
//   // cerr << w << endl;
//   bool loop_ok = ( ( xy.x() == vx[ 0 ] ) && ( xy.y() == vy[ 0 ] ) );
//   // if ( loop_ok )
//   //   cerr << "Loop OK" << endl;
//   // else 
//   //   cerr << "Loop ERROR" << endl;
//   return loop_ok;
// }


// /**
//  * Computes the Minimum Length Polygon (MLP) of the Freeman chain
//  * [fc]. Tells also if vertices are inside or outside. The MLP lies in
//  * the half-integer plane compared with the Freeman Chain. Vertex
//  * coordinates are given as integers. This function gives the
//  * displacement vector to go from the digital contour to the MLP.
//  *
//  * @param vx (returns) the x-coordinates of the MLP vertices.
//  * @param vy (returns) the y-coordinates of the MLP vertices.
//  * @param vi (returns) the indices of the MLP vertices in [fc].
//  *
//  * @param vt (returns) the type (inside=true, outside=false) of the
//  * MLP vertices in [fc].
//  *
//  * @param twice_dv (returns) twice the displacement vector to go from
//  * the integer plane of the Freeman chain to the half-integer plane of
//  * the MLP.
//  *
//  * @param fc the input Freeman chain.
//  *
//  * @param cw when 'true', the inside is considered to be to the right
//  * of the contour (the contour turns clockwise around the inside),
//  * when 'false' the inside is considered to be to the left of the
//  * contour (the contour turns counterclockwise around the inside).
//  *
//  * @return the iterator on the starting step of the MLP or 'end()' if
//  * there was an error (incorrect loop ?).
//  */
// DGtal::FreemanChain::const_iterator
// DGtal::FreemanChain::computeMLP( std::vector<int> & vx,
// 				   std::vector<int> & vy,
// 				   std::vector<unsigned int> & vi,
// 				   std::vector<bool> & vt,
// 				   Vector2i & twice_dv,
// 				   const FreemanChain & fc,
// 				   bool cw )
// {
//   OrderedAlphabet A( '0', 4 );
//   if ( cw ) A.reverseAround12();
//   FreemanChain::const_iterator it = fc.findQuadrantChange4( A );

//   // Compute displacement vector.
//   FreemanChain::const_iterator itprev = it;
//   itprev.previousInLoop();
//   //TODO :Initialisation à 0 par défaut à vérif
//   PointI2D v_it();
//   PointI2D v_itprev();
//   Vector2i v_itprev();
  
//   v_it.move4( it.getCode() );
//   v_itprev.move4( itprev.getCode() );
//   twice_dv = v_it - v_itprev;

//   unsigned int mvt = FreemanChain::movement( itprev.getCode(), it.getCode(), ! cw );
//   ASSERT_FreemanChain( ( ( mvt == 1 ) || ( mvt == 3 ) )
// 		       && "[DGtal::FreemanChain::computeMLP] Invalid start point." );
//   bool cvx = mvt == 1;
  

//   unsigned int j = it.getPosition();
//   unsigned int end = j;
//   // cerr << "end=" << end << endl; 
//   unsigned int nb_a1;
//   unsigned int nb_a2;
//   string w = fc.chain;
//   // cerr << w << endl;
//   PointI2D xy = *it;
//   do 
//     {
//       vx.push_back( xy.x() );
//       vy.push_back( xy.y() );  
//       vi.push_back( j );
//       vt.push_back( cvx );
//       // cerr << "(" << xy.x() << "," << xy.y() << ") at " << j;
//       unsigned int l = A.nextEdge( nb_a1, nb_a2, w, j, cvx );
//       // cerr << " nb_a1=" << nb_a1 << " nb_a2=" << nb_a2 << endl;
//       int dx;
//       int dy;
//       displacement( dx, dy, A.letter( 1 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a1;
//       else if ( dx < 0 ) xy.x() -= nb_a1;
//       else if ( dy > 0 ) xy.y() += nb_a1;
//       else if ( dy < 0 ) xy.y() -= nb_a1;
//       displacement( dx, dy, A.letter( 2 ) - '0' );
//       if ( dx > 0 )      xy.x() += nb_a2;
//       else if ( dx < 0 ) xy.x() -= nb_a2;
//       else if ( dy > 0 ) xy.y() += nb_a2;
//       else if ( dy < 0 ) xy.y() -= nb_a2;
//     }
//   while ( j != end );
//   // cerr << w << endl;
//   bool loop_ok = ( ( xy.x() == vx[ 0 ] ) && ( xy.y() == vy[ 0 ] ) );
//   // if ( loop_ok )
//   //   cerr << "Loop OK" << endl;
//   // else 
//   //   cerr << "Loop ERROR" << endl;
//   return loop_ok ? it : fc.end();
// }


// /**
//  * Computes the Minimum Length Polygon (MLP) of the Freeman chain [fc]
//  * and return its length.
//  *
//  * @param fc the input Freeman chain.
//  *
//  * @return the Euclidean length of the MLP.
//  */
// double
// DGtal::FreemanChain::lengthMLP( const FreemanChain & fc )
// {
//   OrderedAlphabet A( '0', 4 );
//   FreemanChain::const_iterator it = fc.findQuadrantChange4( A );
//   unsigned int j = it.getPosition();
//   unsigned int end = j;
//   unsigned int nb_a1;
//   unsigned int nb_a2;
//   string w = fc.chain;
//   double length = 0.0;
//   bool cvx = true;
//   do 
//     {
//       unsigned int l = A.nextEdge( nb_a1, nb_a2, w, j, cvx );
//       if ( l != 0 )
// 	{
// 	  double dx = (double) nb_a1;
// 	  double dy = (double) nb_a2;
// 	  length += sqrt( dx*dx + dy*dy );
// 	}
//     }
//   while ( j != end );
//   return length;
// }









///////////////////////////////////////////////////////////////////////////////
// Interface - public :

/**
 * Writes/Displays the object on an output stream.
 * @param out the output stream where the object is written.
 */
void
DGtal::FreemanChain::selfDisplay ( std::ostream & out ) const
{
    out << "[FreemanChain]";
}

/**
 * Checks the validity/consistency of the object.
 * @return 'true' if the object is valid, 'false' otherwise.
 */
bool
DGtal::FreemanChain::isValid() const
{
    return true;
}




///////////////////////////////////////////////////////////////////////////////
// Internals - private :

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
