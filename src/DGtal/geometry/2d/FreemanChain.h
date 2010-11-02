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
//LICENSE-END
#pragma once

/**
 * @file FreemanChain.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2010/07/01
 *
 * Header file for module FreemanChain.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FreemanChain_RECURSES)
#error Recursive header files inclusion detected in FreemanChain.h
#else // defined(FreemanChain_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FreemanChain_RECURSES

#if !defined FreemanChain_h
/** Prevents repeated inclusion of headers. */
#define FreemanChain_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <vector>
#include "DGtal/kernel/PointVector.h"
#include "DGtal/base/OrderedAlphabet.h"
#include "DGtal/base/BasicTypes.h"
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/ModuloComputer.h"
//////////////////////////////////////////////////////////////////////////////


namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class FreemanChain
  /////////////////////////////////////////////////////////////////////////////
  /** 
   * Description of class 'FreemanChain' <p> Aim: Describes a digital
   * 4-connected contour as a string of '0', '1', '2', and '3' and the
   * coordinate of the first point. When it is a loop, it is the
   * counterclockwise boundary of the shape.
   */

  template <typename TInteger>
  class FreemanChain
  {

  public :


    //BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
    typedef TInteger Integer;
    typedef PointVector<2,Integer> PointI2;



    
    
    // ------------------------- iterator ------------------------------
  public:
    /**
     * This class represents an iterator on the freeman chain, storing
     * the current coordinate.
     */
    class ConstIterator
    {
      // ------------------------- data -----------------------
    private:
      /**
       * The Freeman chain visited by the iterator.
       */
      const FreemanChain* myFc;

      /**
       * The current position in the word.
       */
      unsigned int myPos;

      /**
       * The current coordinates of the iterator.
       */
      PointI2  myXY;
      


      // ------------------------- Standard services -----------------------
    public:
      /**
       * Default Constructor.
       * The object is not valid.
       */
      ConstIterator();

      /**
       * Constructor.
       * Nb: complexity in O(n).
       *
       * @param aChain a Freeman chain,
       * @param n the position in [chain] (within 0 and chain.size()-1).
       */
      ConstIterator( const FreemanChain & aChain, unsigned int n = 0 );

      /**
       * Copy constructor.
       * @param other the iterator to clone.
       */
      ConstIterator( const ConstIterator & aOther );
    
      /**
       * Assignment.
       * @param aOther the iterator to copy.
       * @return a reference on 'this'.
       */
      ConstIterator& operator=( const ConstIterator & aOther );
    
      /**
       * Destructor. Does nothing.
       */
      ~ConstIterator();
    
      // ------------------------- iteration services -------------------------
    public:
      
      /**
       * @return the current coordinates.
       */
      PointI2 operator*() const;

      /**
       * @return the current coordinates.
       */
      PointI2 get() const;

      /**
       * Pre-increment.
       * Goes to the next point on the chain.
       */
      ConstIterator& operator++();
      
      /**
       * Goes to the next point on the chain.
       */
      void next();

      /**
       * Goes to the next point on the chain as if on a loop.
       */
      void nextInLoop();

      /**
       * @return the current position (as an index in the Freeman chain).
       */
      unsigned int getPosition() const;

      /**
       * @return the associated Freeman chain.
       */
      const FreemanChain* getChain() const;

      /**
       * @return the current Freeman code (specifies the movement to
       * the next point).
       */
      unsigned int getCode() const;

      /**
       * Pre-decrement.
       * Goes to the previous point on the chain.
       */
      ConstIterator& operator--();
      
      /**
       * Goes to the previous point on the chain if possible.
       */
      void previous();

      /**
       * Goes to the previous point on the chain as if on a loop.
       */
      void previousInLoop();

      /**
       * Equality operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if their current positions coincide.
       */
      bool operator==( const ConstIterator & aOther ) const;

      /**
       * Inequality operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if their current positions differs.
       */
      bool operator!=( const ConstIterator & aOther ) const;

      /**
       * Inferior operator.
       *
       * @param aOther the iterator to compare with (must be defined on
       * the same chain).
       *
       * @return 'true' if the current position of 'this' is before
       * the current position of [other].
       */
      bool operator<( const ConstIterator & aOther ) const;
      
    };
      

    // ------------------------- static services ------------------------------
  public:

    /**
     * Outputs the chain [c] to the stream [out].
     * @param out any output stream,
     * @param c a Freeman chain.
     */
    static void write( std::ostream & out, const FreemanChain & c ){
      out << c.x0 << " " << c.y0 << " " << c.chain << endl;
    }
    

    /**
     * Reads a chain from the stream [in] and updates [c].
     * @param in any input stream,
     * @param c (returns) the Freeman chain.
     */
    static void read( std::istream & in, FreemanChain & c ){
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
      
    };

    /**
     * Creates a Freeman chaincode [chain] and a chain coding the
     * quadrant of each step [qchain], given an iterator on a
     * 4-connected path [it], a number of step [nb], the first step
     * [freeman_code] and the first quadrant [quadrant].
     *
     * @param chain (returns) a string of '0', '1', '2', or '3'
     * (Freeman chaincode)
     *
     * @param qchain (returns) a string of '0', '1', '2', or '3'
     * giving the quadrants.
     *
     * @param it any iterator 
     *
     * @param nb the number of 'next' performed on a copy of [it].
     *
     * @param freeman the first code or step.
     *
     * @param quadrant the first quadrant (equal to freeman_code or one below).
     *
     * @param start_index the starting index in [chain] and [qchain],
     * default is 0.
     */
    
    //static void create( std::string & chain,
    //			std::string & qchain,
    //			const C4CIterator & it, 
    //			unsigned int nb, unsigned int freeman, unsigned int quadrant,
    //			unsigned int start_index = 0 );
  
  
    /**
     * @param aZero (returns) the '0' or 'x' letter for quadrant [quadrant].
     * @param aOne (returns) the '1' or 'y' letter for quadrant [quadrant].
     * @param aQuadrant the quadrant as any of '0', '1', '2', or '3'.
     */
    static void alphabet( char & aZero, char & aOne, char aQuadrant ){
        switch ( aQuadrant )
	  {
	  case '0': aZero = '0'; aOne = '1'; break;
	  case '1': aZero = '1'; aOne = '2'; break;
	  case '2': aZero = '2'; aOne = '3'; break;
	  case '3': aZero = '3'; aOne = '0'; break;
	  }
	
    };

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
    static unsigned int movement( unsigned int aCode1, unsigned int aCode2, bool ccw = true ){
        unsigned int cfg = ( ccw ? 0 : 16 ) + ( aCode1 << 2 ) + aCode2;
	static const unsigned int tbl[ 32 ] = { 
	  2, 1, 0, 3, 3, 2, 1, 0,
	  0, 3, 2, 1, 1, 0, 3, 2, 
	  2, 3, 0, 1, 1, 2, 3, 0,
	  0, 1, 2, 3, 3, 0, 1, 2 };
	return tbl[ cfg ];
  
    }

    /**
     * Returns the displacement vector of a Freeman code.
     *
     * @param dx (returns) the x-displacement.
     * @param dy (returns) the y-displacement.
     * @param aCode the code.
     */
    static void displacement( int & dx, int & dy, unsigned int aCode );

    /**
     * @param aCode a Freeman code (between 0-3).
     * Returns the displacement vector of the Freeman code.
     */
    static PointI2 displacement( unsigned int aCode );

    /**
     * @param aCode any Freeman code.
     *
     * @param ccw when 'true' turns counterclockwise (or left),
     * otherwise turns clockwise (right).
     *
     * @return the turned code.
     */
    static unsigned int turnedCode( unsigned int aCode, bool ccw = true );
    
    /**
     * From the Freeman chain [pl_chain] representing a pointel
     * 4-connected contour, constructs the Freeman chain [pix_chain]
     * that represents its inner 4-connected border of pixels. The
     * Freeman chain [pl_chain] has its inside to the left (ie. ccw).
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
     * @param pl_chain the input code of the 4-connected pointel contour.
     */
    static void pointel2pixel( FreemanChain & aPixChain,
			       std::vector<unsigned int> & aPl2pix,
			       std::vector<unsigned int> & aPix2pl,
			       const FreemanChain & aPlChain ){
      innerContour( aPixChain, aPl2pix, aPix2pl, aPlChain, true );
    };

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
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     */
    static void innerContour( FreemanChain & aInnerChain,
			      std::vector<unsigned int> & aOuter2inner,
			      std::vector<unsigned int> & aInner2outer,
			      const FreemanChain & aOuterChain,
			      bool ccw = true ){
      
        unsigned int nb = aOuterChain.chain.size();
  unsigned int j = 0;
  aOuter2inner.clear();
  aOuter2inner.reserve( nb );
  // aInnerChain.chain.reserve( nb + 4 );
  aInnerChain.chain = "";
  aInner2outer.clear();
  aInner2outer.reserve( nb + ( ccw ? 4 : -4 ) );
  int dx0, dy0;
  int dx1, dy1;
  FreemanChain<TInteger>::displacement( dx0, dy0, aOuterChain.code( 0 ) );
  int turn = ccw ? 1 : 3;
  FreemanChain<TInteger>::displacement( dx1, dy1, ( aOuterChain.code( 0 ) + turn ) % 4 );
  dx0 += dx1;
  dy0 += dy1;
  aInnerChain.x0 = dx0 > 0 ? aOuterChain.x0 : aOuterChain.x0 - 1;
  aInnerChain.y0 = dy0 > 0 ? aOuterChain.y0 : aOuterChain.y0 - 1;

  FreemanChain<TInteger>::ConstIterator it_begin = aOuterChain.begin();
  FreemanChain<TInteger>::ConstIterator it = it_begin;
  it.next();
  for ( unsigned int i = 0; i < nb; ++i )
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
	  aInnerChain.chain += ( ( ( (unsigned int) ( aOuterChain.chain[ i ] - '0' ) 
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

     * @param aClean_cs (output) the array of cleaned 4-connected contours.
     *
     * @param aC2clean (output) the mapping associating an element to
     * its clean element as a pair (n,i) where n is the index of the
     * cleaned contour and i the indice of the element in this Freeman
     * chain.
     *
     * @param aClean2c (output) the array of mapping associating a
     * clean element to its non-clean element. clean2c[n][j] gives the
     * index of the non-clean element on c corresponding to the clean
     * element of index j in the n-th contour.
     *
     * @param c the input code of the 4-connected contour.
     *
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     *
     * @todo This method is not implemented.
     */
    static void cleanContour( std::vector<FreemanChain> & aCleanCs,
			      std::vector< std::pair<unsigned int,unsigned int> > & aC2clean,
			      std::vector< std::vector<unsigned int> > & aClean2c,
			      const FreemanChain & c,
			      bool ccw = true ){

    }
    /**
     * Removes outer spikes along a 4-connected contour, meaning steps
     * "02", "13", "20" or "31", which point outside the shape. The
     * inside is given by parameter [ccw]. Note that 4-connected
     * pointel contours should not have any outer spikes, while
     * 4-connected pixel contours should not have any inner spikes.
     *
     * @param aClean_c (output) the cleaned 4-connected contour.
     *
     * @param aC2clean (output) the mapping associating an element to
     * its clean element.
     *
     * @param aClean2c (output) the inverse mapping associating a
     * clean element to its non-clean element. 
     *
     * @param c the input code of the 4-connected contour (should be a loop !).
     *
     * @param ccw 'true' if the contour is seen counterclockwise with
     * its inside to the left.
     *
     * @return 'true' if the contour add an interior, 'false' otherwise.
     */
    static bool cleanOuterSpikes( FreemanChain & aCleanC,
				  std::vector<unsigned int> & aC2clean,
				  std::vector<unsigned int> & aClean2c,
				  const FreemanChain & c,
				  bool ccw = true ){
  unsigned int nb = c.chain.size();
  if ( nb == 0 ) 
    {
      cerr << "[DGtal::FreemanChain::cleanOuterSpikes]"
	   << " cleanOuterSpikes: Empty input chain"
	   << endl;
      return false;
    }

  ModuloComputer< DGtal::int32_t > mc( nb );
  ModuloComputer< DGtal::int32_t >::UnsignedInteger i = 0;
  ModuloComputer< DGtal::int32_t >::UnsignedInteger j = 0;
  vector<unsigned int> c2cleanTMP;
  aCleanC.chain.reserve( nb );
  aCleanC.chain = "";
  aC2clean.clear();
  aClean2c.clear();
  aC2clean.reserve( nb );
  aClean2c.reserve( nb );
  c2cleanTMP.reserve( nb );
  FreemanChain<TInteger>::ConstIterator it = c.begin();
  FreemanChain<TInteger>::ConstIterator itn = c.begin(); itn.nextInLoop();
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
  PointI2 P = *it;
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
  FreemanChain<TInteger>::ConstIterator it_begin = it;
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



  return true;
    };

//     /**
//      * Given a Freeman chain [c] coding a 4-connected pixel loop, computes
//      * its subsampling by the transformation:
//      * X = ( x - x0 ) div h, 
//      * Y = ( y - y0 ) div v.
//      *
//      * @param aSubc (output) the subsampled Freeman chain code (may
//      * contain spikes)
//      * 
//      * @param aC2subc (output) the mapping associating an element to
//      * its subsampled element.
//      *
//      * @param aSubc2c (output) the inverse mapping associating a
//      * subsampled element to its element. More precisely, subc2c[ j ]
//      * is the last pointel to be in j.
//      *
//      * @param c the input chain code.
//      *
//      * @param h the subsampling along x
//      * @param v the subsampling along y
//      * @param x0 the x-origin of the frame (X,Y) in (x,y)
//      * @param y0 the y-origin of the frame (X,Y) in (x,y)
//      *
//      * @return 'false' if initial contour was empty or if [subc] is empty,
//      * 'true' otherwise.
//      */
//     static bool subsample( FreemanChain & aSubc,
// 			   std::vector<unsigned int> & aC2subc,
// 			   std::vector<unsigned int> & aSubc2c,
// 			   const FreemanChain & c,
// 			   unsigned int h, unsigned int v,
// 			   int x0, int y0 ){
//       if ( ( h == 0 ) || ( v == 0 ) ) return false;
//       FreemanChain<TInteger>::ConstIterator it = c.begin();
//   unsigned int j = 0;
//   unsigned int nb = c.chain.size();
//   if ( nb == 0 ) return false;

//   PointI2 fxy( it.get() );
//   PointI2 fXY;
//   fXY.at(0)= ( fxy.at(0) - x0 ) / h; 
//   fXY.at(1)= ( fxy.at(1) - y0 ) / v; 
  
  
//   aSubc.x0 = fXY.at(0);
//   aSubc.y0 = fXY.at(1);
//   aC2subc.clear();
//   aC2subc.reserve( nb );
//   aSubc2c.clear();
//   aSubc2c.reserve( nb );

//   for ( unsigned int i = 0; i < nb; ++i )
//     {
//       aC2subc.push_back( j );
//       it.nextInLoop();
//       PointI2 nxy( it.get() );
//       PointI2 nXY;
//       nXY.at(0)= ( nxy.at(0) - x0 ) / h;
//       nXY.at(1)= ( nxy.at(1) - y0 ) / v;

      
//       if ( nXY != fXY )
// 	{
// 	  aSubc2c.push_back( i );
// 	  char code;
// 	  if ( nXY.at(0) > fXY.at(0) )       code = '0';
// 	  else if ( nXY.at(0) < fXY.at(0) )  code = '2';
// 	  else if ( nXY.at(1) > fXY.at(1) )  code = '1';
// 	  else                           code = '3';
// 	  aSubc.chain += code;
// 	  ++j;
// 	  fXY = nXY;
// 	}
//     }
// //   aC2subc.push_back( j );
// //   it.nextInLoop();
// //   for ( unsigned int i = 1; i <= nb; ++i )
// //     {
// //       // JOL 
// //       //aC2subc.push_back( j );
// //       Vector2i nxy( it.get() );
// //       Vector2i nXY( ( nxy.x() - x0 ) / h, ( nxy.y() - y0 ) / v );
// //       if ( nXY != fXY )
// // 	{
// // 	  char code;
// // 	  if ( nXY.x() > fXY.x() )       code = '0';
// // 	  else if ( nXY.x() < fXY.x() )  code = '2';
// // 	  else if ( nXY.y() > fXY.y() )  code = '1';
// // 	  else                           code = '3';
// // 	  aSubc.chain += code;
// // 	  aSubc2c.push_back( i - 1 );
// // 	  ++j;
// // 	  fXY = nXY;
// // 	}
// //       if ( i != nb ) aC2subc.push_back( j );
// //       it.nextInLoop();
// //     }
// //   // TODO : enhance this.
//   unsigned int nbsub =  aSubc.chain.size();
//   // Last correspondence may be in fact to 0 instead of nbsub.
//   if ( nbsub != 0 )
//     for ( unsigned int i = 0; i < nb; ++i )
//       if ( aC2subc[ i ] >= nbsub ) aC2subc[ i ] -= nbsub;


//   ASSERT( aC2subc.size() == nb );
//   return nbsub != 0;

//     }


  
    /**
     * Return a vector containing all the interger points of the freemanchain.
     *
     * @param fc the FreemanChain
     * @param aVContour (returns) the vector containing all the integer contour points.
     */
    static void getContourPoints(const FreemanChain & fc, std::vector<PointI2> & aVContour){
      aVContour.clear();
      for ( FreemanChain<TInteger>::ConstIterator it = fc.begin();
	    it != fc.end();
	    ++it )
	{
	  aVContour.push_back(*it);
	}
    }

  
  
  
  
    static void movePointFromFC(PointI2 & aPoint, unsigned int aCode );



    // ----------------------- Standard services ------------------------------
  public:

  

  


    /**
     * Destructor.
     */
    ~FreemanChain(){
    };
    
    /**
     * Constructor.
     * @param s the chain code.
     * @param x the x-coordinate of the first point.
     * @param y the y-coordinate of the first point.
     */
    FreemanChain( const std::string & s = "", int x = 0, int y = 0 );


    /**
     * Constructor.
     * @param in any input stream,
     */
    FreemanChain(std::istream & in );


    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    FreemanChain( const FreemanChain & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    FreemanChain & operator=( const FreemanChain & other );


    /**
     * Iterator service.
     * @return an iterator pointing on the first point of the chain.
     */
    FreemanChain::ConstIterator begin() const;

    /**
     * Iterator service.
     * @return an iterator pointing after the last point of the chain.
     */
    FreemanChain::ConstIterator end() const;

    /**
     * @param pos a position in the chain code.
     * @return the code at position [pos].
     */ 
    unsigned int code( unsigned int pos ) const;

    /**
     * @param pos a position in the chain code.
     * @return the next position.
     */ 
    unsigned int next( unsigned int pos ) const;

    /**
     * @param pos a position in the chain code.
     * @return the previous position.
     */ 
    unsigned int previous( unsigned int pos ) const;

    /**
     * @return the length of the Freeman chain code.
     */
    unsigned int size() const;

    /**
     * Computes a bounding box for the Freeman chain code.
     *
     * @param min_x (returns) the minimal x-coordinate.
     * @param min_y (returns) the minimal y-coordinate.
     * @param max_x (returns) the maximal x-coordinate.
     * @param max_y (returns) the maximal y-coordinate.
     */
    void computeBoundingBox( int & min_x, int & min_y, 
			     int & max_x, int & max_y ) const{
      
        min_x = max_x = x0;
  min_y = max_y = y0;
  for ( FreemanChain<TInteger>::ConstIterator it = begin();
	it != end();
	++it )
    {
      PointI2 p( *it );
      if ( p.at(0) < min_x ) min_x = p.at(0);
      else if ( p.at(0) > max_x ) max_x = p.at(0);
      if ( p.at(1) < min_y ) min_y = p.at(1);
      else if ( p.at(1) > max_y ) max_y = p.at(1);
    }

    }

    /**
     * Finds a quadrant change in 'this' Freeman chain and returns the
     * position as an iterator. A quadrant change is some
     <code>
     abb..bc
     |
     iterator
     <endcode>
     *
     * The alphabet is possibly re-ordered so that a > b > c.
     *
     * @param A (possibly updated) a Freeman chain alphabet, possibly
     * re-ordered so that a > b > c.
     *
     * @return an iterator on 'this' that points on the first letter b.
     */
    
  
    //BK
    FreemanChain::ConstIterator
    findQuadrantChange( OrderedAlphabet & A ) const{
        DGtal::FreemanChain<TInteger>::ConstIterator it = begin();
  DGtal::FreemanChain<TInteger>::ConstIterator it_end = end();
  // find first letters a and b.
  unsigned int code1 = it.getCode();
  it.next();
  while ( ( it != it_end ) && ( it.getCode() == code1 ) )
    it.next();
  ASSERT( ( it != it_end ) 
	  && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] 1-letter freeman chain." );
  unsigned int  code2 = it.getCode();
  // find third letter c.
  while ( ( it != it_end ) && ( ( it.getCode() == code1 )
				|| ( it.getCode() == code2 ) ) )
    it.next();
  ASSERT( ( it != it_end ) 
		       && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] 2-letters Freeman chain." );
  unsigned int code3 = it.getCode();
  // reorder a and b.
  it.previous();
  if ( it.getCode() != code2 ) swap( code1, code2 );
  // find first a.
  do 
    { 
      it.previous();
    }
  while ( it.getCode() == code2 );
  char a_char = chain[ it.getPosition() ];
  // the next is the first b.
  it.next();
  char b_char = chain[ it.getPosition() ];
  // Reorder the alphabet to match the quadrant change.
  while ( A.order( b_char ) != 1 )
    A.shiftLeft();
  if ( A.order( a_char ) == 0 )
    {
      A.reverse();
      while ( A.order( b_char ) != 1 )
	A.shiftLeft();
    }
  ASSERT( ( A.order( b_char ) == 1 )
		       && ( A.order( a_char ) == 2 )
		       && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] Internal error: invalid Quadrant change found." );
  return it;


    }
  
    /**
     * Finds a quadrant change in 'this' Freeman chain and returns the
     * position as an iterator. A quadrant change is some
     <code>
     (abc)*bc...cd
     |
     iterator
     <endcode>
     *
     * This quadrant change also guarantees that is not a place where a
     * convexity change occurs in the combinatorial MLP algorithm.
     *
     * The alphabet is possibly re-ordered so that b > c > d > a.
     *
     * @param A (possibly updated) a Freeman chain alphabet, possibly
     * re-ordered so that b > c > d > a.
     *
     * @return an iterator on 'this' that points on the first letter c.
     */
  
    //BK  
    FreemanChain::ConstIterator
    findQuadrantChange4( OrderedAlphabet & A ) const{
        FreemanChain<TInteger>::ConstIterator it = begin();
  FreemanChain<TInteger>::ConstIterator it_end = end();
  // find first letters a and b.
  uint code1 = it.getCode();
  it.next();
  while ( ( it != it_end ) && ( it.getCode() == code1 ) )
    it.next();
  ASSERT( ( it != it_end ) 
	  && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] 1-letter freeman chain." );
  uint code2 = it.getCode();
  // find third letter c.
  while ( ( it != it_end ) && ( ( it.getCode() == code1 )
				|| ( it.getCode() == code2 ) ) )
    it.next();
  ASSERT( ( it != it_end ) 
	  && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] 2-letters Freeman chain." );
  unsigned int code3 = it.getCode();
  // find fourth letter d.
  while ( ( it != it_end ) && ( ( it.getCode() == code1 )
				|| ( it.getCode() == code2 )
				|| ( it.getCode() == code3 ) ) )
    it.next();
  ASSERT( ( it != it_end ) 
		       && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] 3-letters Freeman chain." );
  unsigned int  code4 = it.getCode();
  // define true c.
  it.previous();
  code3 = it.getCode();
  // find first b.
  do 
    { 
      it.previous();
    }
  while ( it.getCode() == code3 );
  char a_char = chain[ it.getPosition() ];
  // the next is the first c.
  it.next();
  char b_char = chain[ it.getPosition() ];
  // Reorder the alphabet to match the quadrant change.
  while ( A.order( b_char ) != 1 )
    A.shiftLeft();
  if ( A.order( a_char ) == 0 )
    {
      A.reverse();
      while ( A.order( b_char ) != 1 )
	A.shiftLeft();
    }
  ASSERT( ( A.order( b_char ) == 1 )
	  && ( A.order( a_char ) == 2 )
	  && "[DGtal::FreemanChain::findQuadrantChange( OrderedAlphabet & A ) const] Internal error: invalid Quadrant change found." );
  return it;

    }

    /**
     * This method takes O(n) operations and works only for Freeman
     * chains whose successive codes are between +1/-1. It determines
     * if the FreemanChain corresponds to a closed contour, and if
     * this is the case, determines how many counterclockwise loops the
     * contour has done. Of course, it the contour has done
     * clockwise loops, then the given number is accordingly
     * negative.
     *
     * @return the number of counterclockwise loops, or '0' is the contour
     * is open or invalid.
     */
    int isClosed() const{
        FreemanChain<TInteger>::ConstIterator it = this->begin();
  FreemanChain<TInteger>::ConstIterator it_end = this->end();
  FreemanChain<TInteger>::ConstIterator it_suiv = it;
  PointI2 spos = *it;
  int nb_ccw_turns = 0;
  while ( it != it_end )
    {
      int code1 = it.getCode();
      it_suiv.nextInLoop();
      int code2 = it_suiv.getCode();
      uint diff = ( code2 - code1 + 4 ) % 4;
      if ( diff == 1 )      ++nb_ccw_turns;
      else if ( diff == 3 ) --nb_ccw_turns;
      else if ( diff == 2 ) return 0;
      ++it;
    }
  if ( spos == *it_suiv )
    return nb_ccw_turns / 4;
  else return 0;


    }
  

    // ----------------------- Interface --------------------------------------
  public:

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const{
          out << "[FreemanChain]";
    };

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const{
          return true;
    };

     public:



    /*
      Draw the object on a LibBoard board
      @param board the output board where the object is drawn.
      @tparam Functor a Functor to specialize the Board style
    */
    template<typename Functor>
    void selfDraw(LibBoard::Board & board ) const;
    


    /*
      Draw the object on a LibBoard board
      @param board the output board where the object is drawn.
    */
    
    void selfDraw(LibBoard::Board & board ) const
    {
      selfDraw<SelfDrawStyle>(board);
    };
    

    

    // ------------------------- Public Datas ------------------------------

  public:
    /**
     * The chain code.
     */
    std::string chain;

    /**
     * the x-coordinate of the first point.
     */
    int x0;

    /**
     * the y-coordinate of the first point.
     */
    int y0;

    

    // ------------------------- Protected Datas ------------------------------
  private:
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    //    FreemanChain();

  private:


    // ------------------------- Internals ------------------------------------
  private:

   /** 
     * Default Style Functor for selfDraw methods
     * 
     * @param aBoard 
     */

    struct SelfDrawStyle
    {
      SelfDrawStyle(LibBoard::Board & aBoard) 
      {
				aBoard.setFillColor(LibBoard::Color::None);
				aBoard.setPenColor(LibBoard::Color::Black);
      }
    };
    

  }; // end of class FreemanChain




  /**
   * Overloads 'operator<<' for displaying objects of class 'FreemanChain'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FreemanChain' to write.
   * @return the output stream after the writing.
   */
  template<typename TInteger>
  std::ostream&
  operator<< ( std::ostream & out, const FreemanChain<TInteger> & object );
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods if necessary.
#if defined(INLINE)
#include "DGtal/geometry/2d/FreemanChain.ih"
#endif

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FreemanChain_h

#undef FreemanChain_RECURSES
#endif // else defined(FreemanChain_RECURSES)
