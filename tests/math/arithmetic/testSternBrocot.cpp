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
 * @file testSternBrocot.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/03/05
 *
 * Functions for testing class SternBrocot.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cstdlib>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/math/arithmetic/IntegerComputer.h"
#include "DGtal/math/arithmetic/SternBrocot.h"
#include "DGtal/math/arithmetic/Pattern.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SternBrocot.
///////////////////////////////////////////////////////////////////////////////

template <typename SB>
bool testInitFraction()
{
  typedef typename SB::Integer Integer;
  typedef typename SB::Fraction Fraction;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Integer p = random() / 10000;
  Integer q = random() / 10000;
  trace.beginBlock ( "Testing block: init fraction." );
  IntegerComputer<Integer> ic;
  Integer g = ic.gcd( p, q );
  p /= g;
  q /= g;
  Fraction f1 = SB::fraction( p, q );
  trace.info() << "p / q = " << p << " / " << q << std::endl;
  trace.info() << "f1 = ";
  SB::display( trace.info(), f1 );
  trace.info() << std::endl;
  nbok += ( ( p == f1.p() ) && ( q == f1.q() ) ) ? 1 : 0;
  ++nb;
  trace.info() << "(" << nbok << "/" << nb << ") " 
               << "( ( p == f1.p() ) && ( q == f1.q() ) )"
               << std::endl;
  trace.info() << "- nbFractions = " << SB::nbFractions << std::endl;
  trace.endBlock();

  return nbok == nb;
}

template <typename SB>
bool testPattern()
{
  typedef typename SB::Integer Integer;
  typedef typename SB::Size Size;
  typedef typename SB::Fraction Fraction;
  typedef Pattern<Fraction> MyPattern;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  Integer p = random() / 10000;
  Integer q = random() / 10000;
  MyPattern pattern( p*6, q*6 );
  trace.info() << pattern << endl;

  // ODD PATTERN
  trace.beginBlock ( "Testing block: Smallest covering subpatterns of ODD pattern." );
  MyPattern pat_odd( 5, 12 );
  trace.info() << "ODD  " << pat_odd << " " << pat_odd.rE() << endl;
  MyPattern sp;
  Size np;
  Integer start;
  bool mod;

  // Left Subpatterns
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 17 );
  trace.info() << "sub(0,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                1, 17 );
  trace.info() << "sub(1,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                7, 17 );
  trace.info() << "sub(7,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                8, 17 );
  trace.info() << "sub(8,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                13, 17 );
  trace.info() << "sub(13,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                14, 17 );
  trace.info() << "sub(14,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                15, 17 );
  trace.info() << "sub(15,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering left Subpatterns." << endl;

  // Right Subpatterns
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 16 );
  trace.info() << "sub(0,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 15 );
  trace.info() << "sub(0,15) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 14 );
  trace.info() << "sub(0,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 8 );
  trace.info() << "sub(0,8) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 7 );
  trace.info() << "sub(0,7) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                0, 1 );
  trace.info() << "sub(0,1) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering right Subpatterns." << endl;

  // Middle Subpatterns
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                1, 16 );
  trace.info() << "sub(1,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                2, 14 );
  trace.info() << "sub(2,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                7, 15 );
  trace.info() << "sub(7,15) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                7, 14 );
  trace.info() << "sub(7,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                3, 6 );
  trace.info() << "sub(3,6) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                6, 8 );
  trace.info() << "sub(6,8) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                8, 12 );
  trace.info() << "sub(8,12) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getSmallestCoveringSubpattern( sp, np, start,
                                                15, 16 );
  trace.info() << "sub(15,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) && np == 1 ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering middle Subpatterns." << endl;
  trace.endBlock();

  // EVEN PATTERN
  trace.beginBlock ( "Testing block: Smallest covering subpatterns of EVEN pattern." );
  MyPattern pat_even( 12, 17 );
  trace.info() << "EVEN " << pat_even << " " << pat_even.rE() << endl;

  // Left Subpatterns
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 0, 29 );
  trace.info() << "sub(0,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 12, 17 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 0, 25 );
  trace.info() << "sub(0,25) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 12, 17 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 0, 17 );
  trace.info() << "sub(0,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 7, 10 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 0, 6 );
  trace.info() << "sub(0,6) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 7, 10 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 0, 5 );
  trace.info() << "sub(0,5) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 3 ) ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering left Subpatterns." << endl;

  // Right Subpatterns
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 4, 29 );
  trace.info() << "sub(4,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 12, 17 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 5, 29 );
  trace.info() << "sub(5,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 2 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 16, 29 );
  trace.info() << "sub(16,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 2 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 17, 29 );
  trace.info() << "sub(17,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering right Subpatterns." << endl;

  // Middle Subpatterns
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 1, 27 );
  trace.info() << "sub(1,27) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 12, 17 ) ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 5, 24 );
  trace.info() << "sub(5,24) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 2 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 4, 17 );
  trace.info() << "sub(4,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 7, 10 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 5, 17 );
  trace.info() << "sub(5,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 7, 12 );
  trace.info() << "sub(7,12) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 1, 4 );
  trace.info() << "sub(1,4) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 3 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getSmallestCoveringSubpattern( sp, np, start,
                                                 18, 25 );
  trace.info() << "sub(18,20) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering middle Subpatterns." << endl;
  trace.endBlock();

  
  // GREATEST INCLUDED SUBPATTERN
  // ODD PATTERN
  trace.beginBlock ( "Testing block: greatest included subpatterns of ODD pattern." );
  trace.info() << "ODD  " << pat_odd << " " << pat_odd.rE() << endl;

  // Left Subpatterns
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 17 );
  trace.info() << "sub(0,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 12 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                1, 17 );
  trace.info() << "sub(1,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                7, 17 );
  trace.info() << "sub(7,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 3, 7 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                8, 17 );
  trace.info() << "sub(8,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                13, 17 );
  trace.info() << "sub(13,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                14, 17 );
  trace.info() << "sub(14,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 1, 2 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                15, 17 );
  trace.info() << "sub(15,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering left Subpatterns." << endl;

  // Right Subpatterns
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 15 );
  trace.info() << "sub(0,15) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 14 );
  trace.info() << "sub(0,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 2 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 13 );
  trace.info() << "sub(0,13) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 7 );
  trace.info() << "sub(0,7) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 6 );
  trace.info() << "sub(0,6) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                0, 1 );
  trace.info() << "sub(0,1) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering right Subpatterns." << endl;

  // Middle Subpatterns
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                1, 16 );
  trace.info() << "sub(1,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                2, 14 );
  trace.info() << "sub(2,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                7, 15 );
  trace.info() << "sub(7,15) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                7, 14 );
  trace.info() << "sub(7,14) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 5 ) && np == 1 ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                3, 6 );
  trace.info() << "sub(3,6) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                6, 8 );
  trace.info() << "sub(6,8) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                8, 12 );
  trace.info() << "sub(8,12) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_odd.getGreatestIncludedSubpattern( sp, np, start,
                                                15, 16 );
  trace.info() << "sub(15,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;

  trace.info() << "(" << nbok << "/" << nb << ") covering middle Subpatterns." << endl;
  trace.endBlock();

  // EVEN PATTERN
  trace.beginBlock ( "Testing block: Greatest included subpatterns of EVEN pattern." );
  trace.info() << "EVEN " << pat_even << " " << pat_even.rE() << endl;

  // Left Subpatterns
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 29 );
  trace.info() << "sub(0,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 12, 17 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 25 );
  trace.info() << "sub(0,25) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 7, 10 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 17 );
  trace.info() << "sub(0,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 7, 10 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 16 );
  trace.info() << "sub(0,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 3 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 6 );
  trace.info() << "sub(0,6) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 3 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 5 );
  trace.info() << "sub(0,5) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 2, 3 ) ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 0, 4 );
  trace.info() << "sub(0,4) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering left Subpatterns." << endl;

  // Right Subpatterns
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 4, 29 );
  trace.info() << "sub(4,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 2 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 5, 29 );
  trace.info() << "sub(5,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 2 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 16, 29 );
  trace.info() << "sub(16,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 17, 29 );
  trace.info() << "sub(17,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 18, 29 );
  trace.info() << "sub(18,29) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering right Subpatterns." << endl;

  // Middle Subpatterns
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 1, 27 );
  trace.info() << "sub(1,27) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 5, 24 );
  trace.info() << "sub(5,24) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 4, 17 );
  trace.info() << "sub(4,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 5, 17 );
  trace.info() << "sub(5,17) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == SB::fraction( 5, 7 ) && np == 1 ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 7, 16 );
  trace.info() << "sub(5,16) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 1, 4 );
  trace.info() << "sub(1,4) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  mod =  pat_even.getGreatestIncludedSubpattern( sp, np, start,
                                                 18, 25 );
  trace.info() << "sub(18,20) = " << sp << " " << sp.rE() << "^" << np << endl;
  ++nb, nbok += sp.slope() == Fraction() ? 1 : 0;
  trace.info() << "(" << nbok << "/" << nb << ") covering middle Subpatterns." << endl;
  trace.endBlock();

  trace.info() << "Odd pattern " << pat_odd << endl;
  trace.info() << " U(0)=" << pat_odd.U( 0 )
               << " L(0)=" << pat_odd.L( 0 )
               << " U(1)=" << pat_odd.U( 1 )
               << " L(1)=" << pat_odd.L( 1 ) << endl;

  trace.info() << "Even pattern " << pat_even << endl;
  trace.info() << " U(0)=" << pat_even.U( 0 )
               << " L(0)=" << pat_even.L( 0 )
               << " U(1)=" << pat_even.U( 1 )
               << " L(1)=" << pat_even.L( 1 ) << endl;
                                        
  return nbok == nb;
}

/**
 * Example of a test. To be completed.
 *
 */
bool testSternBrocot()
{
  unsigned int nbtests = 100;
  unsigned int nbok = 0;
  unsigned int nb = 0;
  typedef BigInteger Integer;
  typedef SternBrocot<Integer,int32_t> SB;
  typedef typename SB::Fraction Fraction;
  trace.beginBlock ( "Testing block: init fractions." );
  for ( unsigned int i = 0; i < nbtests; ++i )
    {
      nbok += testInitFraction<SB>() ? 1 : 0;
      nb++;
    }
  trace.info() << "(" << nbok << "/" << nb << ") init fractions." << endl;
  trace.endBlock();

  trace.beginBlock ( "Testing block: number of fractions." );
  trace.info() << "- nbFractions = " << SB::nbFractions << endl;
  trace.endBlock();

  ++nb, nbok += testPattern<SB>() ? 1 : 0;
  return nbok == nb;
}

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SternBrocot" );
  bool res = testSternBrocot();
  //&& testPattern(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
