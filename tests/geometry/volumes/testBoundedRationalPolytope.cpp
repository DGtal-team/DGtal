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
 * @file testBoundedRationalPolytope.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/04/28
 *
 * Functions for testing class BoundedRationalPolytope.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <vector>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/geometry/volumes/BoundedRationalPolytope.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class BoundedRationalPolytope.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "BoundedRationalPolytope< Z2 > unit tests", "[rational_polytope][2d]" )
{
  typedef SpaceND<2,int>                   Space;
  typedef Space::Point                     Point;
  typedef Space::Vector                    Vector;
  typedef BoundedRationalPolytope< Space >  Polytope;

  GIVEN( "A triangle P at (0,0), (5/2,0), (0,7/2)" ) {
    Point a( 0, 0 );
    Point b( 5, 0 );
    Point c( 0, 7 );
    Polytope P { Point(2,2), a, b, c };
    THEN( "It contains more than 3 integer points" ) {
      REQUIRE( P.count() > 3 );
    }
    THEN( "It contains more points than its area" ) {
      REQUIRE( P.count() > (5*7/8) );
    }
    THEN( "It satisfies #In(P) <= #Int(P) + #Bd(P)" ) {
      auto     nb = P.count();
      auto nb_int = P.countInterior();
      auto  nb_bd = P.countBoundary();
      CAPTURE( nb );
      CAPTURE( nb_int );
      CAPTURE( nb_bd );
      REQUIRE( nb <= nb_int + nb_bd );
    }
    WHEN( "Cut by some half-space" ) {
      Polytope Q = P;
      Q.cut( Vector( -1, 1 ), 3 );
      THEN( "It contains less points" ) {
	REQUIRE( Q.count() < P.count() );
      }
    }
    WHEN( "Multiplied by 4 as Q = 4 * P, it becomes a lattice polytope" ) {
      Polytope Q = 4 * P;
      THEN( "It satisfies Pick's formula, ie 2*Area(P) = 2*#Int(P) + #Bd(P) - 2" ) {
	Polytope IntQ = Q.interiorPolytope();
	auto   nb_int = IntQ.count();
	auto    nb_bd = Q.count() - IntQ.count();
	auto    area2 = (5*2)*(7*2);
      CAPTURE( Q );
      CAPTURE( nb_int );
      CAPTURE( nb_bd );
      CAPTURE( area2 );
      REQUIRE( area2 == 2*nb_int + nb_bd - 2 );
      }
    }
  }
  GIVEN( "A closed segment S at (4/2,0/2), (-8/2,-4/2)" ) {
    Point a( 4, 0 );
    Point b( -8, -4 );
    Polytope P { Point(2,2), a, b };
    THEN( "Its interior is empty #Int(P) == 0" ) {
      auto nb_int = P.countInterior();
      REQUIRE( nb_int == 0 );
    }
    THEN( "It satisfies #In(P) == #Int(P) + #Bd(P) == #Bd(P) == 3" ) {
      auto     nb = P.count();
      auto nb_int = P.countInterior();
      auto  nb_bd = P.countBoundary();
      CAPTURE( nb );
      CAPTURE( nb_int );
      CAPTURE( nb_bd );
      std::vector<Point> Ppts;
      P.getPoints( Ppts );
      CAPTURE( P );
      CAPTURE( Ppts );
      REQUIRE( nb_bd == 3 );
      REQUIRE( nb == nb_int + nb_bd );
    }
  }
  GIVEN( "A thin triangle P at (4/4,2/4), (2/4,4/4), (9/4,9/4)" ) {
    Point a( 4, 2 );
    Point b( 2, 4 );
    Point c( 9, 9 );
    Polytope P { Point(4,4), a, b, c };
    THEN( "It contains 2 integer points" ) {
      REQUIRE( P.count() == 2 );
    }
    THEN( "Its boundary is empty" ) {
      REQUIRE( P.countBoundary() == 0 );
    }
    WHEN( "Multiplied by 4 as Q = 4 * P, it becomes a lattice polytope" ) {
      Polytope Q = 4 * P;
      THEN( "It satisfies Pick's formula, ie 2*Area(P) = 2*#Int(P) + #Bd(P) - 2" ) {
	Polytope IntQ = Q.interiorPolytope();
	auto   nb_int = IntQ.count();
	auto    nb_bd = Q.count() - IntQ.count();
	auto    area2 = 24;
	CAPTURE( Q );
	CAPTURE( nb_int );
	CAPTURE( nb_bd );
	CAPTURE( area2 );
	REQUIRE( area2 == 2*nb_int + nb_bd - 2 );
      }
    }
    WHEN( "Multiplied by 10/3 as Q = 10/3 * P, it is a rational polytope" ) {
      Polytope Q = Polytope::Rational( 10, 3 ) * P;
      THEN( "It has a denominator 3 * gcd(4,10) == 6" ) {
	REQUIRE( Q.denominator() == 6 );
      }
      THEN( "#( 3P Cap Z2 ) <= #( Q Cap Z2 ) < #( 4P Cap Z2 )" ) {
	Polytope R = 3 * P;
	Polytope S = 4 * P;
	auto   nbQ = Q.count();
	auto   nbR = R.count();
	auto   nbS = S.count();
	CAPTURE( nbR );
	CAPTURE( nbQ );
	CAPTURE( nbS );
	REQUIRE( nbR <= nbQ );
	REQUIRE( nbQ <= nbS );
      }
      THEN( "6/5*Q is a polytope equal to 4*P." ) {
	Polytope R = Polytope::Rational( 6, 5 ) * Q;
	Polytope S = 4 * P;
	std::vector<Point> Rpts, Spts;
	R.getPoints( Rpts );
	S.getPoints( Spts );
	CAPTURE( Rpts );
	CAPTURE( Spts );
	REQUIRE( std::equal( Rpts.cbegin(), Rpts.cend(), Spts.cbegin() ) );
      }
    }
  }
} // SCENARIO( "BoundedRationalPolytope< Z2 > unit tests", "[rational_polytope][2d]" )

SCENARIO( "BoundedRationalPolytope< Z3 > unit tests", "[rational_polytope][3d]" )
{
  typedef SpaceND<3,int>                   Space;
  typedef Space::Point                     Point;
  typedef BoundedRationalPolytope< Space >  Polytope;

  GIVEN( "A tetrahedron P at (0,0,0), (5/2,0,0), (0,7/2,0), (0,0,3/2)" ) {
    Point a( 0, 0, 0 );
    Point b( 5, 0, 0 );
    Point c( 0, 7, 0 );
    Point d( 0, 0, 3 );
    Polytope P { Point(2,2), a, b, c, d };
    THEN( "It contains more than 3 integer points" ) {
      REQUIRE( P.count() > 3 );
    }
    THEN( "It contains more points than its volume" ) {
      REQUIRE( P.count() > (5*7*3/8/6) );
    }
    THEN( "It satisfies #In(P) <= #Int(P) + #Bd(P)" ) {
      auto     nb = P.count();
      auto nb_int = P.countInterior();
      auto  nb_bd = P.countBoundary();
      CAPTURE( nb );
      CAPTURE( nb_int );
      CAPTURE( nb_bd );
      REQUIRE( nb <= nb_int + nb_bd );
    }
    WHEN( "Multiplied by 4 as Q = 2 * P, it becomes a lattice polytope" ) {
      Polytope Q = 2 * P;
      THEN( "Its denominator is 1" ) {
	REQUIRE( Q.denominator() == 1 );
      }
      THEN( "It has more interior and boundary points than P" ) {
	auto     nb = P.count();
	auto nb_int = P.countInterior();
	auto  nb_bd = P.countBoundary();
	Polytope IntQ = Q.interiorPolytope();
	auto  nb_intQ = IntQ.count();
	auto      nbQ = Q.count();
	auto   nb_bdQ = nbQ - nb_intQ;
	CAPTURE( Q );
	REQUIRE( nb_int < nb_intQ );
	REQUIRE( nb_bd  < nb_bdQ  );
	REQUIRE( nb     < nbQ     );
      }
    }
    WHEN( "Considering an increasing series f_i = 3/2, 2, 9/4, 3, 11/3, the number of inside points of f_i * P is increasing" ) {
      Polytope Q1 = Polytope::Rational( 3 , 2 ) * P;
      Polytope Q2 = Polytope::Rational( 2 , 1 ) * P;
      Polytope Q3 = Polytope::Rational( 9 , 4 ) * P;
      Polytope Q4 = Polytope::Rational( 3 , 1 ) * P;
      Polytope Q5 = Polytope::Rational( 11, 3 ) * P;
      auto    nb  = P .count();
      auto    nb1 = Q1.count();
      auto    nb2 = Q2.count();
      auto    nb3 = Q3.count();
      auto    nb4 = Q4.count();
      auto    nb5 = Q5.count();
      REQUIRE( nb  < nb1 );
      REQUIRE( nb1 < nb2 );
      REQUIRE( nb2 < nb3 );
      REQUIRE( nb3 < nb4 );
      REQUIRE( nb4 < nb5 );
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
