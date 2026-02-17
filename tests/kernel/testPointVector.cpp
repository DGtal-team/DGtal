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
 * @file
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2015/06/06
 *
 * This file is part of the DGtal library
 */

/**
 * Description of testPointVector-catch' <p>
 * Aim: simple test of \ref PointVector with Catch unit test framework.
 */
#include <cstdio>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <type_traits>
#include <functional>

#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"

#include "DGtalCatch.h"

using namespace DGtal;
using namespace std;

// Compare the value of the two parameters and also their component type.
#define COMPARE_VALUE_AND_TYPE(expr, check) \
  REQUIRE( (expr) == (check) ); \
  REQUIRE( ( std::is_same<decltype(expr)::Component, decltype(check)::Component>::value ) );

TEST_CASE( "2D Point Vector Unit tests" )
{
  using Real = double;
  using Integer = DGtal::int32_t;

  typedef PointVector<2, Integer> Point2D;
  typedef PointVector<2, Real> RealPoint2D;
  typedef PointVector<3, Integer> Point3D;
  typedef PointVector<3, Real> RealPoint3D;

  Integer t1[] = {1,2};
  Integer t2[] = {5,4};
  Real t3[] = {1.5,2.5};
  Real t4[] = {5.5,4.5};

  Point2D p1( t1 );
  Point2D p2( t2 );
  RealPoint2D p3(t3);
  RealPoint2D p4(t4);

  Point3D p1_3d( p1[0], p1[1] );
  Point3D p2_3d( p2[0], p2[1] );
  RealPoint3D p3_3d( p3[0], p3[1] );
  RealPoint3D p4_3d( p4[0], p4[1] );

  SECTION("Cross products with integers")
    {
      COMPARE_VALUE_AND_TYPE( p1.crossProduct(p2), p1_3d.crossProduct(p2_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p1, p2), crossProduct(p1_3d, p2_3d) );
      COMPARE_VALUE_AND_TYPE( p2.crossProduct(p1), p2_3d.crossProduct(p1_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p2, p1), crossProduct(p2_3d, p1_3d) );
    }

  SECTION("Cross products with reals")
    {
      COMPARE_VALUE_AND_TYPE( p3.crossProduct(p4), p3_3d.crossProduct(p4_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p3, p4), crossProduct(p3_3d, p4_3d) );
      COMPARE_VALUE_AND_TYPE( p4.crossProduct(p3), p4_3d.crossProduct(p3_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p4, p3), crossProduct(p4_3d, p3_3d) );
    }

  SECTION("Cross products with mixed integers/reals")
    {
      COMPARE_VALUE_AND_TYPE( p1.crossProduct(p3), p1_3d.crossProduct(p3_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p1, p3), crossProduct(p1_3d, p3_3d) );
      COMPARE_VALUE_AND_TYPE( p3.crossProduct(p1), p3_3d.crossProduct(p1_3d) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p3, p1), crossProduct(p3_3d, p1_3d) );
    }
  SECTION("Access data() of internal container")
    {
      const auto d = p1_3d.data();
      CHECK(d[0] == p1[0]);
      CHECK(d[1] == p1[1]);
    }
}


TEST_CASE( "3D Point Vector Unit tests" )
{
  using Real = double;
  using Integer = DGtal::int32_t;

  typedef PointVector<3, Integer> Point;
  typedef PointVector<3, Real> RealPoint;

  Integer t1[] = {1,2,3};
  Integer t2[] = {5,4,3};
  Real t3[] = {1.5,2.5,3.5};
  Real t4[] = {5.5,4.5,3.5};

  Point p1( t1 );
  Point p2( t2 );
  RealPoint p3(t3);
  RealPoint p4(t4);

  SECTION("Cross products with integers")
    {
      COMPARE_VALUE_AND_TYPE( p1.crossProduct(p2), Point(-6, 12, -6) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p1, p2), Point(-6, 12, -6) );
      COMPARE_VALUE_AND_TYPE( p2.crossProduct(p1), Point(6, -12, 6) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p2, p1), Point(6, -12, 6) );
    }

  SECTION("Cross products with reals")
    {
      COMPARE_VALUE_AND_TYPE( p3.crossProduct(p4), RealPoint(-7., 14., -7.) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p3, p4), RealPoint(-7., 14., -7.) );
      COMPARE_VALUE_AND_TYPE( p4.crossProduct(p3), RealPoint(7., -14., 7.) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p4, p3), RealPoint(7., -14., 7.) );
    }

  SECTION("Cross products with mixed integers/reals")
    {
      COMPARE_VALUE_AND_TYPE( p1.crossProduct(p3), RealPoint(-0.5, 1., -0.5) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p1, p3), RealPoint(-0.5, 1., -0.5) );
      COMPARE_VALUE_AND_TYPE( p3.crossProduct(p1), RealPoint(0.5, -1., 0.5) );
      COMPARE_VALUE_AND_TYPE( crossProduct(p3, p1), RealPoint(0.5, -1., 0.5) );
    }
}

TEST_CASE( "4D Point Vector Unit tests" )
{
  using Real = double;
  using Integer = DGtal::int32_t;

  typedef PointVector<4, Integer> Point;
  typedef PointVector<4, Real> RealPoint;

  const Real pi = std::acos(Real(-1));

  Integer t1[] = {1,2,3,4};
  Integer t2[] = {5,4,3,2};
  Real t3[] = {1.0,-1.0,2.0,-2.0};
  Real t4[] = {5.5,-4.5,3.5,2.5};

  Point p1( t1 );
  Point p1bis( t1 );
  Point p2( t2 );
  RealPoint p3(t3);
  RealPoint p4(t4);
  RealPoint p1r(p1);
  RealPoint p2r(p2);

  SECTION("Construction")
    {
      REQUIRE( p1 == Point( {1, 2, 3, 4} ) );
      REQUIRE( p1r == RealPoint( {1., 2., 3., 4.} ) );

      REQUIRE( p1 == Point( p1r ) );
      REQUIRE( p1 == Point( RealPoint( {0.5, 2.1, 3.1, 3.9} ), DGtal::functors::Round<>() ) );
      REQUIRE( p1 == Point( Point(0, 0, 1, 3), Point(1, 2, 2, 1), std::plus<Integer>() ) );
    }

  SECTION("Assignments")
    {
      Point dummy1;
      dummy1 = p1;
      REQUIRE( p1 == dummy1 );

      Point dummy2(1, 3, 3, 5);
      dummy2.partialCopy( Point(0, 2, 0, 4),  {1, 3} );
      REQUIRE( p1 == dummy2 );

      Point dummy3(2, 2, 1, 4);
      dummy3.partialCopyInv( Point(1, 0, 3, 0),  {1, 3} );
      REQUIRE( p1 == dummy3 );

      RealPoint dummy1r;
      dummy1r = p1;
      REQUIRE( p1r == dummy1r );

      RealPoint dummy2r(1, 3, 3, 5);
      dummy2r.partialCopy( Point(0, 2, 0, 4),  {1, 3} );
      REQUIRE( p1r == dummy2r );

      RealPoint dummy3r(2, 2, 1, 4);
      dummy3r.partialCopyInv( Point(1, 0, 3, 0),  {1, 3} );
      REQUIRE( p1r == dummy3r );

      Point dummy4(1, 3, 3, 5);
      dummy4.partialCopy( RealPoint(0, 1.5, 0, 4.1),  {1, 3}, DGtal::functors::Round<>() );
      REQUIRE( p1 == dummy4 );

      Point dummy5(2, 2, 1, 4);
      dummy5.partialCopyInv( RealPoint(1.1, 0, 2.5, 0),  {1, 3}, DGtal::functors::Round<>() );
      REQUIRE( p1 == dummy5 );
    }

  SECTION("Comparisons")
    {
      // Partial equality
      REQUIRE( p1.partialEqual( RealPoint(0, 2, 0, 4), {1, 3} ) );
      REQUIRE( ! p1.partialEqual( RealPoint(0, 1, 0, 4), {1, 3} ) );
      REQUIRE( p1.partialEqualInv( RealPoint(1, 0, 3, 0), {1, 3} ) );
      REQUIRE( ! p1.partialEqualInv( RealPoint(1, 0, 2, 0), {1, 3} ) );

      // Two equal points of same type
      REQUIRE( p1 == p1bis );
      REQUIRE( p1bis == p1 );
      REQUIRE( ! (p1 != p1bis) );
      REQUIRE( ! (p1bis != p1) );
      REQUIRE( p1 <= p1bis );
      REQUIRE( p1 >= p1bis );
      REQUIRE( p1bis >= p1 );
      REQUIRE( p1bis <= p1 );
      REQUIRE( ! (p1 < p1bis) );
      REQUIRE( ! (p1 > p1bis) );
      REQUIRE( ! (p1bis > p1) );
      REQUIRE( ! (p1bis < p1) );

      // Two equal points of different types
      REQUIRE( p1 == p1r );
      REQUIRE( p1r == p1 );
      REQUIRE( ! (p1 != p1r) );
      REQUIRE( ! (p1r != p1) );
      REQUIRE( p1 <= p1r );
      REQUIRE( p1 >= p1r );
      REQUIRE( p1r >= p1 );
      REQUIRE( p1r <= p1 );
      REQUIRE( ! (p1 < p1r) );
      REQUIRE( ! (p1 > p1r) );
      REQUIRE( ! (p1r > p1) );
      REQUIRE( ! (p1r < p1) );

      // Two ordered points of same type
      REQUIRE( ! (p1 == p2) );
      REQUIRE( ! (p2 == p1) );
      REQUIRE( p1 != p2 );
      REQUIRE( p2 != p1 );
      REQUIRE( p1 <= p2 );
      REQUIRE( ! (p1 >= p2) );
      REQUIRE( p2 >= p1 );
      REQUIRE( ! (p2 <= p1) );
      REQUIRE( p1 < p2 );
      REQUIRE( ! (p1 > p2) );
      REQUIRE( p2 > p1 );
      REQUIRE( ! (p2 < p1) );

      // Two ordered points of different types
      REQUIRE( ! (p1 == p2r) );
      REQUIRE( ! (p2r == p1) );
      REQUIRE( p1 != p2r );
      REQUIRE( p2r != p1 );
      REQUIRE( p1 <= p2r );
      REQUIRE( ! (p1 >= p2r) );
      REQUIRE( p2r >= p1 );
      REQUIRE( ! (p2r <= p1) );
      REQUIRE( p1 < p2r );
      REQUIRE( ! (p1 > p2r) );
      REQUIRE( p2r > p1 );
      REQUIRE( ! (p2r < p1) );
    }

  SECTION("Min/Max of vector components")
    {
      REQUIRE( p3.max() == 2.0 );
      REQUIRE( p3.min() == -2.0 );
      REQUIRE( *p3.maxElement() == 2.0 );
      REQUIRE( *p3.minElement() == -2.0 );
    }

  Point  aPoint;
  aPoint[ 3 ] =  0;
  aPoint[ 2 ] =  2;
  aPoint[ 1 ] = -1;
  aPoint[ 0 ] =  3;

  SECTION("Testing norms")
    {
      RealPoint normalized = aPoint.getNormalized();
      CAPTURE( normalized );
      REQUIRE( aPoint.norm ( Point::L_1 ) == 6 );
      REQUIRE( aPoint.norm ( Point::L_infty ) == 3 );
      REQUIRE( aPoint.squaredNorm() ==  Approx(aPoint.norm()*aPoint.norm()) );
      REQUIRE( normalized[0] == Approx( 0.801784) );
      REQUIRE( normalized[1] == Approx( -0.267261) );
      REQUIRE( normalized[2] == Approx( 0.534522) );
      REQUIRE( normalized[3] == Approx( 0.0) );
    }

  SECTION("PointVector Iterator")
    {
      PointVector<25,int> aPoint25;
      for (unsigned int i=0;i<25;++i)
        aPoint25[i] = i;

      int sum = 0;
      for (PointVector<25,int>::ConstIterator it = aPoint25.begin() ;  it != aPoint25.end(); ++it)
        sum += (*it);

      CAPTURE(aPoint25);
      CAPTURE(sum);
      REQUIRE( sum == 300 );
    }

  SECTION("Arithmetical operators with integers")
    {
      COMPARE_VALUE_AND_TYPE( p1 + p2, Point(6,6,6,6) );
      COMPARE_VALUE_AND_TYPE( p1 - p2, Point(-4,-2,0,2) );
      COMPARE_VALUE_AND_TYPE( p1 * p2, Point(5,8,9,8) );
      COMPARE_VALUE_AND_TYPE( p2 / p1, Point(5,2,1,0) );

      COMPARE_VALUE_AND_TYPE( p1 + 2, Point(3,4,5,6) );
      COMPARE_VALUE_AND_TYPE( 2 + p1, Point(3,4,5,6) );
      COMPARE_VALUE_AND_TYPE( p1 - 2, Point(-1,0,1,2) );
      COMPARE_VALUE_AND_TYPE( 2 - p1, Point(1,0,-1,-2) );
      COMPARE_VALUE_AND_TYPE( p1 * 2, Point(2,4,6,8) );
      COMPARE_VALUE_AND_TYPE( 2 * p1, Point(2,4,6,8) );
      COMPARE_VALUE_AND_TYPE( p1 / 2, Point(0,1,1,2) );
      COMPARE_VALUE_AND_TYPE( 2 / p1, Point(2,1,0,0) );

      COMPARE_VALUE_AND_TYPE( -p1, Point(-1,-2,-3,-4) );

      p1 *= 2; COMPARE_VALUE_AND_TYPE( p1, Point(2,4,6,8) );
      p1 += 2; COMPARE_VALUE_AND_TYPE( p1, Point(4,6,8,10) );
      p1 -= 2; COMPARE_VALUE_AND_TYPE( p1, Point(2,4,6,8) );
      p1 /= 2; COMPARE_VALUE_AND_TYPE( p1, Point(1,2,3,4) );

      p1 *= p2; COMPARE_VALUE_AND_TYPE( p1, Point(5,8,9,8) );
      p1 += p2; COMPARE_VALUE_AND_TYPE( p1, Point(10,12,12,10) );
      p1 -= p2; COMPARE_VALUE_AND_TYPE( p1, Point(5,8,9,8) );
      p1 /= p2; COMPARE_VALUE_AND_TYPE( p1, Point(1,2,3,4) );
    }

  SECTION("Other operators with integers")
    {
      COMPARE_VALUE_AND_TYPE( p1.inf(p2), Point(1,2,3,2) );
      COMPARE_VALUE_AND_TYPE( p1.sup(p2), Point(5,4,3,4) );
      COMPARE_VALUE_AND_TYPE( inf(p1, p2), Point(1,2,3,2) );
      COMPARE_VALUE_AND_TYPE( sup(p1, p2), Point(5,4,3,4) );

      REQUIRE( p1.dot(p2) == 30 );
      REQUIRE( dotProduct(p1, p2) == 30 );

      REQUIRE( p1.cosineSimilarity(p1) == Approx(0.).margin(0.000001));
      REQUIRE( p1.cosineSimilarity(-p1) == Approx(pi).margin(0.000001));
      REQUIRE( p1.cosineSimilarity( Point(-2,1,-4,3) ) == Approx(pi/2).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, p1) == Approx(0.).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, -p1) == Approx(pi).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, Point(-2,1,-4,3)) == Approx(pi/2).margin(0.000001) );

      REQUIRE( p1.isLower(p2) == false );
      REQUIRE( isLower(p1, p2) == false );
      REQUIRE( p2.isUpper(p1) == false );
      REQUIRE( isUpper(p2, p1) == false );
      p1[3] = 2;
      REQUIRE( p1.isLower(p2) ==  true );
      REQUIRE( isLower(p1, p2) == true );
      REQUIRE( p2.isUpper(p1) == true );
      REQUIRE( isUpper(p2, p1) == true );
    }

  SECTION("Arithmetical Operators with reals")
    {
      COMPARE_VALUE_AND_TYPE( p3 + p4, RealPoint(6.5,-5.5,5.5,0.5) );
      COMPARE_VALUE_AND_TYPE( p3 - p4, RealPoint(-4.5,3.5,-1.5,-4.5) );
      COMPARE_VALUE_AND_TYPE( p3 * p4, RealPoint(5.5,4.5,7.0,-5.0) );
      COMPARE_VALUE_AND_TYPE( p4 / p3, RealPoint(5.5,4.5,1.75,-1.25) );

      COMPARE_VALUE_AND_TYPE( p3 + 2., RealPoint(3.,1.,4.,0.) );
      COMPARE_VALUE_AND_TYPE( 2. + p3, RealPoint(3.,1.,4.,0.) );
      COMPARE_VALUE_AND_TYPE( p3 - 2., RealPoint(-1.,-3.,0.,-4.) );
      COMPARE_VALUE_AND_TYPE( 2. - p3, RealPoint(1.,3.,0.,4.) );
      COMPARE_VALUE_AND_TYPE( p3 * 2., RealPoint(2.,-2.,4.,-4.) );
      COMPARE_VALUE_AND_TYPE( 2. * p3, RealPoint(2.,-2.,4.,-4.) );
      COMPARE_VALUE_AND_TYPE( p3 / 2., RealPoint(0.5,-0.5,1.,-1.) );
      COMPARE_VALUE_AND_TYPE( 2. / p3, RealPoint(2.,-2.,1.,-1.) );

      COMPARE_VALUE_AND_TYPE( -p3, RealPoint(-1.,1.,-2.,2.) );

      p3 *= 2.5; COMPARE_VALUE_AND_TYPE( p3, RealPoint(2.5, -2.5, 5., -5.) );
      p3 += 2.5; COMPARE_VALUE_AND_TYPE( p3, RealPoint(5., 0., 7.5, -2.5) );
      p3 -= 2.5; COMPARE_VALUE_AND_TYPE( p3, RealPoint(2.5, -2.5, 5., -5.) );
      p3 /= 2.5; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -1., 2., -2.) );

      p3 *= p4; COMPARE_VALUE_AND_TYPE( p3, RealPoint(5.5, 4.5, 7., -5.) );
      p3 += p4; COMPARE_VALUE_AND_TYPE( p3, RealPoint(11, 0., 10.5, -2.5) );
      p3 -= p4; COMPARE_VALUE_AND_TYPE( p3, RealPoint(5.5, 4.5, 7., -5.) );
      p3 /= p4; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -1., 2., -2.) );
    }

  SECTION("Other operators with reals")
    {
      COMPARE_VALUE_AND_TYPE( p3.inf(p4), RealPoint(1.,-4.5,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( p3.sup(p4), RealPoint(5.5,-1.,3.5,2.5) );
      COMPARE_VALUE_AND_TYPE( inf(p3, p4), RealPoint(1.,-4.5,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( sup(p3, p4), RealPoint(5.5,-1.,3.5,2.5) );

      REQUIRE( p3.dot(p4) == 12. );
      REQUIRE( dotProduct(p3, p4) == 12. );

      REQUIRE( p3.cosineSimilarity(p3) == Approx(0.).margin(0.000001) );
      REQUIRE( p3.cosineSimilarity(-p3) == Approx(pi).margin(0.000001) );
      REQUIRE( p3.cosineSimilarity( RealPoint(1.0,1.0,2.0,2.0) ) == Approx(pi/2).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, p3) == Approx(0.).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, -p3) == Approx(pi).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, RealPoint(1.0,1.0,2.0,2.0)) == Approx(pi/2).margin(0.000001) );

      REQUIRE( p3.isLower(p4) == false );
      REQUIRE( isLower(p3, p4) == false );
      REQUIRE( p4.isUpper(p3) == false );
      REQUIRE( isUpper(p4, p3) == false );
      p4[1] = -p4[1];
      REQUIRE( p3.isLower(p4) == true );
      REQUIRE( isLower(p3, p4) == true );
      REQUIRE( p4.isUpper(p3) == true );
      REQUIRE( isUpper(p4, p3) == true );
    }

  SECTION("Arithmetical Operators with mixed integers/reals")
    {
      COMPARE_VALUE_AND_TYPE( p1 + p4, RealPoint(6.5,-2.5,6.5,6.5) );
      COMPARE_VALUE_AND_TYPE( p4 + p1, RealPoint(6.5,-2.5,6.5,6.5) );
      COMPARE_VALUE_AND_TYPE( p1 - p4, RealPoint(-4.5,6.5,-0.5,1.5) );
      COMPARE_VALUE_AND_TYPE( p4 - p1, RealPoint(4.5,-6.5,0.5,-1.5) );
      COMPARE_VALUE_AND_TYPE( p1 * p4, RealPoint(5.5,-9.0,10.5,10.0) );
      COMPARE_VALUE_AND_TYPE( p4 * p1, RealPoint(5.5,-9.0,10.5,10.0) );
      COMPARE_VALUE_AND_TYPE( p1 / p3, RealPoint(1.,-2.,1.5,-2.0) );
      COMPARE_VALUE_AND_TYPE( p3 / p1, RealPoint(1.,-0.5,2./3.,-0.5) );

      COMPARE_VALUE_AND_TYPE( p3 + 2, RealPoint(3.,1.,4.,0.) );
      COMPARE_VALUE_AND_TYPE( 2 + p3, RealPoint(3.,1.,4.,0.) );
      COMPARE_VALUE_AND_TYPE( p3 - 2, RealPoint(-1.,-3.,0.,-4.) );
      COMPARE_VALUE_AND_TYPE( 2 - p3, RealPoint(1.,3.,0.,4.) );
      COMPARE_VALUE_AND_TYPE( p3 * 2, RealPoint(2.,-2.,4.,-4.) );
      COMPARE_VALUE_AND_TYPE( 2 * p3, RealPoint(2.,-2.,4.,-4.) );
      COMPARE_VALUE_AND_TYPE( p3 / 2, RealPoint(0.5,-0.5,1.,-1.) );
      COMPARE_VALUE_AND_TYPE( 2 / p3, RealPoint(2.,-2.,1.,-1.) );

      COMPARE_VALUE_AND_TYPE( p1 + 2.5, RealPoint(3.5,4.5,5.5,6.5) );
      COMPARE_VALUE_AND_TYPE( 2.5 + p1, RealPoint(3.5,4.5,5.5,6.5) );
      COMPARE_VALUE_AND_TYPE( p1 - 2.5, RealPoint(-1.5,-0.5,0.5,1.5) );
      COMPARE_VALUE_AND_TYPE( 2.5 - p1, RealPoint(1.5,0.5,-0.5,-1.5) );
      COMPARE_VALUE_AND_TYPE( p1 * 2.5, RealPoint(2.5,5.,7.5,10.) );
      COMPARE_VALUE_AND_TYPE( 2.5 * p1, RealPoint(2.5,5.,7.5,10.) );
      COMPARE_VALUE_AND_TYPE( p1 / 0.5, RealPoint(2.,4.,6.,8.) );
      COMPARE_VALUE_AND_TYPE( 2. / p1, RealPoint(2.,1.,2./3.,0.5) );

      p3 *= 2; COMPARE_VALUE_AND_TYPE( p3, RealPoint(2, -2, 4., -4.) );
      p3 += 2; COMPARE_VALUE_AND_TYPE( p3, RealPoint(4., 0., 6., -2.) );
      p3 -= 2; COMPARE_VALUE_AND_TYPE( p3, RealPoint(2, -2, 4., -4.) );
      p3 /= 2; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -1., 2., -2.) );

      p3 *= p1; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -2., 6., -8.) );
      p3 += p1; COMPARE_VALUE_AND_TYPE( p3, RealPoint(2., 0., 9., -4.) );
      p3 -= p1; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -2., 6., -8.) );
      p3 /= p1; COMPARE_VALUE_AND_TYPE( p3, RealPoint(1., -1., 2., -2.) );
    }

  SECTION("Other operators with mixed integers/reals")
    {
      COMPARE_VALUE_AND_TYPE( p1.inf(p3), RealPoint(1.,-1.,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( p3.inf(p1), RealPoint(1.,-1.,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( p1.sup(p3), RealPoint(1.,2.,3.,4.) );
      COMPARE_VALUE_AND_TYPE( p3.sup(p1), RealPoint(1.,2.,3.,4.) );
      COMPARE_VALUE_AND_TYPE( inf(p1, p3), RealPoint(1.,-1.,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( inf(p3, p1), RealPoint(1.,-1.,2.,-2.) );
      COMPARE_VALUE_AND_TYPE( sup(p1, p3), RealPoint(1.,2.,3.,4.) );
      COMPARE_VALUE_AND_TYPE( sup(p3, p1), RealPoint(1.,2.,3.,4.) );

      REQUIRE( p4.dot(p1) == 17.0 );
      REQUIRE( dotProduct(p4, p1) == 17.0 );
      REQUIRE( dotProduct(p1, p4) == 17.0 );

      REQUIRE( p1.cosineSimilarity(RealPoint(p1)) == Approx(0.).margin(0.000001) );
      REQUIRE( p1.cosineSimilarity(-RealPoint(p1)) == Approx(pi).margin(0.000001) );
      REQUIRE( p1.cosineSimilarity( RealPoint(-2,1,-4,3) ) == Approx(pi/2).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, RealPoint(p1)) == Approx(0.).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, -RealPoint(p1)) == Approx(pi).margin(0.000001) );
      REQUIRE( cosineSimilarity(p1, RealPoint(-2,1,-4,3)) == Approx(pi/2).margin(0.000001) );

      REQUIRE( p3.cosineSimilarity(Point(1,-1,2,-2)) == Approx(0.).margin(0.000001) );
      REQUIRE( p3.cosineSimilarity(-Point(1,-1,2,-2)) == Approx(pi).margin(0.000001) );
      REQUIRE( p3.cosineSimilarity( Point(1,1,2,2) ) == Approx(pi/2).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, Point(1,-1,2,-2)) == Approx(0.).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, -Point(1,-1,2,-2)) == Approx(pi).margin(0.000001) );
      REQUIRE( cosineSimilarity(p3, Point(1,1,2,2)) == Approx(pi/2).margin(0.000001) );

      REQUIRE( p2.isLower(p4) == false );
      REQUIRE( isLower(p2, p4) == false );
      REQUIRE( p4.isUpper(p2) == false );
      REQUIRE( isUpper(p4, p2) == false );
      p4[1] = -p4[1];
      REQUIRE( p2.isLower(p4) == true );
      REQUIRE( isLower(p2, p4) == true );
      REQUIRE( p4.isUpper(p2) == true );
      REQUIRE( isUpper(p4, p2) == true );
    }

}


TEST_CASE("Benchmarking","[.benchmark]")
{
  using Integer = DGtal::int32_t;
  typedef PointVector<3, Integer> Point;
  Point p1 = {1,2,3,4};
  Point p2 = {3,4,5,6};

  using Real = double;
  typedef PointVector<3, Real> RPoint;
  RPoint rp1 = {1,2,3,4};
  RPoint rp2 = {3,4,5,6};

  CHECK(p1.dot(p2) == 26);
  CHECK(rp1.dot(rp2) == Approx(26));

  BENCHMARK("Dot product int")
  {
    return p1.dot(p2);
  };

  BENCHMARK("Dot product double (with int->double cast)")
  {
    return rp1.dot(p2);
  };

  BENCHMARK("Dot product double")
  {
    return rp1.dot(rp2);
  };

}

/** @ingroup Tests **/
