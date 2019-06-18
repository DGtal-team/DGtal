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
 * @file testParametricCurveDigitization.cpp
 * @brief Tests for the parametric curves and their digitization.
 * @ingroup Tests
 * @author Kacper Pluta (\c kacper.pluta@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2018/08/14
 *
 * Functions for testing class Digitization of parametric curves.
 *
 * This file is part of the DGtal library.
 *
 * @see UglyNaiveParametricCurveDigitizer3D.h
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtalCatch.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/geometry/curves/parametric/EllipticHelix.h"
#include "DGtal/geometry/curves/parametric/DecoratorParametricCurveTransformation.h"
#include "DGtal/geometry/curves/parametric/Knot_3_1.h"
#include "DGtal/geometry/curves/parametric/Knot_3_2.h"
#include "DGtal/geometry/curves/parametric/Knot_4_1.h"
#include "DGtal/geometry/curves/parametric/Knot_4_3.h"
#include "DGtal/geometry/curves/parametric/Knot_5_1.h"
#include "DGtal/geometry/curves/parametric/Knot_5_2.h"
#include "DGtal/geometry/curves/parametric/Knot_6_2.h"
#include "DGtal/geometry/curves/parametric/Knot_7_4.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"
#include "DGtal/images/RigidTransformation3D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;
using namespace functors;

TEST_CASE( "Elliptic Helix test" )
{
    typedef EllipticHelix < Space > MyHelix;
    typedef NaiveParametricCurveDigitizer3D < MyHelix >  DigitizerHelix;
    typedef NaiveParametricCurveDigitizer3D < MyHelix >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyHelix >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyHelix helix( 15, 10, 1 );
    DigitizerHelix digitize;
    digitize.init ( M_PI / 2., MyHelix::getPeriod() + M_PI / 2., 0.001 );
    digitize.attach ( &helix );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }

    SECTION("Inverses")
    {
        helix.f ( digitalCurve[0] );
        helix.g ( digitalCurve[0] );
        helix.h ( digitalCurve[0] );
    }

}


TEST_CASE( "Parametric Curve Decorator test" )
{
    typedef EllipticHelix < Space > MyHelix;
    typedef ForwardRigidTransformation3D < Space, RealPoint, RealPoint, Identity > ForwardTrans;
    typedef DecoratorParametricCurveTransformation < MyHelix, ForwardTrans > MyRotatedCurve;
    typedef NaiveParametricCurveDigitizer3D < MyRotatedCurve >  Digitizer;
    typedef typename NaiveParametricCurveDigitizer3D < MyRotatedCurve >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyRotatedCurve >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyHelix helix( 30, 20, 1 );

    double angle = M_PI/3.;
    RealVector axis ( 1., 0., 1. );
    ForwardTrans trans ( RealPoint ( 0, 0, 0 ), axis, angle, RealVector ( 0,0,0 ) );
    MyRotatedCurve rotCurve ( helix, trans );

    Digitizer digitize;
    digitize.attach ( &rotCurve );
    digitize.init ( 0, MyHelix::getPeriod(), 0.001 );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 3_1 test" )
{
    typedef Knot_3_1 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 3_2 test" )
{
    typedef Knot_3_2 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 4_1 test" )
{
    typedef Knot_4_1 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 4_3 test" )
{
    typedef Knot_4_3 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 5_1 test" )
{
    typedef Knot_5_1 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 5_2 test" )
{
    typedef Knot_5_2 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 6_2 test" )
{
    typedef Knot_6_2 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}


TEST_CASE( "Knot 7_4 test" )
{
    typedef Knot_7_4 < Space > MyKnot;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
    typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

    MyDigitalCurve digitalCurve;
    MyMetaData metaData;
    MyKnot knot( 10, 10, 10 );
    Digitizer digitize;
    digitize.init ( -3., 3, 0.0001 );
    digitize.attach ( &knot );
    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ) );

    SECTION("Data")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
    }

    digitalCurve.clear ( );

    digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

    SECTION("Size Comparisons")
    {
        REQUIRE( digitalCurve.size ( ) > 0 );
        REQUIRE( metaData.size ( ) > 0 );
        REQUIRE( metaData.size ( ) == digitalCurve.size ( ) );
    }
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
