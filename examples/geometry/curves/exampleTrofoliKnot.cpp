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
* @file exampleTrofoliKnot.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2018/08/03
 *
 * An example file for ParametriCurveDigitizer3D.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iterator>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/parametric/Knot_3_1.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"

#ifdef WITH_VISU3D_QGLVIEWER
#include "DGtal/io/viewers/PolyscopeViewer.h"
#endif

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;
using namespace functors;

template <typename T>
inline
unsigned char findMainAxis ( const T & curve, const long double & t )
{
 RealPoint value;
 value[0] = std::abs ( curve.xp ( t )[0] );
 value[1] = std::abs ( curve.xp ( t )[1] );
 value[2] = std::abs ( curve.xp ( t )[2] );

 if ( value[0] >= value[1] && value[0] >= value[2] )
  return 0;
 else if ( value[1] >= value[0] && value[1] >= value[2] )
  return 1;
 else
  return 2;
}

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
 typedef Knot_3_1< Space > MyKnot;
 typedef NaiveParametricCurveDigitizer3D < MyKnot >  Digitizer;
 typedef NaiveParametricCurveDigitizer3D < MyKnot >::DigitalCurve MyDigitalCurve;
 typedef NaiveParametricCurveDigitizer3D < MyKnot >::MetaData MyMetaData;

 trace.info() << "exampleParamCurve3dDigitization" << endl;

 PolyscopeViewer<> viewer;

 MyDigitalCurve digitalCurve;
 MyMetaData metaData;
 MyKnot knot ( 10, 10, 10 );
 Digitizer digitize;
 digitize.attach ( &knot );
 digitize.init ( -2.1, 2.1, 0.0001 );
 digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );

 trace.info() << "Number of points: " << digitalCurve.size () << " number of metadata: " << metaData.size () << endl;

 for ( unsigned int i = 0; i < digitalCurve.size ( ); i++ )
 {
  if ( findMainAxis ( knot, metaData.at ( i ).first ) == 0 )
   viewer.drawColor ( Color ( 255, 0, 0, 128 ) );
  if ( findMainAxis ( knot, metaData.at ( i ).first ) == 1 )
   viewer.drawColor ( Color ( 0, 255, 0, 128 ) );
  if ( findMainAxis ( knot, metaData.at ( i ).first ) == 2 )
   viewer.drawColor ( Color ( 0, 0, 255, 128 ) );
  viewer << digitalCurve.at ( i );
 }

 viewer.show();
 return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
