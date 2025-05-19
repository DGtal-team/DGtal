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
* @file exampleParamCurve3dDigitization.cpp
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

//! [DigiHelixHeader]
#include "DGtal/geometry/curves/parametric/EllipticHelix.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"
//! [DigiHelixHeader]

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
//! [DigiHelixConstr]
 typedef EllipticHelix < Space > MyHelix;
 typedef NaiveParametricCurveDigitizer3D < MyHelix >  DigitizerHelix;
 typedef NaiveParametricCurveDigitizer3D < MyHelix >::DigitalCurve MyDigitalCurve;
 typedef NaiveParametricCurveDigitizer3D < MyHelix >::MetaData MyMetaData;
//! [DigiHelixConstr]
 trace.info() << "exampleParamCurve3dDigitization" << endl;

 PolyscopeViewer<> viewer;

//! [DigiHelixInit]
 MyDigitalCurve digitalCurve;
 MyMetaData metaData;
 MyHelix helix( 15, 10, 1 );
 DigitizerHelix digitize;
 digitize.init ( M_PI / 2., ( MyHelix::getPeriod() * 10. ) + M_PI / 2., 0.0001 );
 digitize.attach ( &helix );
//! [DigiHelixInit]

//! [DigiHelixComp]
 digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );
//! [DigiHelixComp]

 trace.info() << "Number of points: " << digitalCurve.size () << " number of metadata: " << metaData.size () << endl;

//! [DigiHelixMetadata]
 for ( unsigned int i = 0; i < digitalCurve.size ( ); i++ )
 {
  if ( findMainAxis ( helix, metaData.at ( i ).first ) == 0 )
   viewer <<  Color ( 255, 0, 0, 128 );
  if ( findMainAxis ( helix, metaData.at ( i ).first ) == 1 )
   viewer << Color ( 0, 255, 0, 128 );
  if ( findMainAxis ( helix, metaData.at ( i ).first ) == 2 )
   viewer << Color ( 0, 0, 255, 128 );
  viewer << digitalCurve.at ( i );
 }
//! [DigiHelixMetadata]

 viewer.show();
 return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
