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
* @file exampleParamCurve3dDigitizationTransformationDecorator.cpp
 * @ingroup Examples
 * @author Kacper Pluta (\c kacper.pluta@@esiee.fr )
 * Laboratoire d'Informatique Gaspard-Monge - LIGM, A3SI, France
 *
 * @date 2018/08/03
 *
 * An example file for DecoratorParametricCurveTransformation.h.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/parametric/EllipticHelix.h"
#include "DGtal/geometry/curves/parametric/NaiveParametricCurveDigitizer3D.h"
//! [DigiRotHelixHeader]
#include "DGtal/geometry/curves/parametric/DecoratorParametricCurveTransformation.h"
#include "DGtal/images/RigidTransformation3D.h"
//! [DigiRotHelixHeader]

#ifdef DGTAL_WITH_POLYSCOPE_VIEWER
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
int main()
{
 //! [DigiRotHelixConstr]
 typedef EllipticHelix < Space > MyHelix;
 typedef ForwardRigidTransformation3D < Space, RealPoint, RealPoint, Identity > ForwardTrans;
 typedef DecoratorParametricCurveTransformation < MyHelix, ForwardTrans > MyRotatedCurve;
 typedef NaiveParametricCurveDigitizer3D < MyRotatedCurve >  Digitizer;
 typedef typename NaiveParametricCurveDigitizer3D < MyRotatedCurve >::DigitalCurve MyDigitalCurve;
 //! [DigiRotHelixConstr]
 typedef NaiveParametricCurveDigitizer3D < MyRotatedCurve >::MetaData MyMetaData;
 trace.info() << "exampleParamCurve3dDigitizationTransformationDecorator" << endl;

 PolyscopeViewer<> viewer;

 MyMetaData metaData;

 //! [DigiRotHelixConstrCurve]
 MyDigitalCurve digitalCurve;
 MyHelix helix( 30, 20, 1 );
 //! [DigiRotHelixConstrCurve]

 //! [DigiRotHelixConstrRot]
 double angle = M_PI/3.;
 RealVector axis ( 1., 0., 1. );
 ForwardTrans trans ( RealPoint ( 0, 0, 0 ), axis, angle, RealVector ( 0, 0, 0 ) );
 MyRotatedCurve rotCurve ( helix, trans );
 //! [DigiRotHelixConstrRot]

 //! [DigiRotHelixDigitize]
 Digitizer digitize;
 digitize.attach ( &rotCurve );
 digitize.init ( 0, MyHelix::getPeriod() * 10., 0.0001 );
 digitize.digitize( back_insert_iterator < MyDigitalCurve> ( digitalCurve ), back_insert_iterator < MyMetaData > ( metaData ) );
 //! [DigiRotHelixDigitize]

 trace.info() << "Number of points: " << digitalCurve.size () << " number of metadata: " << metaData.size () << endl;

 for ( unsigned int i = 0; i < digitalCurve.size ( ); i++ )
 {
  if ( findMainAxis ( rotCurve, metaData.at ( i ).first ) == 0 )
   viewer.drawColor ( Color ( 255, 0, 0, 128 ) );
  if ( findMainAxis ( rotCurve, metaData.at ( i ).first ) == 1 )
   viewer.drawColor ( Color ( 0, 255, 0, 128 ) );
  if ( findMainAxis ( rotCurve, metaData.at ( i ).first ) == 2 )
   viewer.drawColor ( Color ( 0, 0, 255, 128 ) );
  viewer << digitalCurve.at ( i );
 }

 viewer.show();
 return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
