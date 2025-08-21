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
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/03
 *
 * Functions for testing class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"

#include "DGtal/io/Color.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class PolyscopeViewer.
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main()
{
  KSpace k;
  k.init(Point(2,2,2), Point(4,4,4), true);

  PolyscopeViewer<Space,KSpace> viewer(k);
  trace.beginBlock ( "Testing class for  PolyscopeViewer" );

  Point p1( 0, 0, 0 );
  Point p2( 0, 1 , 0);
  Point p3( 1, 1, 0);
  Point p4(1, 0, 0 );
  Point p5( 2, 0 , 0);
  Point p6( 2, 1, 0);
  RealVector n(1,1,1);
  RealVector n2(0,1,1);

  // Either is possible
  viewer.drawQuad(p1, p2, p3, p4);
  viewer.drawQuad(p4, p5, p6, p3);

  Cell surfel = k.uCell( Point( 2,3,3) );
  SCell surfel2 = k.sCell( Point( 6,3,3), KSpace::POS);
  SCell surfel3 = k.sCell( Point( 8,3,3), KSpace::NEG  );

  viewer << surfel << surfel2 << surfel3;

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
