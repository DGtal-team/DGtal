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
 * @author David Coeurjolly
 * @date 2013/09/06
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
// Standard services - public :

int main()
{
  PolyscopeViewer<> viewer;
  trace.beginBlock ( "Testing Polygon 3D display in PolyscopeViewer" );
  std::vector<Z3i::RealPoint> polyg1;

  polyg1.push_back(Z3i::RealPoint(0,0,0));
  polyg1.push_back(Z3i::RealPoint(0,1,0));
  polyg1.push_back(Z3i::RealPoint(1,1,0));

  viewer.drawPolygon(polyg1);

  std::vector<Z3i::RealPoint> polyg2;

  polyg2.push_back(Z3i::RealPoint(0,10,0));
  polyg2.push_back(Z3i::RealPoint(0,11,0));
  polyg2.push_back(Z3i::RealPoint(11,11,0));

  viewer.drawPolygon(polyg2, "hop");

  trace.emphase() << "Passed." << endl;
  trace.endBlock();
  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

