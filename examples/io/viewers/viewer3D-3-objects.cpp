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
 * @file io/viewers/viewer3D-3-objects.cpp
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/19/03
 *
 * Simple example of class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

/**
 * Example of 6-18 digital Adjacencies visualization  with PolyscopeViewer.
 * @see \ref DGtalGLV_ModeEx
 * \example io/viewers/viewer3D-3-objects.cpp
 * \image html visu6-18Adj.png " 6-18 digital Adjacencies visualization  with PolyscopeViewer."
 */


///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;


///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main()
{
  PolyscopeViewer v;
  
  // Instructs the viewer to draw adjacencies relation whenever possible
  v.drawAdjacencies(true /* false */);

  Point p1( 0, 0, 0 );
  Point p2( 10, 10 , 10 );
  Domain domain( p1, p2 );

  DigitalSet shape_set( domain );
  Shapes<Domain>::addNorm1Ball( shape_set, Point( 5, 5, 5 ), 2 );
  Shapes<Domain>::addNorm2Ball( shape_set, Point( 3, 3, 3 ), 2 );

  v << shape_set;

  Object6_18 shape1( dt6_18, shape_set );
  Object18_6 shape2( dt18_6, shape_set );
  
  // Draws both the object the adjacencies
  v << shape1;
  v << shape2;

  v.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
