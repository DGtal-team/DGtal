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
 * @file io/viewers/viewer3D-1-points.cpp
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
 * Example of digital point visualization  with PolyscopeViewer.
 * @see DGtalGLV_Viewer3D
 * \example io/viewers/viewer3D-1-points.cpp
 * \image html simple3dVisu1.png "Digital point visualization  with PolyscopeViewer."
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/Display3D.h"
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

  Point p1( 0, 0, 0 );
  Point p2( 5, 5 ,5 );
  Point p3( 2, 3, 4 );
  Domain domain( p1, p2 );
  
  // Drawing can happen with draw function
  v.draw(p1);
  // Or stream operators that can be chained
  v << p2 << p3;
  // Draw operator allows to retrieve the name (id) of an object 
  std::string name = v.draw(domain);
  trace.info() << name << std::endl;

  v.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
