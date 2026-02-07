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
 * @file io/viewers/viewer3D-10-interaction.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @author Bertrand Kerautret (\c bertrand.kerautret@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Lorraine, France
 *
 * @date 2014/10/12
 *
 * Simple example of class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

/**
 * Simple selection of a surfel (with shift + left click) with the
 * Polyscope Viewer proposed by DGtal. You may associates \a names
 * (i.e. integers) to surfels or to group of surfels. You may associate
 * reactions or callback functions to \a named graphical objects (surfels
 * in DGtal 0.9).
 *
 * @verbatim
 * $ ./examples/io/viewers/viewer3D-10-interaction
 * @endverbatim
 *
 *
 * @image html viewer3D-10-interaction.png   "Example of PolyscopeViewer interaction with the selection of surfels. "
 *
 * \example io/viewers/viewer3D-10-interaction.cpp
 *
 */


///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/viewers/PolyscopeViewer.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

typedef PolyscopeViewer<Space,KSpace> MyViewer;
typedef MyViewer::Callback Callback;
typedef KSpace::SCell SCell;

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :
//
struct MyCallback : public Callback {
  void OnClick(
      const std::string& name, size_t index,
      const DisplayData<MyViewer::RealPoint>& data,
      void* polyscopeStructure
  ) {
    ((void) index); ((void) data); ((void) polyscopeStructure);
    std::cout << "Item name: " << name << std::endl;
  }
};

int main()
{
  Point p1( 0, 0, 0 );
  Point p2( 5, 5 ,5 );
  Point p3( 2, 3, 4 );

  KSpace K;
  K.init( p1, p2, true );

  Point v1 = Z3i::Point(10, 10,10);
  Point v2 = Z3i::Point(9, 9, 9);
  Point v3 = Z3i::Point(11, 11,11);

  MyViewer viewer( K );
  viewer.setCallback(new MyCallback);
  Z3i::SCell surfel1 = K.sCell( Point( 1, 1, 2 ), KSpace::POS );
  Z3i::SCell surfel2 = K.sCell( Point( 3, 3, 4 ), KSpace::NEG );
  Z3i::SCell surfel3 = K.sCell( Point( 5, 6, 5 ), KSpace::POS );

  viewer.draw(surfel1, "Surfel 1");
  viewer.draw(surfel2, "Surfel 2");
  viewer << Point(0, 0, 1) << Point(1, 1, 2);

  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
