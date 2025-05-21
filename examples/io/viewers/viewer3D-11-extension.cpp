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
 * @file io/viewers/viewer3D-11-extension.cpp
 * @ingroup examples/3dViewer
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2017/03/05
 *
 * Simple example of class PolyscopeViewer.
 *
 * This file is part of the DGtal library.
 */

/**
 * Example of extension of PolyscopeViewer interface by deriving the class
 * PolyscopeViewer<>::Callback.  Here we have added a callback to the
 * "Shift+R" keypressed event, which adds a point randomly in the domain.
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

///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

//! [viewer3D-extension-derivation]
struct RandomPointExtension : public PolyscopeViewer<>::Callback {
  void OnUI() {
    static int count = 16;
    // Depending on the viewer, this may change
    ImGui::SliderInt("Number of points to draw", &count, 0, 32);
    if (ImGui::Button("Create points")) {
      for (int i = 0; i < count; ++i) {
        // Rand values from -10 to 10
        // Viewer is pointer to the viewer this extension is attached to
        *viewer << Point(rand() % 21 - 10, rand() % 21 - 10, rand() % 21 - 10);
      }
      // Ask viewer to render all data that were added
      viewer->renderNewData();
    }
  }
};
//! [viewer3D-extension-derivation]

int main( int argc, char ** argv )
{
  Point p1( 0, 0, 0 );
  Point p2( 5, 5, 5 );
  Point p3( 2, 3, 4 );

  Domain domain( p1, p2 );

  KSpace K;
  K.init( p1, p2, true );

  PolyscopeViewer viewer(K);
  //! [viewer3D-extension-set-extension]
  viewer.setCallback( new RandomPointExtension );
  //! [viewer3D-extension-set-extension]
  viewer << domain;
  viewer << p1 << p2 << p3;

  viewer.show();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
