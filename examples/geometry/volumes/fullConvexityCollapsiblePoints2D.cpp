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
 * @file geometry/volumes/fullConvexityCollapsiblePoints2D.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/22
 *
 * An example file named fullConvexityCollapsiblePoints2D.
 *
 * This file is part of the DGtal library.
 */

/**
 * Displays fully convex collapsible points in the given image. All
 * these points can be safely flipped without changing the topology of
 * the image.
 *
 * @see \ref moduleDigitalConvexityApplications
 *
 * \example geometry/volumes/fullConvexityCollapsiblePoints2D.cpp
 */
///////////////////////////////////////////////////////////////////////////////
#include <vector>
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
#include "DGtal/geometry/volumes/NeighborhoodConvexityAnalyzer.h"
#include "DGtal/helpers/Shortcuts.h"
#include "ConfigExamples.h"

// Using standard 2D digital space.
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;

typedef DigitalConvexity< KSpace > DConv;
typedef Shortcuts<KSpace> SH2;
typedef NeighborhoodConvexityAnalyzer<KSpace,1> NCA1;

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  double noise = argc > 1 ? atof( argv[ 1 ] ) : 0.1;
  auto params  = SH2::defaultParameters();
  params( "noise", noise )( "thresholdMin", 128 );
  auto g_image = SH2::makeGrayScaleImage( examplesPath + "samples/contourS.pgm" );
  auto b_image = SH2::makeBinaryImage   ( g_image, params );
  Board2D board;
  Domain image_domain = b_image->domain();
  NCA1 nca1( image_domain.lowerBound(), image_domain.upperBound() );
  for ( auto p : image_domain )
    {
      nca1.setCenter( p, (*b_image) );
      bool simple = nca1.isFullyConvexCollapsible();
      if ( (*b_image)( p ) )
        board << CustomStyle( p.className(),
                              simple
                              ? new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 10, 255, 10 ) )
                              : new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 255, 10, 10 ) ) );
      else
        board << CustomStyle( p.className(),
                              simple
                              ? new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 180, 255, 180 ) )
                              : new CustomColors( Color( 0, 0, 0 ),
                                                  Color( 255, 180, 180 ) ) );
      board << p;
    }
  board.saveEPS( "contour-fcvx-collapsible.eps" );
  return 0;
}
