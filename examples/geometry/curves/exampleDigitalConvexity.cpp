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
 * @file geometry/curves/exampleDigitalConvexity.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/02/02
 *
 * @brief An example file for DigitalConvexity in 2D.
 *
 * This file is part of the DGtal library.
 */


/**
   This snippet shows how to identify and display digital fully
   subconvex sets of a grid curve form its tangent bundle.

   @see \ref moduleDigitalConvexity

   \image html grid-curve-dig-convexity.png "Extraction of all subconvex triangles to the digital curve."

   \example geometry/curves/exampleDigitalConvexity.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/FreemanChain.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"

#include "DGtal/io/boards/Board2D.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z2i;


///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  ((void) argc); ((void) argv);
  trace.beginBlock ( "Example for 2d gridcurves" );
  string S = examplesPath + "samples/contourS.fc";

  // domain
  const Point lowerBound( -200, -200 );
  const Point upperBound( 200, 200 );

  fstream inputStream( S.c_str(), ios::in );
  FreemanChain<int> fc(inputStream);
  inputStream.close();
  Curve c;
  c.initFromPointsRange( fc.begin(), fc.end() );
  auto points = c.getPointsRange();
  std::vector<Point> T( points.begin(), points.end() );
  Board2D aBoard;
  aBoard.setUnit(Board2D::UCentimeter);
  DigitalConvexity<KSpace> dconv( lowerBound, upperBound );
  auto c_cover = dconv.makeCellCover( T.begin(), T.end(), 1, 1 );
  const float sx = -0.5;
  const float sy = -0.5;
  trace.beginBlock( "Compute fully subconvex sets" );
  for ( size_t i = 0; i < T.size(); ++i )
    for ( size_t j = i+2; j < T.size(); ++j )
      {
        aBoard.setPenColorRGBi( rand() % 255, rand() % 255, rand() % 255 );
        size_t k = (i+j)/2;
        if ( ! dconv.isSimplexFullDimensional( { T[i], T[j], T[k] } ) ) continue;
        auto triangle = dconv.makeSimplex( { T[i], T[j], T[k] } );
        if ( dconv.isFullySubconvex( triangle, c_cover ) )
          {
            aBoard.drawLine( sx+(float)T[i][0], sy+(float)T[i][1],
                             sx+(float)T[j][0], sy+(float)T[j][1] );
            aBoard.drawLine( sx+(float)T[i][0], sy+(float)T[i][1],
                             sx+(float)T[k][0], sy+(float)T[k][1] );
            aBoard.drawLine( sx+(float)T[k][0], sy+(float)T[k][1],
                             sx+(float)T[j][0], sy+(float)T[j][1] );
          }
        else
          j = T.size();
      }
  trace.endBlock();
  aBoard.setPenColor( Color::Black );
  aBoard << c;
  aBoard.saveEPS( "myGridCurve.eps", Board2D::BoundingBox );//, 5000 );
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
