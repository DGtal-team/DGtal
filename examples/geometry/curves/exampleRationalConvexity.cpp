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

   \image html grid-curve-subconvex-rational-segment.png "Extraction of all maximal rational segments between midpoints that are subconvex to the digital curve."

   \example geometry/curves/exampleRationalConvexity.cpp
*/


///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/topology/KhalimskySpaceND.h"
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
  DigitalConvexity<KSpace> dconv( lowerBound, upperBound );

  fstream inputStream( S.c_str(), ios::in );
  FreemanChain<int> fc(inputStream);
  inputStream.close();
  Curve c;
  c.initFromPointsRange( fc.begin(), fc.end() );
  auto points = c.getPointsRange();
  std::vector<Point> T( points.begin(), points.end() );
  auto midpoints = c.getMidPointsRange();
  std::vector<RealPoint> RT( midpoints.begin(), midpoints.end() );
  std::vector<Point> T2;
  for ( auto && rp : midpoints )
    // there is a shift of (0.5,0.5) between points and cells embedder.
    T2.push_back( Point( (int) round( 2. * rp[ 0 ] + 1. ),
                         (int) round( 2. * rp[ 1 ] + 1. ) ) );
  Board2D aBoard;
  aBoard.setUnit(Board2D::UCentimeter);
  // Display cells
  const KSpace& K = dconv.space();
  Color grey( 200, 200, 200 );
  std::set<Cell> pixels;
  for ( auto p : T )
    {
      pixels.insert( K.uCell( Point( 2*p[ 0 ] - 1, 2*p[ 1 ] - 1 ) ) );
      pixels.insert( K.uCell( Point( 2*p[ 0 ] + 1, 2*p[ 1 ] - 1 ) ) );
      pixels.insert( K.uCell( Point( 2*p[ 0 ] - 1, 2*p[ 1 ] + 1 ) ) );
      pixels.insert( K.uCell( Point( 2*p[ 0 ] + 1, 2*p[ 1 ] + 1 ) ) );
    }
  for ( auto && pixel : pixels )
    aBoard << CustomStyle( pixel.className(), new CustomColors( grey, grey ) )
           << pixel;
  // Display contour
  aBoard.setPenColor( Color::Black );
  aBoard << c;
  // Compute subconvex rational segments.
  auto    c_cover = dconv.makeCellCover( T.begin(), T.end(), 1, 1 );
  trace.beginBlock( "Compute fully subconvex rational sets" );
  Point denominator( 2, 2 );
  unsigned int last_j = 0;
  unsigned int      j = 0;
  for ( unsigned int i = 0; i < T2.size(); ++i )
    {
      aBoard.setPenColorRGBi( rand() % 255, rand() % 255, rand() % 255 );
      unsigned int start_j = ( i + 1 ) % T2.size();
      for ( j = ( start_j + 1 ) % T2.size(); j != start_j;  j = ( j + 1 ) % T2.size() )
        {
          auto segment = dconv.makeRationalSimplex( { denominator, T2[i], T2[j] } );
          if ( ! dconv.isFullySubconvex( segment, c_cover ) ) break;
        }
      j = (unsigned int)( j + T2.size() - 1 ) % T2.size();
      if ( j != last_j )
        { // display fully subconvex segments
          aBoard.setLineWidth( 2.5 );
          aBoard.drawLine( RT[i][0], RT[i][1], RT[j][0], RT[j][1] );
        }
      last_j = j;
    }
  trace.endBlock();
  aBoard.saveEPS( "myGridCurve.eps", Board2D::BoundingBox );//, 5000 );
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
