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
 * @file testShortcuts.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/08/28
 *
 * Functions for testing class Shortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtalCatch.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;


///////////////////////////////////////////////////////////////////////////////
// Functions for testing class Shortcuts.
///////////////////////////////////////////////////////////////////////////////

SCENARIO( "Shortcuts< K3 > pointel ordering", "[shortcuts][pointel]" )
{
  typedef KhalimskySpaceND<3>                       KSpace;
  typedef Shortcuts< KSpace >                       SH3;

  auto params          = SH3::defaultParameters();
  const double h       = 0.25;
  params( "polynomial", "goursat" )( "gridstep", h );
  auto implicit_shape  = SH3::makeImplicitShape3D  ( params );
  auto digitized_shape = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
  auto binary_image    = SH3::makeBinaryImage      ( digitized_shape, params );
  auto K               = SH3::getKSpace( params );
  auto embedder        = SH3::getCellEmbedder( K );
  auto surface         = SH3::makeLightDigitalSurface( binary_image, K, params );

  GIVEN( "A digital surface, its associated polygonal surface, and its pointel range" ) {
    SH3::Cell2Index c2i;
    auto polySurf        = SH3::makePrimalPolygonalSurface( c2i, surface );
    auto pointels        = SH3::getPointelRange( surface );
    THEN( "The polygonal surface and the pointel range have the same number of pointels" ) {
      REQUIRE( pointels.size() == polySurf->nbVertices() );
    }
    THEN( "The vertices of the polygonal surface are in the same order as the pointel range" ) {
      unsigned int nb_ok = 0, nb_ko = 0;
      for ( size_t i = 0; i < polySurf->nbVertices(); i++ )
	{
	  auto    p = pointels[ i ];
	  auto  idx = c2i[ p ];
	  if ( i != idx )
	    {
	      DGtal::trace.error() << "Pointel " << p << " of primal polygonal surface has not the same index in the polygonal surface (" << idx << ") and in the pointel range (" << i << ")." << std::endl;
	      nb_ko += 1;
	    }
	  else nb_ok += 1;
	}
      (void)nb_ok;//not used
      REQUIRE( nb_ko == 0 );
    }
  }
}

//                                                                           //
///////////////////////////////////////////////////////////////////////////////
