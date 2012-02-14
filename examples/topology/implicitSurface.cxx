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
 * @file implicitSurface.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/06
 *
 * A simple marching cube algorithm based on digital surfaces.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [implicitSurface-basicIncludes]
#include <iostream>
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/math/MPolynomial.h"
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/shapes/implicit/ImplicitPolynomial3Shape.h"
//! [implicitSurface-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i;

///////////////////////////////////////////////////////////////////////////////


void usage( int argc, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <fileName.vol> <minT> <maxT> <int=0|ext=1>" << std::endl;
  std::cerr << "\t - displays the boundary of the shape stored in vol file <fileName.vol> as an OFF geomview surface file. It is a kind of marching-cube surface, defined by duality with respect to the digital surface." << std::endl;
  std::cerr << "\t - voxel v belongs to the shape iff its value I(v) follows minT <= I(v) <= maxT." << std::endl;
  std::cerr << "\t - 0: interior adjacency, 1: exterior adjacency." << std::endl;
}

int main( int argc, char** argv )
{
  if ( argc < 1 )
    {
      usage( argc, argv );
      return 1;
    }
  // std::string inputFilename = argv[ 1 ];
  // unsigned int minThreshold = atoi( argv[ 2 ] );
  // unsigned int maxThreshold = atoi( argv[ 3 ] );
  // bool intAdjacency = atoi( argv[ 4 ] ) == 0;

  //! [implicitSurface-makeSurface]
  trace.beginBlock( "Making polynomial surface." );
  typedef typename Space::RealPoint RealPoint;
  typedef typename RealPoint::Coordinate Ring;
  typedef MPolynomial<3, Ring> Polynomial3;
  typedef ImplicitPolynomial3Shape<Space> ImplicitShape;
  typedef GaussDigitizer<Space,ImplicitShape> DigitalShape; 

  MPolynomial<3, double> P = mmonomial<double>( 3, 1, 0 )
    + mmonomial<double>( 1, 0, 3 )
    + mmonomial<double>( 0, 3, 1 )
    + mmonomial<double>( 0, 0, 3 )
    + 5 * mmonomial<double>( 0, 0, 1 );
  ImplicitShape ishape( P );
  DigitalShape dshape;
  dshape.attach( ishape );
  // dshape.init( RealPoint( -1.0, -1.0, -1.0 ), 
  //              RealPoint( 1.0, 1.0, 1.0 ), 0.2 );
  dshape.init( RealPoint( -10.0, -10.0, -10.0 ), 
               RealPoint( 10.0, 10.0, 10.0 ), 0.25 );
  Domain domain = dshape.getDomain();
  trace.endBlock();
  //! [implicitSurface-makeSurface]

  //! [implicitSurface-KSpace]
  // Construct the Khalimsky space from the image domain
  KSpace K;
  // NB: it is \b necessary to work with a \b closed cellular space
  // since umbrellas use separators and pivots, which must exist for
  // arbitrary surfels.
  bool space_ok = K.init( domain.lowerBound(), 
                          domain.upperBound(), true );
  if (!space_ok)
    {
      trace.error() << "Error in the Khamisky space construction."<<std::endl;
      return 2;
    }
  trace.info() << "lower point = " << K.lowerBound() << std::endl;
  trace.info() << "upper point = " << K.upperBound() << std::endl;
  trace.info() << "lower cell  = " << K.lowerCell()  << std::endl;
  trace.info() << "upper cell  = " << K.upperCell()  << std::endl;
  //! [implicitSurface-KSpace]

  //! [implicitSurface-SurfelAdjacency]
  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  MySurfelAdjacency surfAdj( true ); // interior in all directions.
  //! [implicitSurface-SurfelAdjacency]

  //! [implicitSurface-ExtractingSurface]
  trace.beginBlock( "Extracting boundary by tracking the space. " );
  typedef KSpace::Surfel Surfel;
  typedef KSpace::SurfelSet SurfelSet;
  typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
  typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;

  
  MySetOfSurfels theSetOfSurfels( K, surfAdj );
  Surfel bel = Surfaces<KSpace>::findABel( K, dshape, 100000 );
  Surfaces<KSpace>::trackBoundary( theSetOfSurfels.surfelSet(),
                                   K, surfAdj, 
                                   dshape, bel );
  trace.endBlock();
  //! [implicitSurface-ExtractingSurface]

  //! [implicitSurface-makingOFF]
  trace.beginBlock( "Making OFF surface <marching-cube.off>. " );
  MyDigitalSurface digSurf( theSetOfSurfels );
  trace.info() << "Digital surface has " << digSurf.size() << " surfels."
               << std::endl;
  ofstream out( "marching-cube.off" );
  if ( out.good() )
    digSurf.exportSurfaceAs3DOFF( out );
  out.close();
  trace.endBlock();
  //! [implicitSurface-makingOFF]

  return 0;
}
