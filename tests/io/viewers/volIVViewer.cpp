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
 * @file testIVViewer.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2010/12/04
 *
 * Functions for testing class IVViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <Qt/qapplication.h>
#include <Inventor/nodes/SoCube.h>
#include "DGtal/io/viewers/DGtalInventor.h"
#include "DGtal/io/viewers/IVViewer.h"
#include "DGtal/base/Common.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "ConfigTest.h"

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class IVViewer.
///////////////////////////////////////////////////////////////////////////////
using namespace Z3i;


void addAxis( DGtalInventor<Space> & inventor, 
        const Point & low,
        const Point & up,
        Dimension k, 
        const DGtal::Color & c1,
        const DGtal::Color & c2 )
{
  typedef DGtalInventor<Space>::Color Color;
  // @todo GradientColorMap seems to have a bug
  GradientColorMap<int> cmap_grad( 0, up[ k ] - low[ k ] );
  cmap_grad.clearColors();
  cmap_grad.addColor( c1 );
  cmap_grad.addColor( c2 );
  Point p( low );
  for ( Point::Component x = low[ k ]; x <= up[ k ]; ++x )
    {
      p[ k ] = x;
      DGtal::Color c( cmap_grad( x - low[ k ] ) );
      inventor.setDiffuseColor( Color( c.red(), c.green(), c.blue() ) );
      inventor.drawPoint( p );
    }
}
void addBounds( DGtalInventor<Space> & inventor, 
    const Point & low, 
    const Point & up )
{
  typedef DGtalInventor<Space>::Color Color;
  Point p( low );
  for ( Dimension i = 0; i < 8; ++i )
    {
      Color c;
      for ( Dimension j = 0; j < 3; ++j )
  {
    p[ j ] = ( i & ( 1 << j ) ) ? up[ j ] : low[ j ];
    c[ j ] = ( i & ( 1 << j ) ) ? 1.0 : 0.1 ;
  }
      inventor.setDiffuseColor( c ); //Color( 0.1, 0.1, 0.1 ) );
      inventor.drawPoint( p );
    }
  // addAxis( inventor, low, up, 0, 
  //      LibBoard::Color::Black, LibBoard::Color::Blue );
  // addAxis( inventor, low, up, 1, 
  //       LibBoard::Color::Black, LibBoard::Color::Green );
  // addAxis( inventor, low, up, 2, 
  //       LibBoard::Color::Black, LibBoard::Color::Red );
}

int main( int argc, char** argv )
{
  QApplication app( argc, argv );
  trace.beginBlock ( "Testing class IVViewer" );
  IVViewer ivv( argc, argv );

  // Load volumetric image
  typedef ImageSelector<Domain, unsigned char>::Type Image;
  std::string filename = testPath + "samples/cat10.vol";
  Image image = VolReader<Image>::importVol( filename );
  trace.info() << image <<endl;

  // make shape.
  Domain domain =  image.domain() ;
  DigitalSet shape_set( domain );
  for ( Domain::ConstIterator it = domain.begin(), itend = domain.end();
  it != itend;   
  ++it )
    {
      if ( image( *it ) != 0 )
  shape_set.insert( *it );
    }

  // find first surfel on the boundary (not nice).
  Point first;
  for ( Domain::ConstIterator it = domain.begin(), itend = domain.end();
  it != itend;   
  ++it )
    {
      if ( image( *it ) != 0 )
  {
    first = *it;
    break;
  }
    }

  // Builds Khalimsky space.
  typedef KhalimskySpaceND<3> KSpace;
  typedef KSpace::SCell SCell;
  KSpace K3;
  SurfelAdjacency<KSpace::dimension> SAdj( true );
  K3.init( image.domain().lowerBound(), image.domain().upperBound(), true );

  // Tracks the shape boundary.
  SCell intvoxel = K3.sSpel( first );
  SCell surfel = K3.sIncident( intvoxel, 0, false );
  std::set<SCell> bdry;
  Surfaces<KSpace>::trackBoundary( bdry,
           K3, SAdj, shape_set, surfel );

  trace.info() << "tracking finished, size=" << bdry.size() << endl; 

  // Display surface.
  DGtalInventor<Space> inventor;
  typedef DGtalInventor<Space>::Color Color;
  addBounds( inventor, image.domain().lowerBound(), image.domain().upperBound() );
  for ( std::set<SCell>::const_iterator it = bdry.begin(), itend = bdry.end();
  it != itend;
  ++it )
    {
      inventor.setDiffuseColor( Color( 0.7, 0.7, 1.0 ) );
      inventor.drawCell( K3.sKCoords( *it ), 
       ! K3.sDirect( *it, K3.sOrthDir( *it ) ) );
    }
  // start surfel is red.
  inventor.setDiffuseColor( Color( 1.0, 0.0, 0.0 ) );
  inventor.drawCell( K3.sKCoords( surfel ), 
         ! K3.sDirect( surfel, K3.sOrthDir( surfel ) ) );
  inventor.generate( ivv.root() );
  // Qt will get the hand.
  ivv.show();

  bool res = true;
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
