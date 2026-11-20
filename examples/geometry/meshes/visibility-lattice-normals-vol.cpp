#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/surfaces/estimation/VisibilityLatticeEstimator.h"

using namespace DGtal;

int main( int argc, char* argv[] )
{
  if ( argc < 3 || argc > 6 )
  {
    std::cerr << "Usage: " << argv[ 0 ]
              << " input.vol output.txt [gridstep [radius [sigma]]]\n";
    return 1;
  }

  const double gridstep = argc > 3 ? std::stod( argv[ 3 ] ) : 1.0;
  const int radius      = argc > 4 ? std::stoi( argv[ 4 ] ) : -1;
  if ( gridstep <= 0.0 )
  {
    std::cerr << "The gridstep must be positive.\n";
    return 1;
  }
  const double sigma = argc > 5 ? std::stod( argv[ 5 ] ) : 1.0 / std::sqrt( gridstep );
  if ( sigma <= 0.0 )
  {
    std::cerr << "Sigma must be positive.\n";
    return 1;
  }

  typedef Shortcuts< Z3i::KSpace > SH3;
  typedef VisibilityLatticeEstimator< SH3::Space > VLE;

  auto params = SH3::defaultParameters() | ShortcutsGeometry< Z3i::KSpace >::defaultParameters();
  params( "gridstep", gridstep )
        ( "thresholdMin", 0 )( "thresholdMax", 255 )
        ( "surfaceComponents", "All" )( "surfelAdjacency", 0 )
        ( "surfaceTraversal", "default" );

  auto image   = SH3::makeBinaryImage( argv[ 1 ], params );
  auto K       = SH3::getKSpace( image, params );
  auto surface = SH3::makeDigitalSurface( image, K, params );

  SH3::Cell2Index c2i;
  const auto pointelCells = SH3::getPointelRange( c2i, surface );
  std::vector< SH3::Point > pointels( pointelCells.size() );
  std::transform( pointelCells.cbegin(), pointelCells.cend(), pointels.begin(),
                  [&] ( const SH3::Cell& c ) { return K.uKCoords( c ) / 2; } );

  const int visibilityRadius = VLE::resolveVisibilityRadius( radius, gridstep );
  const auto visibility = VLE::getVisibility( pointels, 1, visibilityRadius );
  const auto normals = VLE::getNormalVisibility
    ( pointels, visibility, sigma, 2.0 * sigma, true, 1.0,
      VLE::VisibilityCenter::Centroid, VLE::VisibilityKernel::Gaussian );

  std::ofstream output( argv[ 2 ] );
  if ( !output )
  {
    std::cerr << "Cannot open output file: " << argv[ 2 ] << '\n';
    return 1;
  }
  output << std::setprecision( 17 );
  auto embedder = SH3::getCellEmbedder( K );
  for ( std::size_t i = 0; i < pointelCells.size(); ++i )
  {
    const auto position = gridstep * embedder( pointelCells[ i ] );
    output << position[ 0 ] << ' ' << position[ 1 ] << ' ' << position[ 2 ] << ' '
           << normals[ i ][ 0 ] << ' ' << normals[ i ][ 1 ] << ' ' << normals[ i ][ 2 ] << '\n';
  }
  if ( !output )
  {
    std::cerr << "Failed while writing output file: " << argv[ 2 ] << '\n';
    return 1;
  }

  std::cout << "pointels=" << pointels.size()
            << " normals=" << normals.size()
            << " radius=" << visibilityRadius
            << " sigma=" << sigma
            << " neighborhood=" << 2.0 * sigma << '\n';
  return pointels.size() == normals.size() ? 0 : 1;
}
