/**
 * @file lower-integer-convex-hull.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/06
 *
 * An example file named lower-integer-convex-hull.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [lower-integer-convex-hull-basicIncludes]
#include "DGtal/base/Common.h"
#include "DGtal/arithmetic/LatticePolytope2D.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
//! [lower-integer-convex-hull-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

void usage( int, char** argv )
{
  std::cerr << "Usage: " << argv[ 0 ] << " <a> <b> <c>" << std::endl;
  std::cerr << "\t - Cuts a square by ax+by <= c. Displays everything in files "
            << "lower-integer-convex-hull*.eps" << std::endl;
}

/**
   Main.
*/
int main( int argc, char** argv )
{
  if ( argc < 4 )
    {
      usage( argc, argv );
      return 0;
    }

  //! [lower-integer-convex-hull-types]
  using namespace Z2i;
  typedef LatticePolytope2D<Space> CIP;
  //! [lower-integer-convex-hull-types]

  //! [lower-integer-convex-hull-instantiation]
  CIP cip;
  cip.push_front( Point( -10, -10 ) );
  cip.push_front( Point( -10, 10 ) );
  cip.push_front( Point( 10, 10 ) );
  cip.push_front( Point( 10, -10 ) );
  Domain domain = cip.boundingBoxDomain();
  Board2D board;
  board << domain 
        << CustomStyle( cip.className(), 
                        new CustomColors( Color::Red, Color::None ) )
        << cip;
  board.saveEPS( "lower-integer-convex-hull.eps" );
  board.clear();
  //! [lower-integer-convex-hull-instantiation]

  int a = atoi( argv[ 1 ] );
  int b = atoi( argv[ 2 ] );
  int c = atoi( argv[ 3 ] );

  //! [lower-integer-convex-hull-process]
  typedef LatticePolytope2D<Z2>::HalfSpace HalfSpace;
  HalfSpace hs( Vector( a, b ), c );
  cip.cut( hs );
  DigitalSet aSet( domain );
  Shapes<Domain>::makeSetFromPointPredicate( aSet, hs );
  board << domain 
        << CustomStyle( aSet.className(), 
                        new CustomColors( Color::Green, Color::Green ) )
        << SetMode( Point().className(), "Grid" )
        << aSet
        << CustomStyle( cip.className(), 
                        new CustomColors( Color::Red, Color::None ) )
        << cip;
  board.saveEPS( "lower-integer-convex-hull-cut.eps" );
  //! [lower-integer-convex-hull-process]

  //! [lower-integer-convex-hull-stats]
  std::cout << "Number of vertices        = " << cip.size() << std::endl;
  std::cout << "Area                      = " << (((double)cip.twiceArea())/2.0) << std::endl;
  std::cout << "Number of interior points = " << cip.numberInteriorPoints() << std::endl;
  std::cout << "Number of boundary points = " << cip.numberBoundaryPoints() << std::endl;
  //! [lower-integer-convex-hull-stats]
  return 0;
}

