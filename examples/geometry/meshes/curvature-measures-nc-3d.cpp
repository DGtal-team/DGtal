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
 * @file geometry/meshes/curvature-measures-nc-3d.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/25
 *
 * An example file named curvature-measures-nc-3d.
 *
 * This file is part of the DGtal library.
 */

/**
   Computation of curvatures on a torus mesh, using Normal cycle
   curvature measures (based on the theory of Normal cycle)

\verbatim
./examples/geometry/meshes/curvature-measures-nc-3d torus 20 20 0.5 
\endverbatim
outputs
\verbatim
Expected mean curvatures: min=0.25 max=0.625
Computed mean curvatures: min=0.189446 max=0.772277
Expected Gaussian curvatures: min=-0.5 max=0.25
Computed Gaussian curvatures: min=-0.682996 max=0.547296
\endverbatim

It also produces several OBJ files to display curvature estimation
results, `example-nc-H.obj` and `example-nc-G.obj` as well as the
associated MTL file.

@warning Normal cycle curvature estimation have meaning only for a big enough radius to capture edge/vertex information. This is in opposition with interpolated corrected curvature measures.

<table>
<tr><td>
\image html torus-nc-H-True-r0_5.jpg "Normal cycle mean curvature measure, r=0.5" width=90%
</td><td>
\image html torus-nc-G-True-r0_5.jpg "Normal cycle Gaussian curvature measure, r=0.5" width=90%
</td></tr>
</table>

@see \ref moduleCurvatureMeasures

@note Normal cycle theory fails on bad sampling of smooth surfaces. A
well known example is the Schwarz lantern. You may try the following:

\verbatim
./examples/geometry/meshes/curvature-measures-nc-3d lantern 20 20 0.5
\endverbatim

outputs 

\verbatim
Expected mean curvatures: min=0.25 max=0.25
Computed mean curvatures: min=0.795695 max=1.41211
Expected Gaussian curvatures: min=0 max=0
Computed Gaussian curvatures: min=-6.79045e-14 max=15.0937
\endverbatim
  
\example geometry/meshes/curvature-measures-nc-3d.cpp
*/

#include <iostream>
#include <algorithm>
#include "DGtal/base/Common.h"
#include "DGtal/shapes/SurfaceMesh.h"
#include "DGtal/shapes/SurfaceMeshHelper.h"
//! [curvature-measures-Includes]
#include "DGtal/geometry/meshes/NormalCycleComputer.h"
//! [curvature-measures-Includes]
#include "DGtal/io/writers/SurfaceMeshWriter.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/QuantifiedColorMap.h"

DGtal::GradientColorMap< double >
makeColorMap( double min_value, double max_value )
{
  DGtal::GradientColorMap< double > gradcmap( min_value, max_value );
  gradcmap.addColor( DGtal::Color( 0, 0, 255 ) );
  gradcmap.addColor( DGtal::Color( 0, 255, 255 ) );
  gradcmap.addColor( DGtal::Color( 255, 255, 255 ) );
  gradcmap.addColor( DGtal::Color( 255, 255, 0 ) );
  gradcmap.addColor( DGtal::Color( 255, 0, 0 ) );
  return gradcmap;
}

void usage( int argc, char* argv[] )
{
  ((void) argc);
  std::cout << "Usage: " << std::endl
            << "\t" << argv[ 0 ] << " <shape> <m> <n> <R>" << std::endl
            << std::endl
            << "Computation of mean and Gaussian curvatures on a shape, " << std::endl
            << "using Normal cycle curvature measures (based on the"     << std::endl
            << "theory of Normal cycle)"                                 << std::endl
            << "- builds a <shape> in {torus,lantern,sphere}, with     " << std::endl
            << "  <m> latitude points and <n> longitude points."         << std::endl
            << "- <R> is the radius of the measuring balls."             << std::endl
            << "It produces several OBJ files to display mean and"       << std::endl
            << "Gaussian curvature estimation results: `example-cnc-H.obj`" << std::endl
            << "and `example-cnc-G.obj` as well as the associated MTL file." << std::endl;
}

int main( int argc, char* argv[] )
{
  if ( argc <= 1 )
    {
      usage( argc, argv );
      return 0;
    }
  //! [curvature-measures-Typedefs]
  using namespace DGtal;
  using namespace DGtal::Z3i;
  typedef SurfaceMesh< RealPoint, RealVector >         SM;
  typedef NormalCycleComputer< RealPoint, RealVector > NC;
  typedef SurfaceMeshHelper< RealPoint, RealVector >   SMH;
  //! [curvature-measures-Typedefs]
  std::string input = argv[ 1 ];
  int    m = argc > 2 ? atoi( argv[ 2 ] ) : 20;  // nb latitude points
  int    n = argc > 3 ? atoi( argv[ 3 ] ) : 20;  // nb longitude points
  double R = argc > 4 ? atof( argv[ 4 ] ) : 0.5; // radius of measuring ball

  //! [curvature-measures-SurfaceMesh]
  SM smesh;
  double exp_H_min = 0.0;
  double exp_H_max = 0.0;
  double exp_G_min = 0.0;
  double exp_G_max = 0.0;
  if ( input == "torus" )
    {
      const double big_radius   = 3.0;
      const double small_radius = 1.0;
      smesh = SMH::makeTorus( big_radius, small_radius,
                              RealPoint { 0.0, 0.0, 0.0 }, m, n, 0,
                              SMH::NormalsType::VERTEX_NORMALS );
      exp_H_min = ( 0.5 / ( small_radius - big_radius ) + 0.5 / small_radius );
      exp_H_max = ( 0.5 / ( big_radius + small_radius ) + 0.5 / small_radius );
      exp_G_min = ( 1.0 / ( small_radius - big_radius ) * 1.0 / small_radius );
      exp_G_max = ( 1.0 / ( big_radius + small_radius ) * 1.0 / small_radius );
    }
  else if ( input == "sphere" )
    {
      const double radius = 2.0;
      smesh = SMH::makeSphere( radius, RealPoint { 0.0, 0.0, 0.0 }, m, n,
                               SMH::NormalsType::VERTEX_NORMALS );
      exp_H_min = 1.0 / radius;
      exp_H_max = 1.0 / radius;
      exp_G_min = 1.0 / ( radius * radius );
      exp_G_max = 1.0 / ( radius * radius );
    }
  else if ( input == "lantern" )
    {
      const double radius = 2.0;
      smesh = SMH::makeLantern( radius, 1.0, RealPoint { 0.0, 0.0, 0.0 }, m, n,
                                SMH::NormalsType::VERTEX_NORMALS );
      exp_H_min = 0.5 / radius;
      exp_H_max = 0.5 / radius;
      exp_G_min = 0.0;
      exp_G_max = 0.0;
    }
  //! [curvature-measures-SurfaceMesh]

  //! [curvature-measures-CNC]
  // builds a NormalCycleComputer object onto the mesh
  NC nc( smesh );
  // computes area, mean and Gaussian curvature measures
  auto mu0 = nc.computeMu0();
  auto mu1 = nc.computeMu1();
  auto mu2 = nc.computeMu2();
  //! [curvature-measures-CNC]

  //! [curvature-measures-estimations]
  // estimates mean (H) and Gaussian (G) curvatures by measure normalization.
  std::vector< double > H( smesh.nbFaces() );
  std::vector< double > G( smesh.nbFaces() );
  for ( auto f = 0; f < smesh.nbFaces(); ++f )
    {
      const auto b    = smesh.faceCentroid( f );
      const auto area = mu0.measure( b, R, f );
      H[ f ] = nc.meanCurvature    ( area, mu1.measure( b, R, f ) );
      G[ f ] = nc.GaussianCurvature( area, mu2.measure( b, R, f ) );
    }
  //! [curvature-measures-estimations]

  //! [curvature-measures-check]
  auto H_min_max = std::minmax_element( H.cbegin(), H.cend() );
  auto G_min_max = std::minmax_element( G.cbegin(), G.cend() );
  std::cout << "Expected mean curvatures:"
            << " min=" << exp_H_min << " max=" << exp_H_max
            << std::endl;
  std::cout << "Computed mean curvatures:"
            << " min=" << *H_min_max.first << " max=" << *H_min_max.second
            << std::endl;
  std::cout << "Expected Gaussian curvatures:"
            << " min=" << exp_G_min << " max=" << exp_G_max
            << std::endl;
  std::cout << "Computed Gaussian curvatures:"
            << " min=" << *G_min_max.first << " max=" << *G_min_max.second
            << std::endl;
  //! [curvature-measures-check]

  //! [curvature-measures-output]
  typedef SurfaceMeshWriter< RealPoint, RealVector > SMW;
  const auto colormapH = makeQuantifiedColorMap( makeColorMap( -0.625, 0.625 ) );
  const auto colormapG = makeQuantifiedColorMap( makeColorMap( -0.625, 0.625 ) );
  auto colorsH = SMW::Colors( smesh.nbFaces() );
  auto colorsG = SMW::Colors( smesh.nbFaces() );
  for ( auto i = 0; i < smesh.nbFaces(); i++ )
    {
      colorsH[ i ] = colormapH( H[ i ] );
      colorsG[ i ] = colormapG( G[ i ] );
    }
  SMW::writeOBJ( "example-nc-H", smesh, colorsH );
  SMW::writeOBJ( "example-nc-G", smesh, colorsG );
  //! [curvature-measures-output]
  return 0;
}
