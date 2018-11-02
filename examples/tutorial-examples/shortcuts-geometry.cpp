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
 * @file
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2018/06/26
 *
 * An example file named shortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/helpers/ShortcutsGeometry.h"
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  // 3d tests
  {
    typedef Shortcuts< Z3i::KSpace >         SH3;
    typedef ShortcutsGeometry< Z3i::KSpace > SHG3;
    trace.beginBlock ( "Setting up shape, space, etc" );
    auto params         = SH3::defaultParameters();
    params( "polynomial", "goursat" )( "gridstep", 0.5 )
      ( "surfaceComponents", "All" )( "surfelAdjacency",   0 );
    auto implicit_shape = SH3::makeImplicitShape3D( params );
    auto K              = SH3::getKSpace( params );
    auto digital_shape  = SH3::makeDigitizedImplicitShape3D( implicit_shape, params );
    auto binary_image   = SH3::makeBinaryImage( digital_shape, params );
    trace.endBlock();

    trace.beginBlock ( "Compute true geometry" );
    params( "surfaceTraversal", "DepthFirst" )( "verbose", 0 );
    auto surface     = SH3::makeLightDigitalSurface( binary_image, K, params );
    auto surfels     = SH3::getSurfelRange( surface, params );
    auto positions   = SHG3::getPositions( implicit_shape, K, surfels, params ); 
    auto normals     = SHG3::getNormalVectors( implicit_shape, K, surfels, params ); 
    auto mean_curv   = SHG3::getMeanCurvatures( implicit_shape, K, surfels, params ); 
    auto gauss_curv  = SHG3::getGaussianCurvatures( implicit_shape, K, surfels, params ); 
    SH3::RealPoint p = positions.front();
    SH3::RealPoint q = p;
    for ( auto&& r : positions ) {
      p = p.inf( r ); q = q.sup( r );
    }
    SH3::Scalar   h0 =  1000.0;
    SH3::Scalar   h1 = -1000.0;
    for ( auto&& h : mean_curv ) {
      h0 = std::min( h0, h );	h1 = std::max( h1, h );
    }
    SH3::Scalar   g0 =  1000.0;
    SH3::Scalar   g1 = -1000.0;
    for ( auto&& g : gauss_curv ) {
      g0 = std::min( g0, g );	g1 = std::max( g1, g );
    }
    std::cout << "#position = " << positions.size()
	      << " p=" << p << " q=" << q << std::endl;
    std::cout << "#normals = " << normals.size() << std::endl;
    std::cout << "H_min = " << h0 << " H_max = " << h1;
    std::cout << " expected: H_min = 0.0912870 H_max = 0.263523" << std::endl;
    std::cout << "G_min = " << g0 << " G_max = " << g1;
    std::cout << " expected: G_min = 0.0074074 G_max = 0.0666666" << std::endl;
    trace.endBlock();

    trace.beginBlock ( "Estimate geometry" );
    auto     t_normals = SHG3::getTrivialNormalVectors( K, surfels );
    auto    ct_normals = SHG3::getCTrivialNormalVectors( surface, surfels, params );
    auto   vcm_normals = SHG3::getVCMNormalVectors( surface, surfels, params );
    auto    ii_normals = SHG3::getIINormalVectors( binary_image, surfels, params );
    // Need to reorient II normals with CTrivial (otherwise unstable orientation).
    SHG3::orientVectors( ii_normals, ct_normals );
    auto   t_angle_dev = SHG3::getVectorsAngleDeviation( normals, t_normals );
    auto  ct_angle_dev = SHG3::getVectorsAngleDeviation( normals, ct_normals );
    auto vcm_angle_dev = SHG3::getVectorsAngleDeviation( normals, vcm_normals );
    auto  ii_angle_dev = SHG3::getVectorsAngleDeviation( normals, ii_normals );
    std::cout << "Trivial  angle_dev  mean="
	      << t_angle_dev.mean() << " max=" << t_angle_dev.max() << std::endl;
    std::cout << "CTrivial angle_dev  mean="
	      << ct_angle_dev.mean() << " max=" << ct_angle_dev.max() << std::endl;
    std::cout << "VCM      angle_dev  mean="
	      << vcm_angle_dev.mean() << " max=" << vcm_angle_dev.max() << std::endl;
    std::cout << "II       angle_dev  mean="
	      << ii_angle_dev.mean() << " max=" << ii_angle_dev.max() << std::endl;
  }
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
