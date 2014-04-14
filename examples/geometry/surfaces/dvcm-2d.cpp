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
 * @file dvcm-2d.cpp
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/01/31
 *
 * Computes the 2d voronoi map of a list of digital points.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

#include "DGtal/kernel/BasicPointPredicates.h"
#include "DGtal/math/EigenDecomposition.h"
#include "DGtal/geometry/volumes/distance/ExactPredicateLpSeparableMetric.h"
#include "DGtal/geometry/volumes/estimation/VoronoiCovarianceMeasure.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/boards/Board2D.h"
#include "ConfigExamples.h"


///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  typedef DGtal::Z2i::Space Space;
  typedef DGtal::Z2i::KSpace KSpace;
  typedef DGtal::Z2i::Vector Vector;
  typedef DGtal::Z2i::Point Point;
  typedef DGtal::Z2i::RealPoint RealPoint;
  typedef DGtal::Z2i::RealVector RealVector;
  typedef DGtal::HyperRectDomain<Space> Domain;
  typedef DGtal::ImageContainerBySTLVector<Domain,bool> CharacteristicSet;
  typedef DGtal::ExactPredicateLpSeparableMetric<Space, 2> Metric; // L2-metric
  typedef DGtal::EigenDecomposition<2,double> LinearAlgebraTool;
  typedef LinearAlgebraTool::Matrix Matrix;
  typedef KSpace::Surfel Surfel;
  typedef KSpace::SCell SCell;

  typedef VoronoiCovarianceMeasure<Space,Metric> VCM;
  typedef HatPointFunction<Point,double> KernelFunction;

  // Gets the points
  vector<unsigned int> vPos;
  vPos.push_back(0);
  vPos.push_back(1);
  // string inputSDP = examplesPath + "samples/flower-30-8-3.sdp";
  string inputSDP = examplesPath + "samples/ellipse-20-7-0.4.sdp";
  trace.info() << "Reading input 2d discrete points file: " << inputSDP; 
  std::vector<Point> pts = PointListReader<Point>::getPointsFromFile(inputSDP, vPos); 
  trace.info() << " [done] " << std::endl ; 
  const double R = 20;
  trace.info() << "Big radius   R = " << R << std::endl;
  const double r = 5;
  trace.info() << "Small radius r = " << r << std::endl;
  const double size = 3.0;
  Metric l2;
  VCM vcm( R, ceil( r ), l2, true );
  vcm.init( pts.begin(), pts.end() );
  Domain domain = vcm.domain();
  KernelFunction chi( 1.0, r );

  // First pass to detect maxima.
  Matrix evec;
  RealVector eval;
  double feature_max = 0.0;
  for ( std::vector<Point>::const_iterator it = pts.begin(), itE = pts.end();
        it != itE; ++it )
    {
      // Compute VCM and diagonalize it.
      Matrix vcm_r = vcm.measure( chi, *it );
      LinearAlgebraTool::getEigenDecomposition( vcm_r, evec, eval );
      double feature = eval[ 0 ] / ( eval[ 0 ] +  eval[ 1 ] );
      std::cerr << feature << " : " << eval[ 0 ] << " " << eval[ 1 ] << std::endl;
      feature_max = std::max( feature_max, feature );
    }
  // Flat zones are metallic blue
  // slightly curved zones are white
  // more curved zones are yellow till red.
  GradientColorMap<double> colormap( 0.0, feature_max );
  colormap.addColor( Color( 128, 128, 255 ) );
  colormap.addColor( Color( 255, 255, 255 ) );
  colormap.addColor( Color( 255, 255, 0 ) );
  colormap.addColor( Color( 255, 0, 0 ) );
  Board2D board;
  // Second pass to display everything.
  for ( std::vector<Point>::const_iterator it = pts.begin(), itE = pts.end();
        it != itE; ++it )
    {
      // Compute VCM and diagonalize it.
      Matrix vcm_r = vcm.measure( chi, *it );
      LinearAlgebraTool::getEigenDecomposition( vcm_r, evec, eval );
      double feature = eval[ 0 ] / ( eval[ 0 ] +  eval[ 1 ] );
      board << CustomStyle( it->className(), new CustomColors( Color::Black,  colormap( feature ) ) )
            << *it;
      // Display normal
      RealVector normal = evec.column( 1 );
      RealPoint p( (*it)[ 0 ], (*it)[ 1 ] ); 
      Display2DFactory::draw( board, size*normal, p );
      Display2DFactory::draw( board, -size*normal, p );
    }      
  board.saveSVG("dvcm-hat-r.svg");

  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
