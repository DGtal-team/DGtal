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
 * @file image2freeman.cpp
 * @ingroup Tools
 * @author Bertrand Kerautret (\c kerautre@loria.fr)
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/27/04
 *
 * DGtal convert grey scales image to fremann contour. 
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"

#include "DGtal/topology/KhalimskySpaceND.h"
//#include "DGtal/topology/SurfelAdjacency.h"
//#include "DGtal/topology/SurfelNeighborhood.h"





#include "DGtal/helpers/ShapeFactory.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/colormaps/GrayScaleColorMap.h"
#include "DGtal/kernel/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/kernel/imagesSetsUtils/SetFromImage.h"
#include "DGtal/kernel/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/images/ImageSelector.h"
#include "DGtal/io/readers/MagickReader.h"
#include "DGtal/geometry/2d/FreemanChain.h"

#include "DGtal/io/DGtalBoard.h"
#include "DGtal/helpers/Surfaces.h"

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <vector>
#include <string>

using namespace DGtal;




///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

int main( int argc, char** argv )
{
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("image,i", po::value<std::string>(), "image file name")
    ("min,m", po::value<int>(), "min image threshold value (default 128)")
    ("max,M", po::value<int>(), "max image threshold value (default 255)")
    ("minSize,s", po::value<int>(), "minSize of the extracted freeman chain (default 0)");
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "Extract FreemanChains from thresholded image" <<std::endl << "Basic usage: "<<std::endl
		  << "\t image2freeman [options] --image <imageName> -min 128 -max 255 > contours.fc"<<std::endl
		  << general_opt << "\n";
      return 0;
    }
  
  
  double minThreshold = 128;
  double maxThreshold = 255;
  int minSize =0;
  //Parse options
  if (not(vm.count("image"))){
    trace.info() << "Image file name needed"<< endl;
    return 0;
  } 
  
  if(vm.count("min")){
    minThreshold= vm["min"].as<int>();
  } 
  if(vm.count("max")){
    maxThreshold= vm["max"].as<int>();
  } 
  if(vm.count("minSize")){
    minSize = vm["minSize"].as<int>();
  } 
  
  

  typedef ImageSelector < Z2i::Domain, int>::Type Image;
  string imageFileName = vm["image"].as<std::string>();
  Image image = MagickReader<Image>::importImage( imageFileName ); 
  
  Z2i::DigitalSet set2d (image.domain());
  SetFromImage<Z2i::DigitalSet>::append<Image>(set2d, image, minThreshold, maxThreshold);
  trace.info() << "DGtal set imported from thresholds ["<<  minThreshold << "," << maxThreshold << "]" << endl;
  Z2i::KSpace ks;
  bool space_ok = ks.init( image.domain().lowerBound(), image.domain().upperBound(), true );
  SurfelAdjacency<2> sAdj( true );
  
  std::vector< std::vector< Z2i::Point >  >  vectContoursBdryPointels;
  Surfaces<Z2i::KSpace>::extractAllPointContours4C( vectContoursBdryPointels,
						    ks, set2d, sAdj );  
  for(int i=0; i<vectContoursBdryPointels.size(); i++){
    if(vectContoursBdryPointels.at(i).size()>minSize){
      FreemanChain<Z2i::Integer> fc (vectContoursBdryPointels.at(i), true);    
      cout << fc.x0 << " " << fc.y0   << " " << fc.chain << endl; 
    }

  }
  
    
	
}

