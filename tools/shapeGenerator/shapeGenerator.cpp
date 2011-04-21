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
 * @file visuDistanceTransform.cpp
 * @ingroup Tools
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr)
 * LIRIS (CNRS, UMR 5205), 
 *
 * @date 2011/01/04
 *
 * DGtal shape generator
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"

#include "DGtal/helpers/ShapeFactory.h"
#include "DGtal/helpers/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/io/colormaps/GrayScaleColorMap.h"
#include "DGtal/kernel/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/kernel/imagesSetsUtils/SetFromImage.h"
#include "DGtal/kernel/images/ImageContainerBySTLVector.h"

#include "DGtal/io/writers/PNMWriter.h"
#include "DGtal/io/writers/RawWriter.h"
#include "DGtal/io/writers/VolWriter.h"


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <vector>
#include <string>

using namespace DGtal;

std::vector<std::string> shapesND;
std::vector<std::string> shapesDesc;
std::vector<std::string> shapesParam1;
std::vector<std::string> shapesParam2;
std::vector<std::string> shapesParam3;

void createList()
{
  shapesND.push_back("ball");
  shapesDesc.push_back("Ball for the Euclidean metric.");
  shapesParam1.push_back("--radius");
  shapesParam2.push_back("");
  shapesParam3.push_back("");
 
  shapesND.push_back("cube");
  shapesDesc.push_back("Hypercube in nD.");
  shapesParam1.push_back("--width");
  shapesParam2.push_back("");
  shapesParam3.push_back("");

  shapesND.push_back("lpball");
  shapesDesc.push_back("Ball for the l_power metric in nD.");
  shapesParam1.push_back("--radius");
  shapesParam2.push_back("--power");
  shapesParam3.push_back("");

}

void displayList()
{
  trace.emphase()<<"nD Shapes:"<<std::endl;
  for(unsigned int i=0; i<shapesND.size(); ++i)
    trace.info()<<"\t"<<shapesND[i]<<"\t"
		<<shapesDesc[i]<<std::endl
		<<"\t\tparameter(s): "
		<< shapesParam1[i]<<" "
      		<< shapesParam2[i]<<" "
      		<< shapesParam3[i]<<std::endl;
  
}



unsigned int checkAndRetrunIndex(const std::string &shapeName)
{
  unsigned int pos=0;
  
  while ((pos < shapesND.size()) && (shapesND[pos] != shapeName))
    pos++;
  
  if (pos == shapesND.size())
    {
      trace.error() << "The specified shape has not found.";
      trace.info()<<std::endl;
      exit(1);
    }
  
  return pos;
}

template <typename Set, typename Image>
struct Exporter
{ 
  typedef GrayscaleColorMap<unsigned char> Gray;

  
  static
  void save(const Set &aSet, 
	    const std::string outputName, 
	    const std::string outputFormat)
  {
    
    Image  image = ImageFromSet<Image>::template create<Set>(aSet, 255);
    
    if (outputFormat == "pgm")
      PNMWriter<Image,Gray>::exportPGM(outputName+"."+outputFormat,image,0,255);
    else
      {
	trace.error()<< "Output format: "<<outputFormat<< " not recognized."<<std::endl;
	exit(1);
      }
  }
};

void missingParam(std::string param)
{
  trace.error() <<" Parameter: "<<param<<" is required..";
  trace.info()<<std::endl;
  exit(1);
}

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

int main( int argc, char** argv )
{
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("dimension,d", po::value<unsigned int>()->default_value(2), "Dimension of the shape {2,3}") 
    ("shape,s", po::value<std::string>(), "Shape type")
    ("list,l",  "List all available shapes")
    ("radius,r",  po::value<unsigned int>()->default_value(10), "Radius of the shape" )
    ("width,w",  po::value<unsigned int>()->default_value(10), "Width of the shape" )
    ("power,p",   po::value<double>()->default_value(2.0), "Power of the shape" )
    ("output,o", po::value<string>(), "Basename of the output file")
    ("format,f",   po::value<string>(), "Output format {pgm, pgm3d, raw, vol, svg}" );
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "Generate shapes using DGtal library" << "Usage: " <<  " \n"
		  << general_opt << "\n";
      return 0;
    }
  
  //List creation
  createList();
  
  if (vm.count("list"))
    {
      displayList();
      return 0;
    }

  
  std::string shapeName = vm["shape"].as<std::string>();
  
  if (not(vm.count("output"))) missingParam("--output");
  std::string outputName = vm["output"].as<std::string>();
 
  if (not(vm.count("format"))) missingParam("--format");
  std::string outputFormat = vm["format"].as<std::string>();
  
  unsigned int dimension = vm["dimension"].as<unsigned int>();
  //unsigned int power = vm["power"].as<unsigned int>();
  //unsigned int width = vm["width"].as<unsigned int>();
  
  unsigned int id = checkAndRetrunIndex(shapeName);
  
  if (dimension == 2)
    {
      if (id ==0)
	{
	  if (not(vm.count("radius"))) missingParam("--radius");

	  unsigned int radius = vm["radius"].as<unsigned int>();
	  typedef ImageContainerBySTLVector<Z2i::Domain,unsigned char> Image;
  
	  ImplicitBall<Z2i::Space> ball(Z2i::Point(0,0), radius);
	  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
	  
	  Shapes<Z2i::Domain>::shaper(aSet, ball);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	  return 0;
	}
      if (id ==1)
	{
	  if (not(vm.count("width"))) missingParam("--width");

	  unsigned int width = vm["width"].as<unsigned int>();
	  typedef ImageContainerBySTLVector<Z2i::Domain,unsigned char> Image;
  
	  ImplicitHyperCube<Z2i::Space> object(Z2i::Point(0,0), width/2);
	  Z2i::Domain domain(object.getLowerBound(), object.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
	  
	  Shapes<Z2i::Domain>::shaper(aSet, object);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	  return 0;
	}
      if (id ==2)
	{
	  if (not(vm.count("power"))) missingParam("--power");
	  if (not(vm.count("radius"))) missingParam("--radius");

	  unsigned int radius = vm["radius"].as<unsigned int>();
	  unsigned int power = vm["power"].as<double>();
	  typedef ImageContainerBySTLVector<Z2i::Domain,unsigned char> Image;
  
	  ImplicitRoundedHyperCube<Z2i::Space> ball(Z2i::Point(0,0), radius, power);
	  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
	  
	  Shapes<Z2i::Domain>::shaper(aSet, ball);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	  return 0;
	}
    }
}
