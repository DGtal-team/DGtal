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
#include "DGtal/io/DGtalBoard.h"


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <vector>
#include <string>

using namespace DGtal;

std::vector<std::string> shapes2D;
std::vector<std::string> shapesDesc;
std::vector<std::string> shapesParam1;
std::vector<std::string> shapesParam2;
std::vector<std::string> shapesParam3;
std::vector<std::string> shapesParam4;


/** 
 * Create the static list of shapes.
 * 
 */
void createList()
{
  shapes2D.push_back("ball");
  shapesDesc.push_back("Ball for the Euclidean metric.");
  shapesParam1.push_back("--radius");
  shapesParam2.push_back("");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
 
  shapes2D.push_back("cube");
  shapesDesc.push_back("Hypercube.");
  shapesParam1.push_back("--width");
  shapesParam2.push_back("");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
  
  shapes2D.push_back("lpball");
  shapesDesc.push_back("Ball for the l_power metric.");
  shapesParam1.push_back("--radius");
  shapesParam2.push_back("--power");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
  
  shapes2D.push_back("flower");
  shapesDesc.push_back("Flower with k petals.");
  shapesParam1.push_back("--radius");
  shapesParam2.push_back("--smallradius");
  shapesParam3.push_back("--k");
  shapesParam4.push_back("--phi");


}

/** 
 * Display the shape list with parameters.
 * 
 */
void displayList()
{
  trace.emphase()<<"2D Shapes:"<<std::endl;
  for(unsigned int i=0; i<shapes2D.size(); ++i)
    trace.info()<<"\t"<<shapes2D[i]<<"\t"
		<<shapesDesc[i]<<std::endl
		<<"\t\tRequired parameter(s): "
		<< shapesParam1[i]<<" "
      		<< shapesParam2[i]<<" "
      		<< shapesParam3[i]<<" "
      		<< shapesParam4[i]<<std::endl;
  
}


/** 
 * Check if a given shape is available. If not, we exist with an error.
 * If it is, we return the corresponding index in the global vectors.
 * 
 * @param shapeName name of the shape to search.
 * 
 * @return index of the shape in the shape vectors.
 */
unsigned int checkAndRetrunIndex(const std::string &shapeName)
{
  unsigned int pos=0;
  
  while ((pos < shapes2D.size()) && (shapes2D[pos] != shapeName))
    pos++;
  
  if (pos == shapes2D.size())
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
    
    Image  image = ImageFromSet<Image>::template create<Set>(aSet, 255, true);
    
    if (Set::Domain::dimension == 2)
      if  (outputFormat == "pgm")
	PNMWriter<Image,Gray>::exportPGM(outputName+"."+outputFormat,image,0,255);
      else
	if (outputFormat == "raw")
	  RawWriter<Image,Gray>::exportRaw8(outputName+"."+outputFormat,image,0,255);
	else
	  if (outputFormat == "svg")
	    {
	      DGtalBoard board;
	      board << aSet;
	      board.saveSVG((outputName+"."+outputFormat).c_str());
	    }
	  else
	    if (outputFormat == "pdf")
	      {
		DGtalBoard board;
		board << aSet;
		board.saveCairo((outputName+"."+outputFormat).c_str(), DGtalBoard::CairoPDF);
		
	      }
	    else
	      {
		trace.error()<< "Output format: "<<outputFormat<< " not recognized."<<std::endl;
		exit(1);
	      }
  }
};

/** 
 * Missing parameter error message.
 * 
 * @param param 
 */
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
    ("shape,s", po::value<std::string>(), "Shape name")
    ("list,l",  "List all available shapes")
    ("radius,R",  po::value<unsigned int>()->default_value(10), "Radius of the shape" )
    ("smallradius,r",  po::value<unsigned int>()->default_value(5), "Small radius of the shape" )
    ("k,k",  po::value<unsigned int>()->default_value(3), "Number of branches or corners the shape" )
    ("phi",  po::value<double>()->default_value(0.0), "Phase of the shape (in radian)" )
    ("width,w",  po::value<unsigned int>()->default_value(10), "Width of the shape" )
    ("power,p",   po::value<double>()->default_value(2.0), "Power of the metric (double)" )
    ("output,o", po::value<string>(), "Basename of the output file")
    ("format,f",   po::value<string>()->default_value("pgm"), "Output format {pgm, raw, svg, pdf}" );
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "Generate shapes using DGtal library" <<std::endl << "Basic usage: "<<std::endl
		  << "\tshapeGenerator [options] --shape <shapeName> --output <outputBasename>"<<std::endl
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


  //Parse options
  std::string shapeName = vm["shape"].as<std::string>();
  
  if (not(vm.count("output"))) missingParam("--output");
  std::string outputName = vm["output"].as<std::string>();
 
  if (not(vm.count("format"))) missingParam("--format");
  std::string outputFormat = vm["format"].as<std::string>();
    
  //We check that the shape is known
  unsigned int id = checkAndRetrunIndex(shapeName);
  typedef ImageContainerBySTLVector<Z2i::Domain,unsigned char> Image;
  
  if (id ==0)
    {
      if (not(vm.count("radius"))) missingParam("--radius");
      unsigned int radius = vm["radius"].as<unsigned int>();
      
      ImplicitBall<Z2i::Space> ball(Z2i::Point(0,0), radius);
      Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
      Z2i::DigitalSet aSet(domain);
      
      Shapes<Z2i::Domain>::shaper(aSet, ball);
      Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
      return 0;
    }
  else
    if (id ==1)
      {
	if (not(vm.count("width"))) missingParam("--width");
	unsigned int width = vm["width"].as<unsigned int>();
	
	ImplicitHyperCube<Z2i::Space> object(Z2i::Point(0,0), width/2);
	Z2i::Domain domain(object.getLowerBound(), object.getUpperBound());
	Z2i::DigitalSet aSet(domain);
	
	Shapes<Z2i::Domain>::shaper(aSet, object);
	Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	return 0;
	}
    else
      if (id ==2)
	{
	  if (not(vm.count("power"))) missingParam("--power");
	  if (not(vm.count("radius"))) missingParam("--radius");
	  unsigned int radius = vm["radius"].as<unsigned int>();
	  unsigned int power = vm["power"].as<double>();
	  
	  ImplicitRoundedHyperCube<Z2i::Space> ball(Z2i::Point(0,0), radius, power);
	  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
	  
	  Shapes<Z2i::Domain>::shaper(aSet, ball);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	  return 0;
	}
      else
	//if (id ==3)
	{
	  if (not(vm.count("smallradius"))) missingParam("--smallradius");
	  if (not(vm.count("radius"))) missingParam("--radius");
	  if (not(vm.count("k"))) missingParam("--k");
	  if (not(vm.count("phi"))) missingParam("--phi");
	  double radius = vm["radius"].as<unsigned int>();
	  double smallradius = vm["smallradius"].as<unsigned int>();
	  unsigned int k = vm["k"].as<unsigned int>();
	  double phi = vm["power"].as<double>();
	  
	  Flower2D<Z2i::Space> flower(Z2i::Point(0,0), radius, smallradius,k,phi);
	  Z2i::Domain domain(flower.getLowerBound(), flower.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
	  
	  Shapes<Z2i::Domain>::shaper(aSet, flower);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
	  return 0;
	}
}
