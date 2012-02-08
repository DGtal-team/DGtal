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
 * @file shapeGenerator.cpp
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
#include <vector>
#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/base/Common.h"

#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"


#include "DGtal/io/colormaps/GrayscaleColorMap.h"
#include "DGtal/images/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/ImageContainerBySTLVector.h"

#include "DGtal/io/writers/PNMWriter.h"
#include "DGtal/io/writers/RawWriter.h"
#include "DGtal/io/writers/VolWriter.h"
#include "DGtal/io/boards/Board2D.h"


using namespace DGtal;


/**
 * Global vectors to describe the available shapes and their
 * parameters.
 */
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
  shapesParam1.push_back("--radius [-R]");
  shapesParam2.push_back("");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
 
  shapes2D.push_back("square");
  shapesDesc.push_back("square (no signature).");
  shapesParam1.push_back("--width [-w]");
  shapesParam2.push_back("");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
  
  shapes2D.push_back("lpball");
  shapesDesc.push_back("Ball for the l_power metric (no signature).");
  shapesParam1.push_back("--radius [-R],");
  shapesParam2.push_back("--power [-p]");
  shapesParam3.push_back("");
  shapesParam4.push_back("");
  
  shapes2D.push_back("flower");
  shapesDesc.push_back("Flower with k petals.");
  shapesParam1.push_back("--radius [-R],");
  shapesParam2.push_back("--varsmallradius [-v],");
  shapesParam3.push_back("--k [-k],");
  shapesParam4.push_back("--phi");

  shapes2D.push_back("ngon");
  shapesDesc.push_back("Regular k-gon.");
  shapesParam1.push_back("--radius [-R],");
  shapesParam2.push_back("--k [-k],");
  shapesParam3.push_back("--phi");
  shapesParam4.push_back("");
 
  shapes2D.push_back("accflower");
  shapesDesc.push_back("Accelerated Flower with k petals.");
  shapesParam1.push_back("--radius [-R],");
  shapesParam2.push_back("--varsmallradius [-v],");
  shapesParam3.push_back("--k [-k],");
  shapesParam4.push_back("--phi");

  shapes2D.push_back("ellipse");
  shapesDesc.push_back("Ellipse.");
  shapesParam1.push_back("--axis1 [-A],");
  shapesParam2.push_back("--axis2 [-a],");
  shapesParam3.push_back("--phi");
  shapesParam4.push_back("");
 

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


/** 
 * Functor to export a given shape into an image file
 * (pgm,raw,pdf,svg,...) and to extract its signature.
 * 
 * @tparam Set type of the input Set
 * @tparam Image type of the input Image.
 */
template <typename Set, typename Image>
struct Exporter
{ 
  typedef GrayscaleColorMap<unsigned char> Gray;

  
  /** 
   * Export a given Set into an image file.
   * 
   * @param aSet input set.
   * @param outputName output file name.
   * @param outputFormat output file format.
   *
   */
  static
  void save(const Set &aSet, 
	    const std::string outputName, 
	    const std::string outputFormat)
  {
    
    Image  image = ImageFromSet<Image>::template create<Set>(aSet, 255, true);
    
    if  (outputFormat == "pgm")
      PNMWriter<Image,Gray>::exportPGM(outputName+"."+outputFormat,image,0,255);
    else
      if (outputFormat == "raw")
	RawWriter<Image,Gray>::exportRaw8(outputName+"."+outputFormat,image,0,255);
      else
	if (outputFormat == "svg")
	  {
	    Board2D board;
	    board << aSet;
	    board.saveSVG((outputName+"."+outputFormat).c_str());
	  }
	else
#ifdef WITH_CAIRO
	  if (outputFormat == "pdf")
	    {
	      Board2D board;
	      board << aSet;
	      board.saveCairo((outputName+"."+outputFormat).c_str(), Board2D::CairoPDF);
        
	    }
	  else
	    if (outputFormat == "png")
	      {
		Board2D board;
		board << aSet;
		board.saveCairo((outputName+"."+outputFormat).c_str(), Board2D::CairoPNG);
	      }
	    else
#endif
	      {
		trace.error()<< "Output format: "<<outputFormat<< " not recognized."<<std::endl;
		exit(1);
	      }
  }




  

  /** 
   * Compute and export (std::cout) the boundary of the set and export the signature (normal
   * vector, curvature) at each point of the 2D contour.
   * 
   * @param aShape the shape
   * @param aSet input set corresponding to the shape
   * @param aDomain the domain used to construct the set.
   */
  template <typename Shape>
  static 
  void exportSignature(const Shape & aShape, Set &aSet, const Z2i::Domain &aDomain)
  {
    SetPredicate<Set> aSetPredicate( aSet );
    trace.beginBlock("Extracting the boundary");
    Z2i::KSpace ks;
    bool space_ok = ks.init( aDomain.lowerBound(),aDomain.upperBound(), true );
    SurfelAdjacency<2> sAdj( true );

    ASSERT(space_ok);
    trace.info() << aSet << std::endl;
    trace.info() << ks 
		 << ( space_ok ? " Successfully instantiated" : " Error" )
		 << std::endl;

    std::vector< std::vector< Z2i::Point >  >  vectContoursBdryPointels;
    Surfaces<Z2i::KSpace>::extractAllPointContours4C( vectContoursBdryPointels,
						      ks, aSetPredicate, sAdj );  
    trace.endBlock();
    
    ///Export
    std::cout<<"## shapeGenerator signature export"<<std::endl;
    std::cout<<"## shape: "<<aShape<<std::endl;
    std::cout<<"## x\ty\tdx\tdy\tddx\tddy"<<std::endl;
    for(unsigned int i=0; i<vectContoursBdryPointels.size(); i++)
      for(unsigned int j=0 ; j< vectContoursBdryPointels.at(i).size() - 1; j++)
	{
	  Z2i::Space::Point point = (vectContoursBdryPointels.at(i).at(j) 
				     + vectContoursBdryPointels.at(i).at(j+1));
	  Z2i::Space::RealPoint midpoint (point[0]/2.0,point[1]/2.0);

	  Z2i::Space::RealPoint xp,xpp;
	  double t = aShape.parameter(midpoint);
	  xp = aShape.xp( t );
	  xpp = aShape.xpp( t );
    
	  std::cout<< midpoint[0]<<"\t"<<midpoint[1]<<"\t"
		   << xp[0]<<"\t"<<xp[1]<<"\t"
		   << xpp[0]<<"\t"<<xpp[1]<<std::endl;
      
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
    ("radius,R",  po::value<double>(), "Radius of the shape" )
    ("axis1,A",  po::value<double>(), "Half big axis of the shape (ellipse)" )
    ("axis2,a",  po::value<double>(), "Half small axis of the shape (ellipse)" )
    ("smallradius,r",  po::value<double>()->default_value(5), "Small radius of the shape" )
    ("varsmallradius,v",  po::value<double>()->default_value(5), "Variable small radius of the shape" )
    ("k,k",  po::value<unsigned int>()->default_value(3), "Number of branches or corners the shape" )
    ("phi",  po::value<double>()->default_value(0.0), "Phase of the shape (in radian)" )
    ("width,w",  po::value<double>()->default_value(10.0), "Width of the shape" )
    ("power,p",   po::value<double>()->default_value(2.0), "Power of the metric (double)" )
    ("output,o", po::value<string>(), "Basename of the output file")
    ("signature", "Display to the standard output the signature (normal, curvature) at each point of the specified shape contour (middle point of each contour linel)")
    ("format,f",   po::value<string>()->default_value("pgm"), "Output format:\n\t  Bitmap {pgm, raw}\n\t  Vector {svg} (+ {png,pdf} if libCairo installed)" );
  
  
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
  if (!(vm.count("shape"))) missingParam("--shape");
  std::string shapeName = vm["shape"].as<std::string>();
    
 
  if (!(vm.count("output"))) missingParam("--output");
  std::string outputName = vm["output"].as<std::string>();
 
  if (!(vm.count("format"))) missingParam("--format");
  std::string outputFormat = vm["format"].as<std::string>();
    
  //We check that the shape is known
  unsigned int id = checkAndRetrunIndex(shapeName);
  typedef ImageContainerBySTLVector<Z2i::Domain,unsigned char> Image;
  
  if (id ==0)
    {
      if (!(vm.count("radius"))) missingParam("--radius");
      double radius = vm["radius"].as<double>();
      
      Ball2D<Z2i::Space> ball(Z2i::Point(0,0), radius);
      Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
      Z2i::DigitalSet aSet(domain);
      
      Shapes<Z2i::Domain>::euclideanShaper(aSet, ball);
      Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
      
      if (vm.count("signature"))
	Exporter<Z2i::DigitalSet,Image>::exportSignature(ball,aSet,domain);
      
      return 0;
    }
  else
    if (id ==1)
      {
	if (!(vm.count("width"))) missingParam("--width");
	double width = vm["width"].as<double>();
  
	ImplicitHyperCube<Z2i::Space> object(Z2i::Point(0,0), width/2.0);
	Z2i::Domain domain(object.getLowerBound(), object.getUpperBound());
	Z2i::DigitalSet aSet(domain);
  
	Shapes<Z2i::Domain>::euclideanShaper(aSet, object);
	Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
	if (vm.count("signature"))
	  {
	    trace.error()<< "No signature export for this shape.";
	    trace.info()<<std::endl;
	  }
   
	return 0;
      }
    else
      if (id ==2)
	{
	  if (!(vm.count("power"))) missingParam("--power");
	  if (!(vm.count("radius"))) missingParam("--radius");
	  double radius = vm["radius"].as<double>();
	  double power = vm["power"].as<double>();
    
	  ImplicitRoundedHyperCube<Z2i::Space> ball(Z2i::Point(0,0), radius, power);
	  Z2i::Domain domain(ball.getLowerBound(), ball.getUpperBound());
	  Z2i::DigitalSet aSet(domain);
    
	  Shapes<Z2i::Domain>::euclideanShaper(aSet, ball);
	  Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
	  if (vm.count("signature"))
	    {
	      trace.error()<< "No signature export for this shape.";
	      trace.info()<<std::endl;
	    }
   
	  return 0;
	}
      else
	if (id ==3)
	  {
	    if (!(vm.count("varsmallradius"))) missingParam("--varsmallradius");
	    if (!(vm.count("radius"))) missingParam("--radius");
	    if (!(vm.count("k"))) missingParam("--k");
	    if (!(vm.count("phi"))) missingParam("--phi");
	    double radius = vm["radius"].as<double>();
	    double varsmallradius = vm["varsmallradius"].as<double>();
	    unsigned int k = vm["k"].as<unsigned int>();
	    double phi = vm["phi"].as<double>();
      
	    Flower2D<Z2i::Space> flower(Z2i::Point(0,0), radius, varsmallradius,k,phi);
	    Z2i::Domain domain(flower.getLowerBound(), flower.getUpperBound());
	    Z2i::DigitalSet aSet(domain);
      
	    Shapes<Z2i::Domain>::euclideanShaper(aSet, flower);
	    Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
	    if (vm.count("signature"))
	      Exporter<Z2i::DigitalSet,Image>::exportSignature(flower,aSet,domain);
   
	    return 0;
	  }
	else
	  if (id ==4)
	    {
	      if (!(vm.count("radius"))) missingParam("--radius");
	      if (!(vm.count("k"))) missingParam("--k");
	      if (!(vm.count("phi"))) missingParam("--phi");
	      double radius = vm["radius"].as<double>();
	      unsigned int k = vm["k"].as<unsigned int>();
	      double phi = vm["phi"].as<double>();
        
	      NGon2D<Z2i::Space> object(Z2i::Point(0,0), radius,k,phi);
	      Z2i::Domain domain(object.getLowerBound(), object.getUpperBound());
	      Z2i::DigitalSet aSet(domain);
        
	      Shapes<Z2i::Domain>::euclideanShaper(aSet, object);
	      Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
	      if (vm.count("signature"))
		Exporter<Z2i::DigitalSet,Image>::exportSignature(object,aSet,domain);
   
	      return 0;
	    }
	  else
	    if (id ==5)
	      {
		if (!(vm.count("varsmallradius"))) missingParam("--varsmallradius");
		if (!(vm.count("radius"))) missingParam("--radius");
		if (!(vm.count("k"))) missingParam("--k");
		if (!(vm.count("phi"))) missingParam("--phi");
		double radius = vm["radius"].as<double>();
		double varsmallradius = vm["varsmallradius"].as<double>();
		unsigned int k = vm["k"].as<unsigned int>();
		double phi = vm["phi"].as<double>();
        
		AccFlower2D<Z2i::Space> flower(Z2i::Point(0,0), radius, varsmallradius,k,phi);
		Z2i::Domain domain(flower.getLowerBound(), flower.getUpperBound());
		Z2i::DigitalSet aSet(domain);
        
		Shapes<Z2i::Domain>::euclideanShaper(aSet, flower);
		Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
		if (vm.count("signature"))
		  Exporter<Z2i::DigitalSet,Image>::exportSignature(flower,aSet,domain);
   
		return 0;
	      } 
	    else
	      //if (id ==6)
	      {
		if (!(vm.count("axis1"))) missingParam("--axis1");
		if (!(vm.count("axis2"))) missingParam("--axis2");
		if (!(vm.count("phi"))) missingParam("--phi");
		double a1 = vm["axis1"].as<double>();
		double a2 = vm["axis2"].as<double>();
		double phi = vm["phi"].as<double>();
        
		Ellipse2D<Z2i::Space> ell(Z2i::Point(0,0), a1, a2,phi);
		Z2i::Domain domain(ell.getLowerBound(), ell.getUpperBound());
		Z2i::DigitalSet aSet(domain);
        
		Shapes<Z2i::Domain>::euclideanShaper(aSet, ell);
		Exporter<Z2i::DigitalSet,Image>::save(aSet,outputName,outputFormat);
  
		if (vm.count("signature"))
		  Exporter<Z2i::DigitalSet,Image>::exportSignature(ell,aSet,domain);
   
		return 0;
	      } 
}
