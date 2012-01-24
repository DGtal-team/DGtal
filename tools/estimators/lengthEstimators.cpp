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
 * @file LengthEstimator.cpp
 * @ingroup Tools
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) 
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr ) 
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 *
 * @date 2011/07/07
 *
 * DGtal tool for length estimations on implicit shapes
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/base/Common.h"
#include "DGtal/base/Clock.h"

//space / domain
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"

//shape and digitizer
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"

#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/geometry/curves/representation/GridCurve.h"

//estimators
#include "DGtal/geometry/curves/estimation/TrueLocalEstimatorOnPoints.h"
#include "DGtal/geometry/curves/estimation/TrueGlobalEstimatorOnPoints.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeArcLengthFunctor.h"

#include "DGtal/geometry/curves/estimation/L1LengthEstimator.h"
#include "DGtal/geometry/curves/estimation/BLUELocalLengthEstimator.h"
#include "DGtal/geometry/curves/estimation/RosenProffittLocalLengthEstimator.h"
#include "DGtal/geometry/curves/estimation/MLPLengthEstimator.h"
#include "DGtal/geometry/curves/estimation/FPLengthEstimator.h"
#include "DGtal/geometry/curves/estimation/DSSLengthEstimator.h"


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

template <typename Shape, typename Space>
bool
lengthEstimators( const string & /*name*/,
      Shape & aShape, 
      double h )
{
  // Types
  typedef typename Space::Point Point;
  typedef typename Space::Vector Vector;
  typedef typename Space::RealPoint RealPoint;
  typedef typename Space::Integer Integer;
  typedef HyperRectDomain<Space> Domain;
  typedef KhalimskySpaceND<Space::dimension,Integer> KSpace;
  typedef typename KSpace::SCell SCell;
  typedef typename GridCurve<KSpace>::PointsRange PointsRange;
  typedef typename GridCurve<KSpace>::ArrowsRange ArrowsRange;

  // Digitizer
  GaussDigitizer<Space,Shape> dig;  
  dig.attach( aShape ); // attaches the shape.
  Vector vlow(-1,-1); Vector vup(1,1);
  dig.init( aShape.getLowerBound()+vlow, aShape.getUpperBound()+vup, h ); 
  Domain domain = dig.getDomain();

  // Create cellular space
  KSpace K;
  bool ok = K.init( dig.getLowerBound(), dig.getUpperBound(), true );
  if ( ! ok )
    {
      std::cerr << "[lengthEstimators]"
    << " error in creating KSpace." << std::endl;
      return false;
    }
  try {
    // Extracts shape boundary
    SurfelAdjacency<KSpace::dimension> SAdj( true );
    SCell bel = Surfaces<KSpace>::findABel( K, dig, 10000 );
    // Getting the consecutive surfels of the 2D boundary
    std::vector<Point> points;
    Surfaces<KSpace>::track2DBoundaryPoints( points, K, SAdj, dig, bel );
    // Create GridCurve
    GridCurve<KSpace> gridcurve;
    gridcurve.initFromVector( points );
    // Ranges
    ArrowsRange ra = gridcurve.getArrowsRange(); 
    PointsRange rp = gridcurve.getPointsRange(); 


    // Estimations
    typedef typename PointsRange::ConstIterator ConstIteratorOnPoints; 
    typedef ParametricShapeArcLengthFunctor< Shape > Length;
    TrueGlobalEstimatorOnPoints< ConstIteratorOnPoints, Shape, Length  >  trueLengthEstimator;
    trueLengthEstimator.init( h, rp.begin(), rp.end(), &aShape, gridcurve.isClosed());

    L1LengthEstimator< typename ArrowsRange::ConstCirculator > l1length;
    DSSLengthEstimator< typename PointsRange::ConstIterator > DSSlength;
    MLPLengthEstimator< typename PointsRange::ConstIterator > MLPlength;
    FPLengthEstimator< typename PointsRange::ConstIterator > FPlength;
    BLUELocalLengthEstimator< typename ArrowsRange::ConstIterator > BLUElength;
    RosenProffittLocalLengthEstimator< typename ArrowsRange::ConstIterator > RosenProffittlength;
  
    // Output
    double trueValue = trueLengthEstimator.eval();
    double l1, blue, rosen,dss,mlp,fp;
    double Tl1, Tblue, Trosen,Tdss,Tmlp,Tfp;
    
    Clock c;

    //Length evaluation & timing
    c.startClock();
    l1length.init(h, ra.c(), ra.c());
    l1 = l1length.eval();
    Tl1 = c.stopClock();
    
    c.startClock();
    BLUElength.init(h, ra.begin(), ra.end(), gridcurve.isClosed());
    blue = BLUElength.eval();
    Tblue = c.stopClock();
    
    c.startClock();
    RosenProffittlength.init(h, ra.begin(), ra.end(), gridcurve.isClosed());
    rosen = RosenProffittlength.eval();
    Trosen = c.stopClock();
    
    c.startClock();
    DSSlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());
    dss = DSSlength.eval();
    Tdss = c.stopClock();
    
    c.startClock();
    MLPlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());
    mlp = MLPlength.eval();
    Tmlp = c.stopClock();

    c.startClock();;
    FPlength.init(h, rp.begin(), rp.end(), gridcurve.isClosed());
    fp = FPlength.eval();
    Tfp = c.stopClock();

    cout << setprecision( 15 ) << h << " " << rp.size() << " " << trueValue 
   << " " << l1
   << " " << blue
   << " " << rosen
   << " " << dss
   << " " << mlp   
   << " " << fp
         << " " << Tl1
   << " " << Tblue
   << " " << Trosen
   << " " << Tdss
   << " " << Tmlp
   << " " << Tfp     
   << endl;
    return true;
  }    
  catch ( InputException e )
    {
      std::cerr << "[lengthEstimators]"
    << " error in finding a bel." << std::endl;
      return false;
    }
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
    ("hMin",   po::value<double>()->default_value(0.0001), "Minimum value for the grid step h (double)" )
    ("steps",   po::value<int>()->default_value(32), "Number of multigrid steps between 1 and hMin (integer)" );
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "Generate multigrid length estimations of paramteric shapes using DGtal library. It will output length estimations (and timings) using several algorithms for decreasing grid steps." <<std::endl << "Basic usage: "<<std::endl
      << "\tLengthEstimators [options] --shape <shapeName>"<<std::endl
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
  double hMin  = vm["hMin"].as<double>();
  int nbSteps = vm["steps"].as<int>();
    
  //We check that the shape is known
  unsigned int id = checkAndRetrunIndex(shapeName);


///////////////////////////////////
  cout << "#h nbPoints true-length L1 BLUE RosenProffit "
       << "DSS MLP FP Time-L1 Time-BLUE Time-RosenProffitt "
       << "Time-DSS Time-MLP Time-FP" <<std::endl;
  cout << "# timings are given in msec." <<std::endl;
  
  double h = 1; 
  double step = exp( log(hMin) / (double)nbSteps);
  while (h > hMin) {

    if (id ==0) ///ball
      {
        if (!(vm.count("radius"))) missingParam("--radius");
        double radius = vm["radius"].as<double>();
        
        Ball2D<Z2i::Space> ball(Z2i::Point(0,0), radius);
        
        lengthEstimators<Ball2D<Z2i::Space>,Z2i::Space>("ball",ball,h); 
      }
    else
      if (id ==1)
        {
    if (!(vm.count("width"))) missingParam("--width");
    double width = vm["width"].as<double>();
  
    ImplicitHyperCube<Z2i::Space> object(Z2i::Point(0,0), width/2);
  
          trace.error()<< "Not available.";
          trace.info()<<std::endl;
        }
      else
        if (id ==2)
    {
      if (!(vm.count("power"))) missingParam("--power");
      if (!(vm.count("radius"))) missingParam("--radius");
      double radius = vm["radius"].as<double>();
      double power = vm["power"].as<double>();
      
      ImplicitRoundedHyperCube<Z2i::Space> ball(Z2i::Point(0,0), radius, power);

          trace.error()<< "Not available.";
          trace.info()<<std::endl;
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

        lengthEstimators<Flower2D<Z2i::Space>,Z2i::Space>("flower",flower,h); 
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

          lengthEstimators<NGon2D<Z2i::Space>,Z2i::Space>("NGon",object,h); 

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
          lengthEstimators<AccFlower2D<Z2i::Space>,Z2i::Space>("accFlower",flower,h); 

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

          lengthEstimators<Ellipse2D<Z2i::Space>,Z2i::Space>("Ellipse",ell,h); 

          } 

    h = h * step;
  }
  return 0;
}
