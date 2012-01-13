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
 * @file estimatorComparator.cpp
 * @ingroup Tools
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr)
 * LIRIS (CNRS, UMR 5205), 
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) 
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France

 * @date 2011/07/04
 *
 * DGtal shape generator
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <iomanip>
#include <vector>
#include <string>

#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/base/Common.h"
#include "DGtal/base/Clock.h"

//shapes
#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"

//Digitizer
#include "DGtal/shapes/GaussDigitizer.h"
#include "DGtal/geometry/curves/representation/GridCurve.h"


//Estimators
#include "DGtal/geometry/curves/estimation/TrueLocalEstimatorOnPoints.h"
#include "DGtal/geometry/curves/estimation/TrueGlobalEstimatorOnPoints.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeCurvatureFunctor.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeTangentFunctor.h"
#include "DGtal/geometry/curves/estimation/ParametricShapeArcLengthFunctor.h"

#include "DGtal/geometry/curves/evolution/BinomialConvolver.h"
#include "DGtal/geometry/curves/estimation/MostCenteredMaximalSegmentEstimator.h"
#include "DGtal/geometry/curves/estimation/SegmentComputerFunctor.h"
#include "DGtal/geometry/curves/representation/ArithmeticalDSS.h"

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
  shapesDesc.push_back("Flower with k petals with radius ranging from R+/-v.");
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
unsigned int checkAndReturnIndex(const std::string &shapeName)
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



template <typename Estimator, typename ConstIterator>
std::vector<typename Estimator::Quantity>
estimateQuantity( Estimator & estimator, 
      ConstIterator it, ConstIterator it_end )
{
  std::vector<typename Estimator::Quantity> values;
  for ( ; it != it_end; ++it )
    {
      values.push_back( estimator.eval( it ) );
    }
  return values;
}

template <typename Space, typename Shape>
bool
compareShapeEstimators( const string & name,
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
  typedef typename PointsRange::ConstIterator ConstIteratorOnPoints;


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
      std::cerr << "[compareShapeEstimators]"
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
    PointsRange r = gridcurve.getPointsRange(); 
    std::cout << "# range size = " << r.size() << std::endl;  

    // Estimations
    // True values
    std::cout << "# True values computation" << std::endl;  
    typedef ParametricShapeTangentFunctor< Shape > TangentFunctor;
    typedef ParametricShapeCurvatureFunctor< Shape > CurvatureFunctor;
  
    TrueLocalEstimatorOnPoints< ConstIteratorOnPoints, Shape, TangentFunctor >  
      trueTangentEstimator;
    TrueLocalEstimatorOnPoints< ConstIteratorOnPoints, Shape, CurvatureFunctor >  
      trueCurvatureEstimator;
  
    trueTangentEstimator.init( h, r.begin(), r.end(), &aShape, gridcurve.isClosed());
    std::vector<RealPoint> trueTangents = 
      estimateQuantity( trueTangentEstimator, r.begin(), r.end() );
    trueCurvatureEstimator.init( h, r.begin(), r.end(), &aShape, gridcurve.isClosed());
    std::vector<double> trueCurvatures = 
      estimateQuantity( trueCurvatureEstimator, r.begin(), r.end() );
  
    // Maximal Segments
    std::cout << "# Maximal DSS tangent estimation" << std::endl;  
    typedef ArithmeticalDSS<ConstIteratorOnPoints,Integer,4> SegmentComputer;
    typedef TangentFromDSSFunctor<SegmentComputer> SCFunctor;
    SegmentComputer sc;
    SCFunctor f; 
    MostCenteredMaximalSegmentEstimator<SegmentComputer,SCFunctor> MSTangentEstimator(sc, f); 
   
    Clock c;
    
    c.startClock();
    MSTangentEstimator.init( h, r.begin(), r.end(), gridcurve.isClosed() );
    std::vector<typename SCFunctor::Value> MSTangents = 
      estimateQuantity( MSTangentEstimator, r.begin(), r.end() );
    double TMST = c.stopClock();


    // Binomial
    std::cout << "# Tangent and curvature estimation from binomial convolution" << std::endl;
    typedef BinomialConvolver<ConstIteratorOnPoints, double> MyBinomialConvolver;
    std::cout << "# mask size = " << 
      MyBinomialConvolver::suggestedSize( h, r.begin(), r.end() ) << std::endl;
    typedef TangentFromBinomialConvolverFunctor< MyBinomialConvolver, RealPoint >
      TangentBCFct;
    typedef CurvatureFromBinomialConvolverFunctor< MyBinomialConvolver, double >
      CurvatureBCFct;
    BinomialConvolverEstimator< MyBinomialConvolver, TangentBCFct> BCTangentEstimator;
    BinomialConvolverEstimator< MyBinomialConvolver, CurvatureBCFct> BCCurvatureEstimator;
    
    c.startClock();
    BCTangentEstimator.init( h, r.begin(), r.end(), gridcurve.isClosed() );
    std::vector<RealPoint> BCTangents = 
      estimateQuantity( BCTangentEstimator, r.begin(), r.end() );
    double TBCTan = c.stopClock();

    c.startClock();
    BCCurvatureEstimator.init( h, r.begin(), r.end(), gridcurve.isClosed() );
    std::vector<double> BCCurvatures =
      estimateQuantity( BCCurvatureEstimator, r.begin(), r.end() );
    double TBCCurv = c.stopClock();

    // Output
    std::cout << "# Shape = "<< name <<std::endl
        << "# Time-BCtangent = "<<TBCTan <<std::endl
        << "# Time-BCcurvature = "<<TBCCurv<<std::endl
        << "# Time-MStangent = "<<TMST<<std::endl
        << "# id x y tangentx tangenty curvature"
        << " BCtangentx BCtangenty BCcurvature"
        << " MStangentx MStangenty"
        << std::endl;  
    unsigned int i = 0;
    for ( ConstIteratorOnPoints it = r.begin(), it_end = r.end();
    it != it_end; ++it, ++i )
      {
  Point p = *it;
  std::cout << i << setprecision( 15 )
      << " " << p[ 0 ] << " " << p[ 1 ] 
      << " " << trueTangents[ i ][ 0 ]
      << " " << trueTangents[ i ][ 1 ]
      << " " << trueCurvatures[ i ]
      << " " << BCTangents[ i ][ 0 ]
      << " " << BCTangents[ i ][ 1 ]
      << " " << BCCurvatures[ i ]
      << " " << MSTangents[ i ][ 0 ]
      << " " << MSTangents[ i ][ 1 ]
      << std::endl;
      }
    return true;
  }    
  catch ( InputException e )
    {
      std::cerr << "[compareShapeEstimators]"
    << " error in finding a bel." << std::endl;
      return false;
    }
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
    ("center_x,x",   po::value<double>()->default_value(0.0), "x-coordinate of the shape center (double)" )
    ("center_y,y",   po::value<double>()->default_value(0.0), "y-coordinate of the shape center (double)" )
    ("gridstep,g",  po::value<double>()->default_value(1.0), "Gridstep for the digitization" );

  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "Compare local estimators on implicit shapes using DGtal library" <<std::endl << "Basic usage: "<<std::endl
      << "\testimatorComparator [options] --shape <shapeName> --output <outputBasename>"<<std::endl
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
    
 
    
  //We check that the shape is known
  unsigned int id = checkAndReturnIndex(shapeName);

  // standard types
  typedef Z2i::Space Space;
  typedef Space::Point Point;
  typedef Space::RealPoint RealPoint;

  RealPoint center( vm["center_x"].as<double>(),
        vm["center_y"].as<double>() );
  double h = vm["gridstep"].as<double>();
  if (id ==0)
    {
      if (!(vm.count("radius"))) missingParam("--radius");
      double radius = vm["radius"].as<double>();
      Ball2D<Space> ball(Z2i::Point(0,0), radius);
      compareShapeEstimators<Space>( "Ball", ball, h ); 
    }
  else if (id ==1)
    {
      if (!(vm.count("width"))) missingParam("--width");
      double width = vm["width"].as<double>();
      ImplicitHyperCube<Space> object(Z2i::Point(0,0), width/2);
      trace.error()<< "Not available.";
      trace.info()<<std::endl;
    }
  else if (id ==2)
    {
      if (!(vm.count("power"))) missingParam("--power");
      if (!(vm.count("radius"))) missingParam("--radius");
      double radius = vm["radius"].as<double>();
      double power = vm["power"].as<double>();
      ImplicitRoundedHyperCube<Space> ball(Z2i::Point(0,0), radius, power);
      trace.error()<< "Not available.";
      trace.info()<<std::endl;
    }
  else if (id ==3)
    {
      if (!(vm.count("varsmallradius"))) missingParam("--varsmallradius");
      if (!(vm.count("radius"))) missingParam("--radius");
      if (!(vm.count("k"))) missingParam("--k");
      if (!(vm.count("phi"))) missingParam("--phi");
      double radius = vm["radius"].as<double>();
      double varsmallradius = vm["varsmallradius"].as<double>();
      unsigned int k = vm["k"].as<unsigned int>();
      double phi = vm["phi"].as<double>();
      Flower2D<Space> flower( center, radius, varsmallradius, k, phi );
      compareShapeEstimators<Space>( "Flower", flower, h ); 
    }
  else if (id ==4)
    {
      if (!(vm.count("radius"))) missingParam("--radius");
      if (!(vm.count("k"))) missingParam("--k");
      if (!(vm.count("phi"))) missingParam("--phi");
      double radius = vm["radius"].as<double>();
      unsigned int k = vm["k"].as<unsigned int>();
      double phi = vm["phi"].as<double>();
      NGon2D<Space> object( center, radius, k, phi );
      compareShapeEstimators<Space>( "NGon", object, h ); 
    }
  else if (id ==5)
    {
      if (!(vm.count("varsmallradius"))) missingParam("--varsmallradius");
      if (!(vm.count("radius"))) missingParam("--radius");
      if (!(vm.count("k"))) missingParam("--k");
      if (!(vm.count("phi"))) missingParam("--phi");
      double radius = vm["radius"].as<double>();
      double varsmallradius = vm["varsmallradius"].as<double>();
      unsigned int k = vm["k"].as<unsigned int>();
      double phi = vm["phi"].as<double>();
      AccFlower2D<Space> accflower( center, radius, varsmallradius, k, phi );
      compareShapeEstimators<Space>( "AccFlower", accflower, h ); 
    } 
  else if (id ==6)
    {
      if (!(vm.count("axis1"))) missingParam("--axis1");
      if (!(vm.count("axis2"))) missingParam("--axis2");
      if (!(vm.count("phi"))) missingParam("--phi");
      double a1 = vm["axis1"].as<double>();
      double a2 = vm["axis2"].as<double>();
      double phi = vm["phi"].as<double>();
      Ellipse2D<Space> ellipse( center, a1, a2, phi );
      compareShapeEstimators<Space>( "Ellipse", ellipse, h ); 
    } 
}
