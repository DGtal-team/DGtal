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
 * @file exampleFMM3D.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/02/23
 *
 * @brief The aim of this example
 * is to use the FMM (fast marching method) class
 * in order to incrementally compute a signed distance
 * field from a digital surface. The resulting field
 * is visualized with QGLViewer  
 *
 * This file is part of the DGtal library.
 */

#include <iostream>
#include <QtGui/qapplication.h>
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/io/Color.h"
#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/GradientColorMap.h"

using namespace std;
using namespace DGtal;
using namespace Z3i;


/////////////////////
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

namespace po = boost::program_options;

//image
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/kernel/BasicPointPredicates.h"

//frontier
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/helpers/FrontierPredicate.h"
#include "DGtal/topology/LightExplicitDigitalSurface.h"

// FMM
#include "DGtal/geometry/volumes/distance/FMM.h"

// Standard services - public :
int main( int argc, char** argv )
{

  //////////////////////////////////////////////////////////////////////////////////
  // parse command line 
  po::options_description general_opt("Allowed options are");
  general_opt.add_options()
    ("help,h", "display this message")
    ("inputImage,i",  po::value<string>(), "Grey-level image (vol format)" )
    ("threshold,t",  po::value<int>()->default_value(1), "Set defined by the voxels whose value is below t" )
    ("width,w",  po::value<double>()->default_value(3.0), "Band width where DT is computed" ); 
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      trace.info()<< "3d incremental DT transform" << std::endl
		  << "Basic usage: "<<std::endl
		  << argv[0] << " [other options] -i <vol file> -t <threshold> " << std::endl
		  << general_opt << "\n";
      return 0;
    }
  
  //Parse options
  //threshold
  int t = vm["threshold"].as<int>(); 
  //width
  double maximalWidth = vm["width"].as<double>(); 


  //////////////////////////////////////////////////////////////////////////////////
  // image binarization and surface extraction
  //types
  typedef ImageContainerBySTLVector<Domain,short int> LabelImage; 
  typedef ConstImageAdapter<LabelImage, Domain, DefaultFunctor, bool, Thresholder<LabelImage::Value> > BinaryImage;
  typedef FrontierPredicate<KSpace, BinaryImage> SurfelPredicate;
  typedef LightExplicitDigitalSurface<KSpace, SurfelPredicate> Frontier;
  typedef Frontier::SurfelConstIterator SurfelIterator;

  if (!(vm.count("inputImage"))) 
    {
      trace.emphase() << "you should use option -i" << std::endl; 
      return 0; 
    }

  //reading image
  string imageFileName = vm["inputImage"].as<std::string>();
  trace.emphase() << imageFileName <<std::endl; 
  DGtal::trace.beginBlock("image reading..."); 
  LabelImage labelImage = VolReader<LabelImage>::importVol( imageFileName);
  DGtal::trace.endBlock(); 

  DGtal::trace.beginBlock("binarization..."); 

  DefaultFunctor g;
  Thresholder<LabelImage::Value> thresholder( t ); 
  BinaryImage binaryImage(labelImage, labelImage.domain(), g, thresholder);
  trace.info() << "threshold: "
	       << t
	       << std::endl;

  //space and starting bel
  KSpace ks;
  Domain d = labelImage.domain(); 
  ks.init( d.lowerBound(), d.upperBound(), true ); 
  KSpace::SCell bel;

  try { 
    //getting a bel
    bel = Surfaces<KSpace>::findABel( ks, binaryImage, d.size() );

    trace.info() << "starting bel: "
		 << bel
		 << std::endl;
 
  } catch (DGtal::InputException i) {
    trace.emphase() << "starting bel not found" << std::endl; 
    return 0; 
  }

  //implicit frontier 
  SCellToIncidentPoints<KSpace> functor( ks );
  std::pair<Point,Point> bpair = functor(bel);    
  SurfelPredicate surfelPredicate( ks, binaryImage, 
				   binaryImage( bpair.second ), 
				   binaryImage( bpair.first ) );  
  Frontier frontier( ks, surfelPredicate, 
		     SurfelAdjacency<KSpace::dimension>( true ), bel ); 

  DGtal::trace.endBlock();

  //////////////////////////////////////////////////////////////////////////////////
  /// FMM types
  typedef ImageContainerBySTLMap<Domain,double> DistanceImage; 
  typedef DigitalSetFromMap<DistanceImage> PointSet; 
  //! [FMMDef]
  typedef FMM<DistanceImage, PointSet, Domain::Predicate > FMM;
  //! [FMMDef]

  DGtal::trace.beginBlock("FMM..."); 

  /// FMM init
  //! [FMMInit]
  DistanceImage image( d, 0.0 );
  PointSet points(image);
  FMM::initFromBelsRange( ks, frontier.begin(), frontier.end(), image, points, 0.5 ); 
  //! [FMMInit]

  /// FMM main
  //! [FMMUsage]
  FMM fmm( image, points, d.predicate(), d.size(), maximalWidth );
  fmm.compute(); 
  trace.info() << fmm << std::endl;  
  //! [FMMUsage]

  DGtal::trace.endBlock();

  //////////////////////////////////////////////////////////////////////////////////
  //visualisation
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();

  //
  GradientColorMap<double> colorMap( 0, 2*maximalWidth );
  colorMap.addColor( Color( 255, 0, 0 ) );
  colorMap.addColor( Color( 0, 250, 0 ) );
  for (DistanceImage::const_iterator it = image.begin(), itEnd = image.end(); 
       it != itEnd; ++it)
    {
      Point p = it->first;
      viewer << CustomColors3D( colorMap(it->second), colorMap(it->second) ) ;
      viewer << p;
    }

  Vector extent = d.extent(); 
  double a = -extent[0]/2, b = extent[1]/2;
  double c = 0, mu = (a+b);  
  trace.info() << "clipping plane (" 
	       << a << ", " << b << ", " << c << ", " << mu << ")" 
	       << std::endl;  
  viewer << ClippingPlane(a,b,c,mu); 
  
  viewer << Viewer3D::updateDisplay;
  return application.exec();
}
//                                                                           //
