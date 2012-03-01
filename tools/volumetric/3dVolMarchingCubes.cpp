/**
 * @file 3dVolMarchingCubes.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2012/02/29
 *
 * An example file named 3dVolMarchingCubes.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
//! [3dVolMarchingCubes-basicIncludes]
#include <iostream>
#include <queue>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>
#include <QImageReader>
#include <QtGui/qapplication.h>
#include "DGtal/kernel/sets/SetPredicate.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/ImageLinearCellEmbedder.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/shapes/CanonicEmbedder.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/helpers/Surfaces.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/topology/SetOfSurfels.h"
//! [3dVolMarchingCubes-basicIncludes]

///////////////////////////////////////////////////////////////////////////////

using namespace DGtal;
using namespace Z3i;
namespace po = boost::program_options;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  //! [3dVolMarchingCubes-parseCommandLine]
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("input-file,i", po::value<std::string>(), "the volume file (.vol)" )
    ("threshold,t",  po::value<unsigned int>()->default_value(1), "the value that defines the isosurface in the image (an integer between 0 and 255)." ) 
    ("adjacency,a",  po::value<unsigned int>()->default_value(0), "0: interior adjacency, 1: exterior adjacency")
    ("output-file,o",  po::value<std::string>()->default_value( "marching-cubes.off" ), "the output OFF file that represents the geometry of the isosurface") ; 
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if ( vm.count("help") || ( argc <= 1 ) )
    {
      std::cout << "Usage: " << argv[0] 
                << " [-i <fileName.vol>] [-t <threshold>] [-a <adjacency>] [-o <output.off>]" << std::endl
                << "Outputs the isosurface of value <threshold> of the volume <fileName.vol>  as an OFF file <output.off>. The <adjacency> (0/1) allows to choose between interior (6,18) and exterior (18,6) adjacency." << std::endl
                << general_opt << std::endl;
      return 0;
    }
  if ( ! vm.count("input-file") )
    {
      trace.error() << "The input file name was defined." << std::endl;      
      return 1;
    }
  string inputFilename = vm["input-file"].as<std::string>();
  unsigned int threshold = vm["threshold"].as<unsigned int>();
  bool intAdjacency = ( vm["adjacency"].as<unsigned int>() == 0 );
  string outputFilename = vm["output-file"].as<std::string>();
  //! [3dVolMarchingCubes-parseCommandLine]

  //! [3dVolMarchingCubes-readVol]
  trace.beginBlock( "Reading vol file into an image." );
  typedef ImageSelector < Domain, int>::Type Image;
  Image image = VolReader<Image>::importVol(inputFilename);
  DigitalSet set3d (image.domain());
  SetPredicate<DigitalSet> set3dPredicate( set3d );
  SetFromImage<DigitalSet>::append<Image>(set3d, image, 
                                          threshold, 255 );
  trace.endBlock();
  //! [3dVolMarchingCubes-readVol]
  
  //! [3dVolMarchingCubes-KSpace]
  trace.beginBlock( "Construct the Khalimsky space from the image domain." );
  KSpace ks;
  bool space_ok = ks.init( image.domain().lowerBound(), 
                           image.domain().upperBound(), true );
  if (!space_ok)
    {
      trace.error() << "Error in the Khamisky space construction."<<std::endl;
      return 2;
    }
  trace.endBlock();
  //! [3dVolMarchingCubes-KSpace]

  //! [3dVolMarchingCubes-SurfelAdjacency]
  typedef SurfelAdjacency<KSpace::dimension> MySurfelAdjacency;
  MySurfelAdjacency surfAdj( intAdjacency ); // interior in all directions.
  //! [3dVolMarchingCubes-SurfelAdjacency]

  //! [3dVolMarchingCubes-ExtractingSurface]
  trace.beginBlock( "Extracting boundary by scanning the space. " );
  typedef KSpace::SurfelSet SurfelSet;
  typedef SetOfSurfels< KSpace, SurfelSet > MySetOfSurfels;
  typedef DigitalSurface< MySetOfSurfels > MyDigitalSurface;
  MySetOfSurfels theSetOfSurfels( ks, surfAdj );
  Surfaces<KSpace>::sMakeBoundary( theSetOfSurfels.surfelSet(),
                                   ks, set3dPredicate,
                                   image.domain().lowerBound(), 
                                   image.domain().upperBound() );
  MyDigitalSurface digSurf( theSetOfSurfels );
  trace.info() << "Digital surface has " << digSurf.size() << " surfels."
               << std::endl;
  trace.endBlock();
  //! [3dVolMarchingCubes-ExtractingSurface]

  //! [3dVolMarchingCubes-makingOFF]
  trace.beginBlock( "Making OFF surface. " );
  typedef CanonicEmbedder< Space > MyEmbedder;
  typedef 
    ImageLinearCellEmbedder< KSpace, Image, MyEmbedder > CellEmbedder;
  CellEmbedder cellEmbedder;
  MyEmbedder trivialEmbedder;
  cellEmbedder.init( ks, image, trivialEmbedder, threshold );
  ofstream out( outputFilename.c_str() );
  if ( out.good() )
    digSurf.exportEmbeddedSurfaceAs3DOFF( out, cellEmbedder );
  out.close();
  trace.endBlock();
  //! [3dVolMarchingCubes-makingOFF]
  return 0;
}

