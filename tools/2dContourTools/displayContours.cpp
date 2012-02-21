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

#include "DGtal/helpers/StdDefs.h"

#include "DGtal/shapes/ShapeFactory.h"
#include "DGtal/shapes/Shapes.h"
#include "DGtal/topology/helpers/Surfaces.h"

//image
#include "DGtal/images/imagesSetsUtils/ImageFromSet.h"
#include "DGtal/images/imagesSetsUtils/SetFromImage.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageSelector.h"
#include "DGtal/io/readers/PointListReader.h"
#include "DGtal/io/boards/Board2D.h"
#include "DGtal/io/Color.h"

#ifdef WITH_MAGICK
#include "DGtal/io/readers/MagickReader.h"
#endif


//contour
#include "DGtal/geometry/curves/representation/FreemanChain.h"

//processing
#include "DGtal/geometry/curves/representation/ArithmeticalDSS.h"
#include "DGtal/geometry/curves/representation/GreedyDecomposition.h"
#include "DGtal/geometry/curves/representation/MaximalSegments.h"
#include "DGtal/geometry/curves/representation/FP.h"

//boost
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

//STL
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
    ("FreemanChain,f", po::value<std::string>(), "FreemanChain file name")
    ("SDP", po::value<std::string>(), "Import a contour as a Sequence of Discrete Points (SDP format)")
    ("SFP", po::value<std::string>(), "Import a contour as a Sequence of Floating Points (SFP format)")
    ("drawContourPoint", po::value<double>(), "<size> display contour points as disk of radius <size>")    
    ("lineWidth", po::value<double>()->default_value(1.0), "Define the linewidth of the contour (SDP format)") 
    ("withProcessing", po::value<std::string>(), "Processing (used only with --FreemanChain):\n\t DSS segmentation {DSS}\n\t  Maximal segments {MS}\n\t Faithful Polygon {FP}\n\t Minimum Length Polygon {MLP}")   
    ("outputEPS", po::value<std::string>(), " <filename> specify eps format (default format output.eps)")
    ("outputSVG", po::value<std::string>(), " <filename> specify svg format.")
    ("outputFIG", po::value<std::string>(), " <filename> specify fig format.")
#ifdef WITH_CAIRO
    ("outputPDF", po::value<std::string>(), "outputPDF <filename> specify pdf format. ")
    ("outputPNG", po::value<std::string>(), "outputPNG <filename> specify png format.")
    ("invertYaxis", " invertYaxis invert the Y axis for display contours (used only with --SDP)")
#endif
#ifdef WITH_MAGICK
    ("backgroundImage", po::value<std::string>(), "backgroundImage <filename> <alpha> : display image as background with transparency alpha (defaut 1) (transparency works only if cairo is available)")
    ("alphaBG", po::value<double>(), "alphaBG <value> 0-1.0 to display the background image in transparency (default 1.0)")
#endif
    ("scale", po::value<double>(), "scale <value> 1: normal; >1 : larger ; <1 lower resolutions  )");
  
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1 || (!(vm.count("FreemanChain")) && !(vm.count("SDP")) && !(vm.count("SFP"))&&
           !(vm.count("backgroundImage")) ) )
    {
      trace.info()<< "Display discrete contours. " <<std::endl << "Basic usage: "<<std::endl
      << "\t displayContours [options] --FreemanChain  <fileName>  --imageName image.png "<<std::endl
      << general_opt << "\n";
      return 0;
    }
  
  
  
  double lineWidth=  vm["lineWidth"].as<double>();
  
  double scale=1.0;
  if(vm.count("scale")){
    scale = vm["scale"].as<double>();
  }
  
  Board2D aBoard;
  aBoard.setUnit (0.05*scale, LibBoard::Board::UCentimeter);
  



#ifdef WITH_MAGICK
  double alpha=1.0;
  if(vm.count("alphaBG")){
    alpha = vm["alphaBG"].as<double>(); 
  }
  
  if(vm.count("backgroundImage")){
    string imageName = vm["backgroundImage"].as<string>();
    typedef ImageSelector<Z2i::Domain, unsigned char>::Type Image;
    DGtal::MagickReader<Image> reader;
    Image img = reader.importImage( imageName );
    Z2i::Point ptInf = img.domain().lowerBound(); 
    Z2i::Point ptSup = img.domain().upperBound(); 
    unsigned int width = abs(ptSup.at(0)-ptInf.at(0)+1);
    unsigned int height = abs(ptSup.at(1)-ptInf.at(1)+1);
    
    aBoard.drawImage(imageName, 0-0.5,height-0.5, width, height, -1, alpha );
  }
#endif
 

 
  if(vm.count("FreemanChain")){
    string fileName = vm["FreemanChain"].as<string>();
    vector< FreemanChain<int> > vectFc =  PointListReader< Z2i::Point>:: getFreemanChainsFromFile<int> (fileName); 
    //aBoard <<  SetMode( vectFc.at(0).className(), "InterGrid" );
    aBoard << CustomStyle( vectFc.at(0).className(), 
         new CustomColors( Color::Red  ,  Color::None ) );    
    for(unsigned int i=0; i<vectFc.size(); i++){
      aBoard <<  vectFc.at(i) ;

      if(vm.count("withProcessing")){
  std::string processingName = vm["withProcessing"].as<std::string>();

  vector<Z2i::Point> vPts(vectFc.at(i).size()+1); 
  copy ( vectFc.at(i).begin(), vectFc.at(i).end(), vPts.begin() ); 
  bool isClosed;
  if ( vPts.at(0) == vPts.at(vPts.size()-1) ) { 
          isClosed = true;
          vPts.pop_back(); 
  } else isClosed = false;

  if (processingName == "DSS") {

          typedef ArithmeticalDSS<vector<Z2i::Point>::iterator,int,4> DSS4;
          typedef deprecated::GreedyDecomposition<DSS4> Decomposition4;

          //Segmentation
    DSS4 computer;
          Decomposition4 theDecomposition( vPts.begin(),vPts.end(),computer,isClosed );
          //for each segment
          aBoard << SetMode( computer.className(), "BoundingBox" );
          string className = computer.className() + "/BoundingBox";
          for ( Decomposition4::SegmentIterator it = theDecomposition.begin();
    it != theDecomposition.end(); ++it ) 
            {
        DSS4 segment(*it);
        aBoard << CustomStyle( className, 
             new CustomPenColor( DGtal::Color::Gray ) ); 
        aBoard << segment; // draw each segment
            } 

  } else if (processingName == "MS") {

          typedef ArithmeticalDSS<vector<Z2i::Point>::iterator,int,4> DSS4;
          typedef deprecated::MaximalSegments<DSS4> Decomposition4;

          //Segmentation
    DSS4 computer;
          Decomposition4 theDecomposition( vPts.begin(),vPts.end(),computer,isClosed );

          //for each segment
          aBoard << SetMode( computer.className(), "BoundingBox" );
          string className = computer.className() + "/BoundingBox";
          for ( Decomposition4::SegmentIterator it = theDecomposition.begin();
    it != theDecomposition.end(); ++it ) 
            {
        DSS4 segment(*it);
        aBoard << CustomStyle( className, 
             new CustomPenColor( DGtal::Color::Black ) ); 
        aBoard << segment; // draw each segment
            } 


  } else if (processingName == "FP") {

    typedef FP<vector<Z2i::Point>::iterator,int,4> FP;
    FP theFP( vPts.begin(),vPts.end(),isClosed );
          aBoard << CustomStyle( theFP.className(), 
         new CustomPenColor( DGtal::Color::Black ) ); 
          aBoard << theFP;


  } else if (processingName == "MLP") {

    typedef FP<vector<Z2i::Point>::iterator,int,4> FP;
    FP theFP( vPts.begin(),vPts.end(),isClosed );

          vector<FP::RealPoint> v( theFP.size() );
          theFP.copyMLP( v.begin() );

          //polyline to draw
    vector<LibBoard::Point> polyline;
    vector<FP::RealPoint>::const_iterator it = v.begin();
    for ( ;it != v.end();++it) {
      FP::RealPoint p = (*it);
      polyline.push_back(LibBoard::Point(p[0],p[1]));
    }
          if (isClosed) {
      FP::RealPoint p = (*v.begin());
      polyline.push_back(LibBoard::Point(p[0],p[1]));
          }
          aBoard.setPenColor(DGtal::Color::Black);
          aBoard.drawPolyline(polyline);

  }

      }

    }



  }
 
 

  if(vm.count("SDP") || vm.count("SFP")){
    bool drawPoints= vm.count("drawContourPoint");
    bool invertYaxis = vm.count("invertYaxis");
    double pointSize=1.0;
    if(drawPoints){
      pointSize = vm["drawContourPoint"].as<double>();
    }
    vector<LibBoard::Point> contourPt;
    if(vm.count("SDP")){
      string fileName = vm["SDP"].as<string>();
      vector< Z2i::Point >  contour = 
  PointListReader< Z2i::Point >::getPointsFromFile(fileName); 
      for(unsigned int j=0; j<contour.size(); j++){
  LibBoard::Point pt((double)(contour.at(j)[0]),
         (invertYaxis? (double)(-contour.at(j)[1]+contour.at(0)[1]):(double)(contour.at(j)[1])));
  contourPt.push_back(pt);
  if(drawPoints){
    aBoard.fillCircle(pt.x, pt.y, pointSize);
  }
      }
    }
 
    if(vm.count("SFP")){
      string fileName = vm["SFP"].as<string>();
      vector< PointVector<2,double>  >  contour = 
	PointListReader<  PointVector<2,double>  >::getPointsFromFile(fileName); 
      for(unsigned int j=0; j<contour.size(); j++){
  LibBoard::Point pt((double)(contour.at(j)[0]),
         (invertYaxis? (double)(-contour.at(j)[1]+contour.at(0)[1]):(double)(contour.at(j)[1])));
  contourPt.push_back(pt);
  if(drawPoints){
    aBoard.fillCircle(pt.x, pt.y, pointSize);
  }
      }
    }
  
  
    aBoard.setPenColor(Color::Red);
    aBoard.setLineStyle (LibBoard::Shape::SolidStyle );
    aBoard.setLineWidth (lineWidth);
    aBoard.drawPolyline(contourPt);
  
  
  }

 



  

  
  if (vm.count("outputSVG")){
    string outputFileName= vm["outputSVG"].as<string>();
    aBoard.saveSVG(outputFileName.c_str());
  } else   
    if (vm.count("outputFIG")){
      string outputFileName= vm["outputFIG"].as<string>();
      aBoard.saveFIG(outputFileName.c_str());
    } else
      if (vm.count("outputEPS")){
  string outputFileName= vm["outputEPS"].as<string>();
  aBoard.saveEPS(outputFileName.c_str());
      }  
#ifdef WITH_CAIRO
      else
  if (vm.count("outputEPS")){
    string outputFileName= vm["outputEPS"].as<string>();
    aBoard.saveCairo(outputFileName.c_str(),Board2D::CairoEPS );
  } else 
    if (vm.count("outputPDF")){
      string outputFileName= vm["outputPDF"].as<string>();
      aBoard.saveCairo(outputFileName.c_str(),Board2D::CairoPDF );
    } else 
      if (vm.count("outputPNG")){
        string outputFileName= vm["outputPNG"].as<string>();
        aBoard.saveCairo(outputFileName.c_str(),Board2D::CairoPNG );
      }
#endif
      else { //default output
        string outputFileName= "output.eps";
        aBoard.saveEPS(outputFileName.c_str());
      }
  
}

