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
 * @ingroup Examples
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/01/04
 *
 * An example file named qglViewer.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include <QtGui/qapplication.h>

#include "DGtal/base/Common.h"
#include "DGtal/io/readers/VolReader.h"
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"

#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/images/ImageSelector.h"


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include "DGtal/helpers/StdDefs.h"

using namespace std;
using namespace DGtal;
using namespace DGtal::Z3i;

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

int main( int argc, char** argv )
{
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("input-file,i", po::value<std::string>(), "volume file" )
    ("thresholdMin,m",  po::value<int>()->default_value(0), "threshold min to define binary shape" ) 
    ("thresholdMax,M",  po::value<int>()->default_value(255), "threshold max to define binary shape" )
    ("transparency,t",  po::value<uint>()->default_value(255), "transparency") ; 
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      std::cout << "Usage: " << argv[0] << " [input-file]\n"
    << "Display volume file as a voxel set by using QGLviewer"
    << general_opt << "\n";
      return 0;
    }
  
  if(! vm.count("input-file"))
    {
      trace.error() << " The file name was defined" << endl;      
      return 0;
    }
  string inputFilename = vm["input-file"].as<std::string>();
  int thresholdMin = vm["thresholdMin"].as<int>();
  int thresholdMax = vm["thresholdMax"].as<int>();
  unsigned char transp = vm["transparency"].as<uint>();
 
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.setWindowTitle("simple Volume Viewer");
  viewer.show();
 
  typedef ImageSelector<Domain, unsigned char>::Type Image;
  Image image = VolReader<Image>::importVol( inputFilename );

  trace.info() << "Image loaded: "<<image<< std::endl;

  Domain domain = image.domain();
  GradientColorMap<long> gradient( thresholdMin, thresholdMax);
  gradient.addColor(Color::Blue);
  gradient.addColor(Color::Green);
  gradient.addColor(Color::Yellow);
  gradient.addColor(Color::Red);
  for(Domain::ConstIterator it = domain.begin(), itend=domain.end(); it!=itend; ++it){
    unsigned char  val= image( (*it) );     
   
    Color c= gradient(val);
    if(val<=thresholdMax && val >=thresholdMin){
      viewer <<  CustomColors3D(Color((float)(c.red()), (float)(c.green()),(float)(c.blue()), transp),
        Color((float)(c.red()), (float)(c.green()),(float)(c.blue()), transp));     
      viewer << *it;     
    }     
  }
  viewer << Viewer3D::updateDisplay;
  return application.exec();
}
