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
 * @file exampleGridCurve3d.cpp
 * @ingroup Examples
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/06/03
 *
 * @brief An example file for GridCurve in 3d.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>

#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"
#include "ConfigExamples.h"

#include "DGtal/geometry/curves/GridCurve.h"

  #ifdef WITH_VISU3D_QGLVIEWER
#include <QtGui/qapplication.h>
#include "DGtal/io/viewers/Viewer3D.h"
#include "DGtal/io/DrawWithDisplay3DModifier.h"
   #endif

///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;
using namespace Z3i; 

/////////////////////
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

namespace po = boost::program_options;

///////////////////////////////////////////////////////////////////////////////
int main( int argc, char** argv )
{
  //////////////////////////////////////////////////////////////////////////////////
  // parse command line 
  po::options_description general_opt("Allowed options are");
  general_opt.add_options()
    ("help,h", "display this message")
    ("range,r",  po::value<string>()->default_value("gridcurve"), 
     " Either <gridcurve> (default), <scells>, <points>, <midpoints>, <arrows> " ); 
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help"))
    {
      trace.info()<< "exampleGridCurve3d" << std::endl
		  << "Basic usage: "<<std::endl
		  << argv[0] << " " << std::endl
		  << general_opt << "\n";
      return 0;
    }
  
  //Parse options
  string type = vm["range"].as<string>(); 

  //curve
  string sinus = examplesPath + "samples/sinus.dat";
  
  // domain
  Point lowerBound = Point::diagonal( -100 );
  Point upperBound = Point::diagonal( 100 ); 

  //! [GridCurveDeclaration]
  K3 ks; ks.init( lowerBound, upperBound, true ); 
  GridCurve<K3> gc( ks ); 
  //! [GridCurveDeclaration]
  
  //! [GridCurveFromDataFile]
  fstream inputStream;
  inputStream.open (sinus.c_str(), ios::in);

  gc.initFromVectorStream(inputStream);

  inputStream.close();
  //! [GridCurveFromDataFile]

  bool flag = false; 
  #ifdef WITH_VISU3D_QGLVIEWER
  QApplication application(argc,argv);
  Viewer3D viewer;
  viewer.show();

  if (type == "gridcurve")
    {
      viewer  << gc;
    }
  else if (type == "scells")
    {
      viewer << gc.getSCellsRange(); 
    }
  else if (type == "points")
    {
      viewer << gc.getPointsRange(); 
    }
  else if (type == "midpoints")
    {
      viewer << gc.getMidPointsRange(); 
    }
  else if (type == "arrows")
    {
      viewer << gc.getArrowsRange(); 
    }
  else
    {
      trace.info() << "Display type not known. Use option -h" << std::endl; 
    }
  viewer << Viewer3D::updateDisplay;
  flag = application.exec();
   #endif
  
  return flag;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
