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
#include "DGtal/io/writers/VolWriter.h"
#include "DGtal/io/writers/RawWriter.h"
#include "DGtal/io/writers/PNMWriter.h"


#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>

#include <vector>
#include <string>

using namespace DGtal;

std::vector<string> shapesND;
std::vector<string> shapesDesc;
std::vector<string> shapesParam1;
std::vector<string> shapesParam2;
std::vector<string> shapesParam3;

void createList()
{
  shapesND.push_back("ball");
  shapesDesc.push_back("Ball for the Euclidean metric.");
  shapesParam1("-radius");
  shapesParam2("");
  shapesParam3("");
 
  shapesND.push_back("cube");
  shapesDesc.push_back("hypercube in nD.");
  shapesParam1("-width");
  shapesParam2("");
  shapesParam3("");

}

void displayList()
{
  trace.emphase()<<"nD Shapes:"<<std::endl;
  for(unsigned int i=0; i<shapesND.size(); ++i)
    trace.info()<<"\t"<<shapesND[i]<<"\t parameter(s): "
		<< shapesParam1[i]<<" "
      		<< shapesParam2[i]<<" "
      		<< shapesParam3[i]<<" "
		<< " Description: "<<shapesDesc[i]<<endl;
  
}

///////////////////////////////////////////////////////////////////////////////
namespace po = boost::program_options;

int main( int argc, char** argv )
{
  // parse command line ----------------------------------------------
  po::options_description general_opt("Allowed options are: ");
  general_opt.add_options()
    ("help,h", "display this message")
    ("dimension,d", po::value<unsigned int>()->default_value(2), "Dimension of the shape") 
    ("shape,s", po::value<std::string>(), "Shape type")
    ("list,l",  "List all available shapes")
    ("radius,r",  po::value<unsigned int>()->default_value(10), "Radius of the shape" )
    ("width,w",  po::value<unsigned int>()->default_value(10), "Width of the shape" )
    ("power,p",   po::value<double>()->default_value(2.0), "Power of the shape" );
 
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1)
    {
      std::cout	<< "Generate shapes using DGtal library" << "Usage: " <<  " \n"
		<< general_opt << "\n";
      return 0;
    }

  


  

}
