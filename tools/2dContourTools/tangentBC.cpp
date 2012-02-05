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
 * @file curvatureBC.cpp
 * @ingroup Tools
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/07/14
 *
 * Output the tangent of the Freeman code of a grid curve
 * using the Binomial convolver
 * 
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <cmath>
#include <iostream>
#include <vector>
#include <string>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/variables_map.hpp>


#include "DGtal/base/Common.h"
#include "DGtal/helpers/StdDefs.h"

//Grid curve
#include "DGtal/geometry/curves/representation/FreemanChain.h"
#include "DGtal/geometry/curves/representation/GridCurve.h"

//Estimators
#include "DGtal/geometry/curves/evolution/BinomialConvolver.h"

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
    ("GridStep,step", po::value<double>()->default_value(1.0), "Grid step");
  
  
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, general_opt), vm);  
  po::notify(vm);    
  if(vm.count("help")||argc<=1 || (!(vm.count("FreemanChain"))) )
    {
      trace.info()<< "Tangent using a binomial convolver " <<std::endl << "Basic usage: "<<std::endl
      << "\t tangentBC [options] --FreemanChain  <fileName> "<<std::endl
      << general_opt << "\n"
      << "NB: the file may contain several freeman chains." << "\n";
      return 0;
    }
  
  
  double h = vm["GridStep"].as<double>();  


 
  if(vm.count("FreemanChain")){
    string fileName = vm["FreemanChain"].as<string>();

    typedef Z2i::Space Space; 
    typedef Space::Point Point; 
    typedef PointVector<2, double> RealPoint; 
    typedef Space::Integer Integer;  
    typedef FreemanChain<Integer> FreemanChain; 
    typedef vector< Point > Storage;
    typedef Storage::const_iterator ConstIteratorOnPoints; 

    vector< FreemanChain > vectFcs =  
      PointListReader< Point >:: getFreemanChainsFromFile<Integer> (fileName); 

    for(unsigned int i=0; i<vectFcs.size(); i++){

      bool isClosed = vectFcs.at(i).isClosed(); 
      cout << "# grid curve " << i << "/" << vectFcs.size() << " "
      << ( (isClosed)?"closed":"open" ) << endl;

      Storage vectPts; 
      FreemanChain::getContourPoints( vectFcs.at(i), vectPts ); 

      // Binomial
      std::cout << "# Curvature estimation from binomial convolution" << std::endl;
      typedef BinomialConvolver<ConstIteratorOnPoints, double> MyBinomialConvolver;
      std::cout << "# mask size = " << 
      MyBinomialConvolver::suggestedSize( h, vectPts.begin(), vectPts.end() ) << std::endl;
      typedef 
  TangentFromBinomialConvolverFunctor< MyBinomialConvolver, RealPoint >
  TangentBCFct;
      BinomialConvolverEstimator< MyBinomialConvolver, TangentBCFct> 
  BCTangentEstimator;

      BCTangentEstimator.init( h, vectPts.begin(), vectPts.end(), isClosed );

      vector<RealPoint> tangents( vectPts.size() ); 
      BCTangentEstimator.eval( vectPts.begin(), vectPts.end(), 
             tangents.begin() ); 

      // Output
      cout << "# id tangent.x tangent.y angle(atan2(y,x))" << endl;  
      unsigned int j = 0;
      for ( ConstIteratorOnPoints 
        it = vectPts.begin(), it_end = vectPts.end();
      it != it_end; ++it, ++j ) 
  {
    double x = tangents[ j ][ 0 ];
    double y = tangents[ j ][ 1 ];
    cout << j << setprecision( 15 )
         << " " << x << " " << y 
         << " " << atan2( y, x )
         << endl;
  }

    }

  }
  return 0;
}

