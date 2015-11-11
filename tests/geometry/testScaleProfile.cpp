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
 * @file testScaleProfile.cpp
 * @ingroup Tests
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2015/11/08
 *
 * Functions for testing class ScaleProfile.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtalCatch.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/geometry/helpers/ScaleProfile.h" 
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class ScaleProfile.
///////////////////////////////////////////////////////////////////////////////

TEST_CASE( "Testing ScaleProfile" )
{
  
  ScaleProfile sp(ScaleProfile::MEAN);
  sp.init(5);
  
  SECTION("Testing basic add of ScaleProfile")
    {
      sp.addValue(0, 10.0);
      sp.addValue(1, 10.0);
      sp.addValue(2, 10.0);
      sp.addValue(3, 10.0);
      sp.addValue(4, 10.0);
      std::vector<double> x;
      std::vector<double> y;
      sp.getProfile(x, y);

      REQUIRE( (x[3] == Approx(log(4))) );
      REQUIRE( (y[3] == Approx(log(10.0))) );
    }



  sp.clear();
  sp.init(6);
  SECTION("Testing noise level detect of ScaleProfile")
    {
      sp.addValue(0,22);
      sp.addValue(1,15);
      sp.addValue(2,8);
      sp.addValue(3,17);
      sp.addValue(4,7);
      sp.addValue(5,2);      
      std::vector< std::pair<uint, uint> > interval;
      sp.meaningfulScales(interval, 1);
      uint n = sp.noiseLevel();
      REQUIRE( (interval[0].first == 1) );
      REQUIRE( (interval[0].second == 3) );
      REQUIRE( (interval[1].first == 4) );
      REQUIRE( (n == 1) );
    }


  sp.clear();

  
  SECTION("Testing noise level detect of ScaleProfile with iterator init ")
    {
      std::vector<double> scales;
      for(unsigned int i =0; i < 6; i++){
        scales.push_back(5+i);
      }
      sp.init(scales.begin(), scales.end());  
      sp.addValue(0,22);
      sp.addValue(1,15);
      sp.addValue(2,8);
      sp.addValue(3,17);
      sp.addValue(4,7);
      sp.addValue(5,2);      
      std::vector<double> x,y;
      sp.getProfile(x,y); 
      REQUIRE( (x[0] == Approx(log(5))) );
      REQUIRE( (y[0] == Approx(log(22))) );
      REQUIRE( (x[3] == Approx(log(8))) );
      REQUIRE( (y[3] == Approx(log(17))) );
    }

}

/** @ingroup Tests **/
