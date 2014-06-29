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
 * @file testSimpleRegression.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/03/26
 *
 * Functions for testing class SimpleRegression.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "ConfigTest.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/math/SimpleLinearRegression.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////
// Functions for testing class SimpleRegression.
///////////////////////////////////////////////////////////////////////////////
/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleRegression()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing SimpleLinearRegression ..." );

  SimpleLinearRegression SLR;

  SLR.addSample(0,0);
  SLR.addSample(1,1);
  SLR.addSample(34,34);
  SLR.addSample(3,3);

  nbok += SLR.computeRegression() ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Regression == true" << std::endl;

  trace.info() << "Got slope= "<< SLR.slope()<<std::endl;
  trace.info() << "Got Intercept= "<< SLR.intercept()<<std::endl;


  nbok += ( std::abs(SLR.slope() - 1) < 0.01)  ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "slope == 1" << std::endl;
  nbok += ( std::abs(SLR.intercept() - 0.0) < 0.01)  ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "intercept == 0" << std::endl;
  trace.endBlock();

  return nbok == nb;
}


/**
 * Example of a test. To be completed.
 *
 */
bool testSimpleRegression2()
{
  unsigned int nbok = 0;
  unsigned int nb = 0;

  trace.beginBlock ( "Testing SimpleLinearRegression2..." );

  SimpleLinearRegression SLR;

  std::vector<double> x;
  x.push_back(0);
  x.push_back(2);
  x.push_back(34);
  x.push_back(3);
  std::vector<double> y;
  y.push_back(0);
  y.push_back(2);
  y.push_back(33);
  y.push_back(2.7);

  SLR.addSamples( x.begin(), x.end(), y.begin());

  nbok += SLR.computeRegression() ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "Regression == true" << std::endl;

  trace.info() << "Got slope= "<< SLR.slope()<<std::endl;
  trace.info() << "Got Intercept= "<< SLR.intercept()<<std::endl;


  nbok += ( std::abs(SLR.slope() - 1) < 0.1)  ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "|slope| =~= 1" << std::endl;
  nbok += ( std::abs(SLR.intercept() ) < 0.1)  ? 1 : 0;
  nb++;
  trace.info() << "(" << nbok << "/" << nb << ") "
	       << "|intercept| < (10^-1)" << std::endl;
  trace.endBlock();

  return nbok == nb;
}
///////////////////////////////////////////////////////////////////////////////
// Standard services - public :

int main( int argc, char** argv )
{
  trace.beginBlock ( "Testing class SimpleRegression" );
  trace.info() << "Args:";
  for ( int i = 0; i < argc; ++i )
    trace.info() << " " << argv[ i ];
  trace.info() << endl;

  bool res = testSimpleRegression()
    && testSimpleRegression2(); // && ... other tests
  trace.emphase() << ( res ? "Passed." : "Error." ) << endl;
  trace.endBlock();
  return res ? 0 : 1;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
