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
 * @file
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2018/06/26
 *
 * An example file named shortcuts.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "ConfigExamples.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/helpers/Shortcuts.h"
#include "DGtal/base/Common.h"
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  typedef Shortcuts< Z3i::KSpace > SH3;
  trace.beginBlock ( "Setting parameters" );
  // Preferred way of creating parameters.
  auto params = Parameters( "polynomial", "3*x^2+2*y^2+z^2-90" )( "gridstep", 0.5 );
  std::cout << params << std::endl;
  trace.endBlock();
  trace.beginBlock ( "Making implicit shape" );
  auto implicit_shape = SH3::makeImplicitShape( params );
  std::cout << *implicit_shape << std::endl;
  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
