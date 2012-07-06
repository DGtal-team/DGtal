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
 * @file testIndexedListWithBlocks.cpp
 * @ingroup Tests
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 *
 * @date 2012/07/02
 *
 * This file is part of the DGtal library
 */

//#define TRACE_BITS

#include <cstdio>
#include <cmath>
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/IndexedListWithBlocks.h"

using namespace DGtal;
using namespace std;

int main()
{
  unsigned int nb = 0;
  unsigned int nbok = 0;
  trace.beginBlock ( "Testing IndexedListWithBlocks" );
  IndexedListWithBlocks<double, 2, 4> l;
  std::cout << "l=" << l << std::endl; 
  trace.endBlock();
  return nb == nbok;
}
/** @ingroup Tests **/
