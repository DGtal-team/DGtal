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
 * @file kernelDomain.cpp
 * @ingroup Examples
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 * @date 2011/03/05
 *
 * An example file named demo-kernel-domain.
 *
 * This file is part of the DGtal library.
 */

///////////////////////////////////////////////////////////////////////////////
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/SpaceND.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/io/DGtalBoard.h"

#ifdef WITH_GMP
#include <gmpxx.h>
#endif
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main()
{
  typedef DGtal::SpaceND<2, DGtal::int32_t> MySpace;
  typedef MySpace::Point MyPoint;
  typedef HyperRectDomain<MySpace> MyDomain;
  MyPoint a(-3,-4);
  MyPoint b(10,4);
  MyDomain domain(a,b);
  DGtalBoard board; // for 2D display
  board << domain << MyPoint(5,1);
  board.saveSVG("demo-kernel-1.svg");
  board.saveEPS("demo-kernel-1.eps");
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
