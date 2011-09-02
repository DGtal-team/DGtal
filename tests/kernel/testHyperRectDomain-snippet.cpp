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
 * @file test_HyperRectDomain-snippet.cpp
 * @ingroup Tests
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 *
 *
 * @date 2010/05/25
 *
 * This file is part of the DGtal library
 */

/**
 * Description of test_HyperRectDomain-snippet <p>
 * Aim: simple test of \ref HyperRectDomain based on the code snippet in the \ref HyperRectDomain class documentation.
 */

#include <algorithm> //for std::copy()
#include <iterator>
#include <iostream>
#include <DGtal/kernel/SpaceND.h>
#include <DGtal/kernel/domains/HyperRectDomain.h>


using namespace DGtal;

int main()
{
  //We create a digital Space based on 'int' integers and in dimension 4
  typedef DGtal::SpaceND<4> Space4D;
  typedef Space4D::Point Point4D;

  const DGtal::int32_t rawA[ ] = { 1, 2, 3 ,4};
  const DGtal::int32_t rawB[ ] = { 4, 4, 5 ,5};
  Point4D A ( rawA );
  Point4D B ( rawB );

  //Domain construction from two points
  HyperRectDomain<Space4D> myDomain ( A, B );

  //We just iterate on the Domain points and print out the point coordinates.
  std::copy ( myDomain.begin(),
        myDomain.end(),
        std::ostream_iterator<Point4D> ( std::cout, " " ) );
}
/** @ingroup Tests **/
