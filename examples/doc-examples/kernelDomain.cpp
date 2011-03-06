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
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/03/05
 *
 * An example file named kernelDomain.
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
///////////////////////////////////////////////////////////////////////////////

using namespace std;
using namespace DGtal;

///////////////////////////////////////////////////////////////////////////////

int main( int argc, char** argv )
{
  trace.beginBlock ( "Example kernelDomain" );
  
  //We create several space models.
  typedef DGtal::SpaceND<3, DGtal::int32_t> MySpace32;
  typedef DGtal::SpaceND<1, DGtal::int64_t> MySpace8;
  typedef DGtal::SpaceND<3, mpz_class> MySpaceGMP;
  typedef DGtal::Z2i::Space MySpace; 
  
  //Point lying in the Z2i::Space
  typedef MySpace::Point MyPoint;
  
  MyPoint p(13,-5);
  
  trace.info() << "Point p="<<p<<endl;
  
  //We create a domain
  typedef HyperRectDomain<MySpace> MyDomain;
  MyPoint a(-3,-4);
  MyPoint b(10,4);
  MyDomain domain(a,b);
  
  //We trace  domain information
  trace.info() <<"Domain domain="<<domain<<endl;

  //We generate a board
  DGtalBoard board;
  board << domain;
  board.saveSVG("kernel-domain.svg");

  MyPoint c(5,1);

  if ( domain.isInside(c) )
    trace.info() << "C is inside the domain"<<endl;
  else
    trace.info() << "C is outside the domain"<<endl;
    
  board << c;
  board.saveSVG("kernel-domain-point.svg");
  
  //We scan the domain
  for( MyDomain::ConstIterator it = domain.begin(), itend = domain.end();
       it != itend;   
       ++it)
    trace.info() << "Processing point"<< (*it) << endl;
    
  //We clean up the current board
  board.clear();
  board << domain;

  //Let us prepare the mapping between coordinates and colors
  MyPoint lower = domain.lowerBound();
  MyPoint upper = domain.upperBound();
  MyPoint size = domain.size();
  MySpace::Integer extent = size[0]*size[1];

  //since DGtal::Integer may be unbounded, we have to cast it to int64_t
  // to be able to use the build-in '/' operator.
  DGtal::int64_t castExtent = IntegerTraits<MySpace::Integer>::castToInt64_t(extent);
  DGtal::int64_t pos=0;
  
  //we draw each point with a custom color  
  for( MyDomain::ConstIterator it = domain.begin(), itend = domain.end();
       it != itend;   
       ++it)
    {
      board << CustomStyle( (*it).styleName(), 
			    new CustomFillColor( DGtalBoard::Color( (255*pos) / castExtent,
								    0,
								    255 - (255*pos) / castExtent)))
	    << (*it);
      pos++;

    }
  board.saveSVG("kernel-domain-it.svg");


  trace.endBlock();
  return 0;
}
//                                                                           //
///////////////////////////////////////////////////////////////////////////////
