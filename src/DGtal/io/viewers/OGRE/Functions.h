/**
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as
* published by the Free Software Foundation, either version 3 of the
* License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
**/

#pragma once

/**
* @file Functions.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/06/26
*
* Implementation of methods defined in Functions.h
*
* This file is part of the DGtal library.
*/


#if defined(Functions_RECURSES)
#error Recursive header files inclusion detected in Functions.h
#else // defined(Functions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Functions_RECURSES

#if !defined Functions_h
/** Prevents repeated inclusion of headers. */
#define Functions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/shapes/Shapes.h"
#include <iostream>
//////////////////////////////////////////////////////////////////////////////




DGtal::Z3i::DigitalSet & setModify(DGtal::Z3i::DigitalSet  aSet, int  aValue)
{
      DGtal::Z3i::Point p4( 30, 30 ,30 );
      DGtal::Z3i::Point p5( -30, -30 ,-30 );
      DGtal::Z3i::Domain domain( p4, p5 );
      
      DGtal::Z3i::DigitalSet * shape_set1 = new DGtal::Z3i::DigitalSet( domain );

      
        typedef typename std::set<DGtal::Z3i::Point>::const_iterator ConstIterator;

      int nbOfPoints=0;
      
      for ( ConstIterator it = aSet.begin(); (it != aSet.end())&&(nbOfPoints<aValue); ++it )
	{
	  nbOfPoints++;
	  shape_set1->insert((*it));
	}
      
      
      return * shape_set1;
}
// //
///////////////////////////////////////////////////////////////////////////////

#endif // !Functions XXX_h

#undef Functions_RECURSES
#endif // else defined(Functions_RECURSES)
