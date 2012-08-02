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
* @file CommonOgre.h
* @author Anis Benyoub (\c anis.benyoub@insa-lyon.fr )
* Liris CNRS
*
* @date 2012/07/11
*
* Header file for module CommonOgre.cpp
*
* This file is part of the DGtal library.
*/

#if defined(CommonOgre_RECURSES)
#error Recursive header files inclusion detected in CommonOgre.h
#else // defined(CommonOgre_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CommonOgre_RECURSES

#if !defined CommonOgre_h
/** Prevents repeated inclusion of headers. */
#define CommonOgre_h

#include <iostream>

namespace DGtal
 {
   class ViewerOgre3D;

 struct DrawableWithViewerOgre3D {
    virtual void setStyle( ViewerOgre3D & ) const 
    {
    }

  };

 
 } // namespace DGtal


 #endif /* CommonOgre_H_ */

#undef CommonOgre_RECURSES
#endif // else defined(CommonOgre_RECURSES)
