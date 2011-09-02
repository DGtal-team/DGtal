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

#pragma once

/**
 * @file Modifier.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2011/09/01
 *
 * Header file for module Modifier.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Modifier_RECURSES)
#error Recursive header files inclusion detected in Modifier.h
#else // defined(Modifier_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Modifier_RECURSES

#if !defined Modifier_h
/** Prevents repeated inclusion of headers. */
#define Modifier_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/PointVector.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  
  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dXY
  /**
   * Description of template class 'Point3dTo2dXY' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the xy-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  ...
  a2dPoint = Point3dTo2dXY<int>::get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dXY
  {
        
    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    static Output get(Input p) 
    {
      Input tmp = p;
      return Output(tmp.at(0),tmp.at(1));
    }
  
  }; // end of class Point3dTo2dXY
  
  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dXZ
  /**
   * Description of template class 'Point3dTo2dXZ' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the xz-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  ...
  a2dPoint = Point3dTo2dXZ<int>::get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dXZ
  {

    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    static Output get(Input p) 
    {
      Input tmp = p;
      return Output(tmp.at(0),tmp.at(2));
    }
  
  }; // end of class Point3dTo2dXZ

  /////////////////////////////////////////////////////////////////////////////
  // template class Point3dTo2dYZ
  /**
   * Description of template class 'Point3dTo2dYZ' <p>
   * \brief Aim: transforms a 3d point into a 2d point
   * due to a projection on the yz-plane
   * @tparam Coordinate the type for the coordinates of the points
   * @code
  PointVector<3,int> a3dPoint; 
  PointVector<2,int> a2dPoint; 
  ...
  a2dPoint = Point3dTo2dYZ<int>::get(a3dPoint); 
   * @endcode
   * @see ConstIteratorAdapter
   */
  template <typename Coordinate>
  class Point3dTo2dYZ
  {
    
    public: 
      
    typedef PointVector<3,Coordinate> Input; 
    typedef PointVector<2,Coordinate> Output; 
    
    public:
      
    static Output get(Input p) 
    {
      Input tmp = p;
      return Output(tmp.at(1),tmp.at(2));
    }
  
  }; // end of class Point3dTo2dYZ

  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//#include "DGtal/base/Modifier.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Modifier_h

#undef Modifier_RECURSES
#endif // else defined(Modifier_RECURSES)
