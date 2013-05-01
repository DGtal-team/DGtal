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
 * @file GenericReader.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Header file for module GenericReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericReader_RECURSES)
#error Recursive header files inclusion detected in GenericReader.h
#else // defined(GenericReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericReader_RECURSES

#if !defined GenericReader_h
/** Prevents repeated inclusion of headers. */
#define GenericReader_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericReader
  /**
   * Description of template class 'GenericReader' <p>
   * \brief Aim: Provide a mechanism to load with the bestloader according to an image filename (by parsing the extension).
   *  
   * The typical use is very simple:
   * 
   *
   *
   */
template <typename TContainer, typename TPoint = typename TContainer::Point, int Tdim=TPoint::dimension >
  struct GenericReader
  {
    static TContainer import(const std::string &filename)  throw(DGtal::IOException);

  };


template <typename TContainer, typename TPoint>
struct GenericReader<TContainer, TPoint, 3 >
  {
    static TContainer import(const std::string &filename,  unsigned int x=0, 
			     unsigned int y=0, unsigned int z=0)  throw(DGtal::IOException);

  };

template <typename TContainer, typename TPoint>
struct GenericReader<TContainer,  TPoint, 2>
  {
    static TContainer import(const std::string &filename,  const std::string &datasetName="empty")  throw(DGtal::IOException);

  };


// @todo when a MeshFromPoints(Mesh) will contain a type Point
/*
template <typename TPoint>
struct GenericReader<MeshFromPoints<TPoint>,  TPoint, 2>
{
  static MeshFromPoints<TPoint> import(const std::string &filename)  throw(DGtal::IOException);
};
*/


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/GenericReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericReader_h

#undef GenericReader_RECURSES
#endif // else defined(GenericReader_RECURSES)
