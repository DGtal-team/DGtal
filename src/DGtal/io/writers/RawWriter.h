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
 * @file RawWriter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/07/22
 *
 * Header file for module RawWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RawWriter_RECURSES)
#error Recursive header files inclusion detected in RawWriter.h
#else // defined(RawWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RawWriter_RECURSES

#if !defined RawWriter_h
/** Prevents repeated inclusion of headers. */
#define RawWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <string>
#include <boost/static_assert.hpp>
#include <boost/type_traits.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/base/BasicFunctors.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class RawWriter
  /**
   * Description of template struct 'RawWriter' <p>
   * \brief Aim: Raw binary export of an Image.
   *
   * A functor can be specified to convert image values to raw values
   * (unsigned char for exportRaw8).
   *
   * @tparam TImage the Image type.
   * @tparam TFunctor the type of functor used in the export.
   */
  template <typename TImage, typename TFunctor = DefaultFunctor>
  struct RawWriter
  {
    // ----------------------- Standard services ------------------------------

    typedef TImage Image;
    typedef typename TImage::Value Value;
    typedef TFunctor Functor;
    
    BOOST_CONCEPT_ASSERT((  CUnaryFunctor<TFunctor, Value, unsigned char> )) ;    
    
    /** 
     * Export an Image to  Raw format (8bits, unsigned char).
     *
     * @param filename name of the output file
     * @param aImage the image to export
     * @param aFunctor functor used to cast image values
     * @return true if no errors occur.
     */
    static bool exportRaw8(const std::string & filename, const Image &aImage, 
			   const Functor & aFunctor = Functor());
    
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/RawWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RawWriter_h

#undef RawWriter_RECURSES
#endif // else defined(RawWriter_RECURSES)
