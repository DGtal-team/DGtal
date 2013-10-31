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
 * @file ITKReader.h
 * @author Pierre Gueth (\c pierre.gueth@gmail.com )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/10/28
 *
 * Header file for module ITKReader.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ITKReader_RECURSES)
#error Recursive header files inclusion detected in ITKReader.h
#else // defined(ITKReader_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ITKReader_RECURSES

#if !defined ITKReader_h
/** Prevents repeated inclusion of headers. */
#define ITKReader_h

#include "DGtal/base/Common.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/io/ITKIOTrait.h"

namespace DGtal
{

  /**
   * Description of template class 'ITKReader'
   * \brief Aim: Import a 2D/3D Image using the ITK formats.
   *
   * @tparam TImage the Image type.
   * @tparam TFunctor the type of functor used in the export.
   * @see ITKWriter
   * @see ITKIOTrait
   */
  template <typename TImage, typename TFunctor = typename ITKIOTrait<typename TImage::Value>::DefaultReadFunctor >
  struct ITKReader
  {
    typedef TImage Image;
    typedef typename TImage::Value Value;
    typedef typename ITKIOTrait<Value>::ValueOut ValueOut;
    typedef TFunctor Functor;

    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TFunctor, ValueOut, Value> )) ;
    BOOST_STATIC_ASSERT(( (TImage::Domain::dimension == 3) || (TImage::Domain::dimension == 2) ));

    /**
     * Import an Image with a format supported by ITK.
     *
     * @param filename name of the output file
     * @param aFunctor functor used to cast image values
     * @return read image
     */
    static Image importITK(const std::string & filename,
        const Functor & aFunctor = Functor()) throw(DGtal::IOException);
  };
}//namespace

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/readers/ITKReader.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ITKReader_h

#undef ITKReader_RECURSES
#endif // else defined(ITKReader_RECURSES)
