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
 * @file GenericWriter.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2013/05/01
 *
 * Header file for module GenericWriter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(GenericWriter_RECURSES)
#error Recursive header files inclusion detected in GenericWriter.h
#else // defined(GenericWriter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define GenericWriter_RECURSES

#if !defined GenericWriter_h
/** Prevents repeated inclusion of headers. */
#define GenericWriter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/base/CUnaryFunctor.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class GenericWriter


  template <typename TContainer, int Tdim=TContainer::Point::dimension, typename TFunctor = DefaultFunctor >
  struct GenericWriter
  {
    static bool exporT(const std::string &filename, const TContainer &anImage,  
			const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);
  };

  /**
   * GenericWriter
   * Template partial specialisation for volume images of dimension 3
   **/
  template <typename TContainer, typename TFunctor>
  struct GenericWriter<TContainer, 3 , TFunctor>
  {

    /**
     * Export a volume image file. 
     *
     **/
    static bool exporT(const std::string &filename,  const TContainer &anImage,
		       const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);

  };

  /**
   * GenericWriter
   * Template partial specialisation for volume images of dimension 2
   **/
  template <typename TContainer, typename TFunctor>
  struct GenericWriter<TContainer, 2, TFunctor>
  {

    /**
     * Write a volume image file.  
     *
     **/

    static bool exporT(const std::string &filename, const TContainer &anImage,
		        const TFunctor & aFunctor = TFunctor() )  throw(DGtal::IOException);

  }; 




} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/writers/GenericWriter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined GenericWriter_h

#undef GenericWriter_RECURSES
#endif // else defined(GenericWriter_RECURSES)
