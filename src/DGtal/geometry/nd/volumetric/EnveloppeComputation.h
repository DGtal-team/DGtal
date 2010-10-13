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
//LICENSE-END
#pragma once

/**
 * @file EnveloppeComputation.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/13
 *
 * Header file for module EnveloppeComputation.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(EnveloppeComputation_RECURSES)
#error Recursive header files inclusion detected in EnveloppeComputation.h
#else // defined(EnveloppeComputation_RECURSES)
/** Prevents recursive inclusion of headers. */
#define EnveloppeComputation_RECURSES

#if !defined EnveloppeComputation_h
/** Prevents repeated inclusion of headers. */
#define EnveloppeComputation_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class EnveloppeComputation
  /**
   * Description of template class 'EnveloppeComputation' <p>
   * \brief Aim:
   */
  template <typename TImage>
  struct EnveloppeComputation
  {
    typedef TImage Image;
    typedef typename Image::Size Size;

    static void lowerEnveloppe(const Image &input, Size s[], Size t[], Size & q);


  };
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/nd/volumetric/EnveloppeComputation.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined EnveloppeComputation_h

#undef EnveloppeComputation_RECURSES
#endif // else defined(EnveloppeComputation_RECURSES)
