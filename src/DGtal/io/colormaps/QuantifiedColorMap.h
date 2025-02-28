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
 * @file QuantifiedColorMap.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/10/30
 *
 * Header file for module QuantifiedColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(QuantifiedColorMap_RECURSES)
#error Recursive header files inclusion detected in QuantifiedColorMap.h
#else // defined(QuantifiedColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define QuantifiedColorMap_RECURSES

#if !defined QuantifiedColorMap_h
/** Prevents repeated inclusion of headers. */
#define QuantifiedColorMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <sstream>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clone.h"
#include "DGtal/io/Color.h"
#include "DGtal/io/colormaps/CColorMap.h"

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class QuantifiedColorMap
  /**
     Description of template class 'QuantifiedColorMap' <p> \brief
     Aim: A modifier class that quantifies any colormap into a given
     number of colors. It is particularly useful when rendering
     colored objects, since for instance blender is very slow to load
     many different materials.

     @tparam TColorMap an arbitrary model of concepts::CColorMap.
  */
  template < concepts::CColorMap TColorMap >
  struct QuantifiedColorMap
  {
    using ColorMap = TColorMap;
    using Self     = QuantifiedColorMap< ColorMap >;
    using Value    = typename ColorMap::Value;

    //---------------------------------------------------------------------------
  public:

    /// Constructor from \a colormap and \a nb colors
    /// @param[in] colormap the colormap to quantify in \a nb colors.
    /// @param[in] nb the targeted maximum number of colors (default is 50).
    QuantifiedColorMap( Clone< ColorMap > colormap, int nb = 50 )
      : myColorMap( colormap ), myNbColors( nb )
    {}

    /// Computes the color associated with a value in a given range.
    ///
    /// @param value A value within the value range.
    /// @return A color whose brightness linearly depends on the
    /// position of [value] within the current range.
    Color operator()( const Value & value ) const
    {
      const Value rel  = ( value - myColorMap.min() )
        / ( myColorMap.max() - myColorMap.min() );
      const Value qrel = round( myNbColors * rel ) / myNbColors;
      const Value outv = qrel * ( myColorMap.max() - myColorMap.min() )
        + myColorMap.min();
      return myColorMap( outv );
    }

    //---------------------------------------------------------------------------
  public:
    /// the colormap that is quantified
    ColorMap myColorMap;
    /// the maximum number of colors 
    int myNbColors;
    
  };

  /// Template function to simplify the build of QuantifiedColorMap object.
  /// @tparam TColorMap an arbitrary model of concepts::CColorMap.
  /// @param[in] colormap the colormap to quantify in \a nb colors.
  /// @param[in] nb the targeted maximum number of colors (default is 50).
  template < typename TColorMap >
  QuantifiedColorMap< TColorMap >
  makeQuantifiedColorMap( TColorMap colormap, int nb = 50 )
  {
    return QuantifiedColorMap< TColorMap >( colormap, nb );
  }

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined QuantifiedColorMap_h

#undef QuantifiedColorMap_RECURSES
#endif // else defined(QuantifiedColorMap_RECURSES)
