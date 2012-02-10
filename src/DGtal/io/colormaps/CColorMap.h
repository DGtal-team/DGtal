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
 * @file CColorMap.h
 * @author Sebastien Fourey (\c Sebastien.Fourey@greyc.ensicaen.fr )
 * Groupe de Recherche en Informatique, Image, Automatique et Instrumentation de Caen - GREYC (CNRS, UMR 6072), ENSICAEN, France
 *
 * @date 2010/07/30
 *
 * Header file for concept CColorMap.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CColorMap_RECURSES)
#error Recursive header files inclusion detected in CColorMap.h
#else // defined(CColorMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CColorMap_RECURSES

#if !defined CColorMap_h
/** Prevents repeated inclusion of headers. */
#define CColorMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/base/CLabel.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/Color.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CColorMap
  /**
   * Description of \b concept \b 'CColorMap' <p>
   * @ingroup Concepts
   * \brief Aim: Defines the concept describing a color map. A color map converts
   * a value within a given range into an RGB triple.
   * 
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CColorMap
   * - \t x, \t y  : Object of type X
   *
   * <p> Definitions
   *
   * <p> Valid expressions and semantics <br>
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td> 
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr> 
   * <td>Construction</td> <td>CMap<Value> cmap(min,max);</td> <td>min and max are of the same Value</td> <td> </td>
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * </tr>
   * <tr> 
   * <td>Obtain a color</td> <td>color=cmap(value)</td> <td>value is a Value</td> <td>DGtal::Color</td>
   * <td>min &le; value &le; max </td> <td>Returns a color computed after the position of \em value \em within
   * the range [min,max]</td> <td> </td> <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *        GradientColorMap
   *        HueShadeColorMap
   *        ColorBrightnessColorMap 
   *        GrayScaleColorMap
   *        RandomColorMap
   * <p> Notes <br>
   */
  template <typename CMap>
  struct CColorMap
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename CMap::Value Value;
    
    BOOST_CONCEPT_ASSERT(( CLabel<Value> ));
    
    BOOST_CONCEPT_USAGE( CColorMap )
    {
      CMap myCMap( myMin, myMax );
      // operator() exists, takes a Value, and returns a LibBoard::Color.
      ConceptUtils::sameType( myColor, myCMap.operator()( myValue ) );
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    Color myColor;
    Value myMin, myMax, myValue;    
  }; // end of concept CColorMap
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/io/colormaps/CColorMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CColorMap_h

#undef CColorMap_RECURSES
#endif // else defined(CColorMap_RECURSES)
