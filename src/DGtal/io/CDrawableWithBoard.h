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
 * @file CDrawableWithBoard.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2010/10/21
 *
 * Header file for concept CDrawableWithBoard.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDrawableWithBoard_RECURSES)
#error Recursive header files inclusion detected in CDrawableWithBoard.h
#else // defined(CDrawableWithBoard_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDrawableWithBoard_RECURSES

#if !defined CDrawableWithBoard_h
/** Prevents repeated inclusion of headers. */
#define CDrawableWithBoard_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
#include "DGtal/io/DGtalBoard.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDrawableWithBoard
  /**
   * Description of \b concept '\b CDrawableWithBoard' <p>
   * @ingroup Concepts
   * Aim:  The concept CDrawableWithBoard specifies what are the classes
   * that admit an export with DGtalBoarD.
   *
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CDrawableWithBoard
   * - \t x, \t y	: Object of type X
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
   * <td> the default draw style</td> <td> x.defaultStyle() </td> <td></td><td> DrawableWithBoard * </td> <td> </td> <td> returns a dynamic allocation of the default style for the model X </td><td> </td>
   *  <td> O(1)</td>
   * </tr>
   * <tr>
   * <td> the name of the model X</td> <td> x.styleName() </td> <td></td><td> std::string </td> <td> </td> <td> returns a string designing the name of the model X </td><td> </td>
   *  <td> O(1)</td>
   * </tr>
   * <tr>
   * <td> the way the object x are drawn</td> <td> x.selfDraw(DGtalBoard &board) </td> <td></td><td> std::string </td> <td> </td> <td> returns a string designing the name of the model X </td><td> </td>
   *  <td> O(1)</td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * <p> Notes <br>
   */
  template <typename T>
  struct CDrawableWithBoard
  {
    // ----------------------- Concept checks ------------------------------
  public:
    BOOST_CONCEPT_USAGE( CDrawableWithBoard )
    {
      //Drawable model should have a defaultStyle() returing a DrawableWithBoard*
      ConceptUtils::sameType( myD, myT.defaultStyle() );
      //Drawable model should have a styleName() returing a string
      ConceptUtils::sameType( myS, myT.styleName() );
      //Drawable model should have a selfDraw()
      ///@todo FIXME: si on décommente ça plante
//      myT.selfDraw( myB );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myT;
    DrawableWithBoard *myD;

    ///@todo FIXME: si on décommente ça plante
    // DGtalBoard myB;
    std::string myS;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CDrawableWithBoard

} // namespace DGtal


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDrawableWithBoard_h

#undef CDrawableWithBoard_RECURSES
#endif // else defined(CDrawableWithBoard_RECURSES)
