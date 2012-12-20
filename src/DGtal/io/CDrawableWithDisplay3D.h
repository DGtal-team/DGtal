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
 * @file CDrawableWithDisplay3D.h
 * @author Bertrand Kerautret (\c kerautre@loria.fr )
 * LORIA (CNRS, UMR 7503), University of Nancy, France
 *
 * @date 2011/08/08
 *
 * Header file for concept CDrawableWithDisplay3D.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CDrawableWithDisplay3D_RECURSES)
#error Recursive header files inclusion detected in CDrawableWithDisplay3D.h
#else // defined(CDrawableWithDisplay3D_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CDrawableWithDisplay3D_RECURSES

#if !defined CDrawableWithDisplay3D_h
/** Prevents repeated inclusion of headers. */
#define CDrawableWithDisplay3D_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "boost/concept_check.hpp"
#include "DGtal/base/Common.h"
//#include "DGtal/io/Display3D.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // class CDrawableWithDisplay3D
  /**
   * Description of \b concept '\b CDrawableWithDisplay3D' <p>
   * @ingroup Concepts
   * Aim:  The concept CDrawableWithDisplay3D specifies what are the classes
   * that admit an export with Display3D.
   * An object x satisfying this concept may then be used as:
   * 
   \code
   Display3D display;
   display << CustomStyle( x.className(), x.defaultStyle() )
         << x;
   \endcode 
   *
   * <p> Refinement of
   *
   * <p> Associated types :
   *
   * <p> Notation
   * - \t X : A type that is a model of CDrawableWithDisplay3DD
   * - \t x, \t y  : Object of type X
   * - \t m  : a string of characters
   *
   * <p> Definitions
   *
   * <p> Valid expressions and 
   * <table> <tr> <td> \b Name </td> <td> \b Expression </td>
   * <td> \b Type requirements </td> <td> \b Return type </td>
   * <td> \b Precondition </td> <td> \b Semantics </td>
   * <td> \b Postcondition </td> <td> \b Complexity </td>
   * </tr>
   * <tr>
   * <td> the default draw style</td> <td> x.defaultStyle( m = "" ) </td> <td> \t mode : \c std::string</td><td> CDrawableWithDisplay3D * </td> <td> </td> <td> returns a dynamic allocation of the default style for the model \t X in mode \t m</td><td> </td>
   *  <td> </td>
   * </tr>
   * <tr>
   * <td> the name of the model X</td> <td> x.className() </td> <td></td><td> std::string </td> <td> </td> <td> returns a string telling the name of the model X </td><td> </td>
   *  <td> </td>
   * </tr>
   * <tr>
   * <td> the way the object \t x is drawn</td> <td> x.setStyle(CDrawableWithDisplay3D &display) </td> <td></td> <td> </td> <td> </td> <td> draws on the \c display stream the object \c x </td><td> </td>
   *  <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants###
   *
   * <p> Models###
   * ArimeticalDSS, FreemanChain, HyperRectDomain, ImageContainerByHashTree, ImageContainerBySTLVector, PointVector, DigitalSetBySTLSet,DigitalSetBySTLVector, Object
   *
   * <p> Notes###
   * @todo ImageContainerByHashTree does not implement setStyle(display &).
   * @todo ImageContainerByHashTree does not implement defaultStyle(std::string&)const.
   */  

  class Display3D;
  
  template <typename T>
  struct CDrawableWithDisplay3D
  {

  BOOST_CONCEPT_USAGE( CDrawableWithDisplay3D )
    {
      //Drawable model should have a className() returning a string
      ConceptUtils::sameType( myS, myT.className() );

      //Drawable model should be associated to global functions draw and defaultStyle.
      //draw(myD3D, myT);
      //ConceptUtils::sameType( myD, defaultStyle( myT) );
    }

    // ------------------------- Private Datas --------------------------------
  private:
    T myT;
    DrawableWithDisplay3D *myD;

    Display3D myD3D;
    std::string myS;

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of concept CDrawableWithDisplay3D
  
} // namespace DGtal



//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CDrawableWithDisplay3D_h

#undef CDrawableWithDisplay3D_RECURSES
#endif // else defined(CDrawableWithDisplay3D_RECURSES)
