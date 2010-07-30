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
#include "DGtal/utils/ConceptUtils.h"
#include "DGtal/kernel/images/CValueType.h"
#include "DGtal/base/Common.h"
#include "Board/Color.h"
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
   * <td>Construction</td> <td>CMap<ValueType> cmap(min,max);</td> <td>min and max are of the same ValueType</td> <td> </td>
   * <td> </td> <td> </td> <td> </td> <td> </td>
   * </tr>
   * <tr> 
   * <td>Obtain a color</td> <td>color=cmap(value)</td> <td>value is a ValueType</td> <td>LibBoard::Color</td>
   * <td>min &le; value &le; max </td> <td>Returns a color computed after the position of \em value \em within
   * the range [min,max]</td> <td> </td> <td> </td>
   * </tr>
   * </table>
   *
   * <p> Invariants <br>
   *
   * <p> Models <br>
   *
   * <p> Notes <br>
   */
  template <typename CMap>
  struct CColorMap
  {
    // ----------------------- Concept checks ------------------------------
  public:
    
    typedef typename CMap::ValueType ValueType;
    
    BOOST_CONCEPT_ASSERT(( CValueType<ValueType> ));
    
    BOOST_CONCEPT_USAGE( CColorMap )
    {
      CMap myCMap( myMin, myMax );
      // operator() exists, takes a ValueType, and returns a LibBoard::Color.
      ConceptUtils::sameType( myColor, myCMap.operator()( myValue ) );
    }
    
    // ------------------------- Private Datas --------------------------------
  private:
    
    // ------------------------- Internals ------------------------------------
  private:
    LibBoard::Color myColor;
    ValueType myMin, myMax, myValue;    
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
