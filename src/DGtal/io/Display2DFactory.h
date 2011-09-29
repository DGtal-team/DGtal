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
 * @file   Display2DFactory.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 28 septembre 2011
 * 
 * @brief
 *
 * Header file for module Display2DFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(Display2DFactory_RECURSES)
#error Recursive header files inclusion detected in Display2DFactory.h
#else // defined(Display2DFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Display2DFactory_RECURSES

#if !defined Display2DFactory_h
/** Prevents repeated inclusion of headers. */
#define Display2DFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"

//#include "DGtal/io/DrawWithDisplay3DModifier.h"

#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/topology/Object.h"
//#include "DGtal/topology/KhalimskySpaceND.h"

// TODO: begin
// TODO: end

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // struct Display2DFactory
  /**
   * Description of struct 'Display2DFactory' <p>
   * \brief Factory for Display2D:
   */
  struct Display2DFactory
  {
    // ArithmeticalDSS3d
    template <typename TIterator, typename TInteger, int connectivity=8>
    static void draw( Board2D & board, const DGtal::ArithmeticalDSS<TIterator,TInteger,connectivity> & );
    // ArithmeticalDSS3d
    
    
    // FreemanChain
    /*template <typename TInteger>
    static void draw( Board2D & board, const DGtal::FreemanChain<TInteger> & );*/
    // FreemanChain
    
    
    // PointVector
    template<Dimension dim, typename TComponent>
    static void draw( Board2D & board, const DGtal::PointVector<dim,TComponent> & );
    // PointVector
    
    
    // HyperRectDomain
    template<typename TSpace>
    static void draw( Board2D & board, const DGtal::HyperRectDomain<TSpace> & );
    // HyperRectDomain
    
    
    // DigitalSetBySTLSet
    template<typename Domain>
    static void draw( Board2D & board, const DGtal::DigitalSetBySTLSet<Domain> & );
    // DigitalSetBySTLSet
    
    
    // DigitalSetBySTLVector
    template<typename Domain>
    static void draw( Board2D & board, const DGtal::DigitalSetBySTLVector<Domain> & );
    // DigitalSetBySTLVector
    
    
    // Object
    template <typename TDigitalTopology, typename TDigitalSet>
    static void draw( Board2D & board, const DGtal::Object<TDigitalTopology, TDigitalSet> & );
    // Object
    
    
    // KhalimskyCell
    // KhalimskyCell
    
    // SignedKhalimskyCell
    // SignedKhalimskyCell
    
    //
    
    static void draw( Board2D & board, const DGtal::SetMode & );
    static void draw( Board2D & board, const DGtal::CustomStyle & );
    //static void draw( Display3D & display, const DGtal::CustomColors3D & );

  }; // end of struct Display2DFactory

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods 
#include "DGtal/io/Display2DFactory.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display2DFactory_h

#undef Display2DFactory_RECURSES
#endif // else defined(Display2DFactory_RECURSES)
