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
 * @file   Display3DFactory.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 21 septembre 2011
 * 
 * @brief
 *
 * Header file for module Display3DFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(Display3DFactory_RECURSES)
#error Recursive header files inclusion detected in Display3DFactory.h
#else // defined(Display3DFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Display3DFactory_RECURSES

#if !defined Display3DFactory_h
/** Prevents repeated inclusion of headers. */
#define Display3DFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"

#include "DGtal/io/DrawWithDisplay3DModifier.h"

#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"


// verif concept
// verif comment all class
// verif comment modifier

// verif qglviewer ?


//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  
  /////////////////////////////////////////////////////////////////////////////
  // struct Display3DFactory
  /**
   * Description of struct 'Display3DFactory' <p>
   * \brief Aim:
   */
  struct Display3DFactory
  {
    
    template<Dimension dim, typename TComponent>
    static void draw( Display3D & display, const DGtal::PointVector<dim,TComponent> & );
    
    template<typename TSpace>
    static void draw( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename Domain>
    static void draw( Display3D & display, const DGtal::DigitalSetBySTLSet<Domain> & );
    
    template <typename TDigitalTopology, typename TDigitalSet>
    static void draw( Display3D & display, const DGtal::Object<TDigitalTopology, TDigitalSet> & );
    
    /*template<typename Domain>
    static void draw( Display3D & display, const DGtal::DigitalSetBySTLVector<Domain> & );*/
    
    template < Dimension dim, typename TInteger >
    static void draw( Display3D & display, const DGtal::KhalimskyCell<dim, TInteger> & );
    
    template< Dimension dim, typename TInteger >
    static void draw( Display3D & display, const DGtal::SignedKhalimskyCell<dim, TInteger> & );
    
    //
    
    static void draw( Display3D & display, const DGtal::SetMode3D & );
    static void draw( Display3D & display, const DGtal::CustomColors3D & );
    static void draw( Display3D & display, const DGtal::ClippingPlane & );
    
    static void draw( Display3D & display, const DGtal::CameraPosition & );    
    static void draw( Display3D & display, const DGtal::CameraDirection & );
    static void draw( Display3D & display, const DGtal::CameraUpVector & );

  }; // end of struct Display3DFactory

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods 
#include "DGtal/io/Display3DFactory.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display3DFactory_h

#undef Display3DFactory_RECURSES
#endif // else defined(Display3DFactory_RECURSES)
