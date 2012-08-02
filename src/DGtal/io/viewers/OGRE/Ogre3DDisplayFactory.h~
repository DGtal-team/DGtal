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
 * @file   Ogre3DDisplayFactory.h
 * @author Anis Benyoub
 * @date   mercredi 14 juin 2012
 *
 * @brief
 *
 * Header file for module Ogre3DDisplayFactor
 *
 * This file is part of the DGtal library.
 */

#if defined(Ogre3DDisplayFactory_RECURSES)
#error Recursive header files inclusion detected in Ogre3DDisplayFactory.h
#else // defined(Ogre3DDisplayFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Ogre3DDisplayFactory_RECURSES

#if !defined Ogre3DDisplayFactory_h
/** Prevents repeated inclusion of headers. */
#define Ogre3DDisplayFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DrawableWithOgreWrapper.h"
#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/Object.h"
#include "DGtal/kernel/PointVector.h"
#include  "DrawWithViewerOgre3DModifier.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
namespace DGtal
  {

  class ViewerOgre3D;

/////////////////////////////////////////////////////////////////////////////
  struct Ogre3DDisplayFactory
        /**
         * Description of struct 'Ogre3DDisplayFactory' <p>
         * \brief Ogre Factory for Ogre3DDisplayFactory:
         */
    {
    	
      // MeshFromPoints    
    
	  template <typename TPoint>
	  static void drawAsFaces( ViewerOgre3D & display,  const DGtal::MeshFromPoints<TPoint> & );
	
	  template <typename TPoint>
	  static void draw( ViewerOgre3D & display, const  DGtal::MeshFromPoints<TPoint> &  );


      // ArithmeticalDSS
      template < typename TIterator,typename TInteger, int connectivity>
      static void draw ( ViewerOgre3D & display, const DGtal::ArithmeticalDSS3d <TIterator, TInteger,connectivity> & c);
      
      
      
      //---------------------------------------------------------------------------DGtalSetBySTLSet
      
      //Generic method
      template<typename Domain>
      static void draw( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & s );
      
      // Paving/Voxel drawing
      template<typename Domain>
      static void drawAsPaving( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & s );
      
      //Transparent Paving/Voxel drawing
      template<typename Domain>
      static void drawAsPavingTransparent( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & s );
      
      
      // Grid/Point drawing
      template<typename Domain>
      static void drawAsGrid( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & s );
      
      
     
      //---------------------------------------------------------------------------DigitalSetBySTLVector
      template<typename Domain>
      static void draw( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & s );
      
      template<typename Domain>
      static void drawAsPaving( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & );
      
      template<typename Domain>
      static void drawAsPavingTransparent( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & );
      
      template<typename Domain>
      static void drawAsGrid( ViewerOgre3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & );
    
    
    
      //---------------------------------------------------------------------------HyperRectDomain      
      template<typename TSpace>
      static void draw( ViewerOgre3D & viewer, const DGtal::HyperRectDomain<TSpace> & s );
 
      //---------------------------------------------------------------------------KhalimskyCell
      template<Dimension dim, typename TInteger>
      static void draw ( ViewerOgre3D & viewer, const DGtal::KhalimskyCell<dim,TInteger> & kc );
      
      
        
      //------------------------------------------------------------------------------------  Object
      template <typename TDigitalTopology, typename TDigitalSet>
      static void draw (ViewerOgre3D &viewer, const DGtal::Object< TDigitalTopology, TDigitalSet > &);
      
      //---------------------------------------------------------------------------PointVector
      template<Dimension dim, typename TComponent>
      static void draw ( ViewerOgre3D & viewer, const PointVector<dim,TComponent> & );
       
      template<Dimension dim, typename TComponent>
      static void drawAsGrid( ViewerOgre3D & viewer, const DGtal::PointVector<dim,TComponent> & );

      template<Dimension dim, typename TComponent>
      static void drawAsPaving( ViewerOgre3D & viewer, const DGtal::PointVector<dim,TComponent> & );
      
      
     //---------------------------------------------------------------------------SignedKhalimskyCell
     template< Dimension dim,typename TInteger>
     static void draw ( ViewerOgre3D & display, const DGtal::SignedKhalimskyCell<dim,TInteger> & skc);
     
     
     

  
     static void draw( ViewerOgre3D & display, const DGtal::SetViewerMode3D & );
     static void draw( ViewerOgre3D & display, const DGtal::CustomViewerStyle3D & );
     static void draw( ViewerOgre3D & display, const DGtal::CustomViewerColors3D & );
     
  
     static void draw( ViewerOgre3D & display, const DGtal::ViewerClippingPlane & );
     //Camera 
     static void draw( ViewerOgre3D & display, const DGtal::ViewerCameraPosition & );
     static void draw( ViewerOgre3D & display, const DGtal::ViewerCameraDirection & );
     static void draw( ViewerOgre3D & display, const DGtal::ViewerCameraUpVector & );
     static void draw( ViewerOgre3D & display, const DGtal::ViewerCameraZNearFar & );

      


    }; // end of struct Ogre3DDisplayFactor

} //  namespace DGtal




///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "ViewerOgre3D.h"
#include "Ogre3DDisplayFactory.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Ogre3DDisplayFactory_h

#undef Ogre3DDisplayFactory_RECURSES
#endif // else defined(Ogre3DDisplayFactory_RECURSES)
