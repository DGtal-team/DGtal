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
 * @file   OgreStyle3DFactory.h
 * @author Anis Benyoub
 * @date   mercredi 11/07/2012
 * 
 * @brief
 *
 * Header file for module OgreStyle3DFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(OgreStyle3DFactory_RECURSES)
#error Recursive header files inclusion detected in OgreStyle3DFactory.h
#else // defined(OgreStyle3DFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define OgreStyle3DFactory_RECURSES

#if !defined OgreStyle3DFactory_h
/** Prevents repeated inclusion of headers. */
#define OgreStyle3DFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions

#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/Object.h"
#include "DGtal/kernel/PointVector.h"

#include "DGtal/io/viewers/OGRE/ViewerOgre3D.h"
#include "DGtal/base/Common.h"
#include "DGtal/io/viewers/OGRE/CommonOgre.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
struct OgreStyle3DFactory
{
    static void draw( DGtal::ViewerOgre3D & aBoard, DGtal::DrawableWithViewerOgre3D *style )
    {
	style->setStyle(aBoard);
    }

  
//------------------------------------------------------------------------------------  MeshFromPoints
template <typename TPoint>
static
DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::MeshFromPoints<TPoint> & /*m*/,
					   std::string mode  = "" );
//------------------------------------------------------------------------------------  ArithmeticalDSS3d
template <typename TIterator, typename TInteger, int connectivity>
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & /*a*/,
					   std::string mode = "" );
//------------------------------------------------------------------------------------  ArithmeticalDSS3d



//------------------------------------------------------------------------------------  DigitalSetBySTLSet
template<typename Domain>

static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::DigitalSetBySTLSet<Domain> & /*s*/, std::string mode = "" );
//------------------------------------------------------------------------------------  DigitalSetBySTLSet



//------------------------------------------------------------------------------------  DigitalSetBySTLVector
template<typename Domain>
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::DigitalSetBySTLVector<Domain> & /*v*/, std::string mode = "" );
//------------------------------------------------------------------------------------  DigitalSetBySTLVector



//------------------------------------------------------------------------------------  HyperRectDomain
template<typename TSpace>
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::HyperRectDomain<TSpace> & /*h*/, std::string mode = "" );
//------------------------------------------------------------------------------------  HyperRectDomain

//------------------------------------------------------------------------------------   KhalimskyCell
template < Dimension dim, typename TInteger >
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::KhalimskyCell<dim, TInteger> & /*k*/, std::string mode = "" );
//------------------------------------------------------------------------------------  KhalimskyCell


//------------------------------------------------------------------------------------  Object
template <typename TDigitalTopology, typename TDigitalSet>
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::Object<TDigitalTopology, TDigitalSet> & /*o*/, std::string mode = "" );
//------------------------------------------------------------------------------------  Object


//------------------------------------------------------------------------------------  PointVector
template<Dimension dim, typename TComponent>
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::PointVector<dim,TComponent> & /*p*/, std::string mode = "");
//------------------------------------------------------------------------------------  PointVector


//------------------------------------------------------------------------------------  SignedKhalimskyCell
template < Dimension dim, typename TInteger >
static DGtal::DrawableWithViewerOgre3D* defaultStyle(const DGtal::SignedKhalimskyCell<dim, TInteger> & /*sk*/, std::string mode = "" );
//------------------------------------------------------------------------------------   SignedKhalimskyCell


static DrawableWithViewerOgre3D* defaultStyle(const CustomViewerStyle3D & cs, std::string mode = "" );


static DrawableWithViewerOgre3D* defaultStyle(const SetViewerMode3D & sm, std::string mode = "" );

static DrawableWithViewerOgre3D* defaultStyle(const CustomViewerColors3D & sm, std::string mode = "" );

static DrawableWithViewerOgre3D* defaultStyle(const ViewerClippingPlane & sm, std::string mode = "" );
static DrawableWithViewerOgre3D* defaultStyle(const ViewerCameraPosition & sm, std::string mode = "" );
static DrawableWithViewerOgre3D* defaultStyle(const ViewerCameraUpVector & sm, std::string mode = "" );
static DrawableWithViewerOgre3D* defaultStyle(const ViewerCameraZNearFar & sm, std::string mode = "" );
static DrawableWithViewerOgre3D* defaultStyle(const ViewerCameraDirection & sm, std::string mode = "" );
  };
}

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods 
#include "OgreStyle3DFactory.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined OgreStyle3DFactory_h

#undef OgreStyle3DFactory_RECURSES
#endif // else defined(OgreStyle3DFactory_RECURSES)
