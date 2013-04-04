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
#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/Object.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/shapes/fromPoints/MeshFromPoints.h"
#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/CColorMap.h"



//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // struct Display3DFactory
  /**
   * Description of struct 'Display3DFactory' <p>
   * \brief Factory for GPL Display3D:
   */
  struct Display3DFactory
  {
    
    // SphericalAccumulator
    /** 
     * Display an spherical accumulator in 3D. Bin values are mapped
     * using a default HueShadeColorMap.
     * 
     * @param display current display
     * @param accumulator the accumulator to display
     * @param shift translate vector for display purposes (default:
     * zero vector)
     * @param radius scale factor for the unit sphere radius (default:1)
     * @tparam TVector a vector model
     */
    template <typename TVector>
    static void draw( Display3D & display, const  DGtal::SphericalAccumulator<TVector> & accumulator,
                      const typename DGtal::SphericalAccumulator<TVector>::RealVector &shift = 
                      typename DGtal::SphericalAccumulator<TVector>::RealVector(0,0,0),
                      const double radius=1.0);
    // SphericalAccumulator

    // MeshFromPoints        
    template <typename TPoint>
    static void drawAsFaces( Display3D & display,  const DGtal::MeshFromPoints<TPoint> & );

    template <typename TPoint>
    static void draw( Display3D & display, const  DGtal::MeshFromPoints<TPoint> &  );
    // MeshFromPoints


    
    // ArithmeticalDSS3d
    /**
      * Default drawing style object.
      * @return the dyn. alloc. default style for this object.
      */
    template <typename TIterator, typename TInteger, int connectivity>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & );
    
    template <typename TIterator, typename TInteger, int connectivity>
    static void drawAsPoints( Display3D & display, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & );
    
    template <typename TIterator, typename TInteger, int connectivity>
    static void drawAsBoundingBox( Display3D & display, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & );

    template <typename TIterator, typename TInteger, int connectivity>
    static void draw( Display3D & display, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & );
    // ArithmeticalDSS3d
    
    
    // DigitalSetBySTLSet
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    template<typename Domain>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::DigitalSetBySTLSet<Domain> & );
  
    template<typename Domain>
    static void drawAsPavingTransparent( Display3D & display, const DGtal::DigitalSetBySTLSet<Domain> & );
    
    template<typename Domain>
    static void drawAsPaving( Display3D & display, const DGtal::DigitalSetBySTLSet<Domain> & );
    
    template<typename Domain>
    static void drawAsGrid( Display3D & display, const DGtal::DigitalSetBySTLSet<Domain> & );
    
    template<typename Domain>
    static void draw( Display3D & display, const DGtal::DigitalSetBySTLSet<Domain> & );
    // DigitalSetBySTLSet
    
    
    // DigitalSetBySTLVector
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    template<typename Domain>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::DigitalSetBySTLVector<Domain> & );

    template<typename Domain>
    static void drawAsPavingTransparent( Display3D & display, const DGtal::DigitalSetBySTLVector<Domain> & );

    template<typename Domain>
    static void drawAsPaving( Display3D & display, const DGtal::DigitalSetBySTLVector<Domain> & );
    
    template<typename Domain>
    static void drawAsGrid( Display3D & display, const DGtal::DigitalSetBySTLVector<Domain> & );

    template<typename Domain>
    static void draw( Display3D & display, const DGtal::DigitalSetBySTLVector<Domain> & );
    // DigitalSetBySTLVector
    
    
    // HyperRectDomain
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    template<typename TSpace>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename TSpace>
    static void drawAsBoundingBox( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename TSpace>
    static void drawAsGrid( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename TSpace>
    static void drawAsPavingPoints( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename TSpace>
    static void drawAsPaving( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    
    template<typename TSpace>
    static void draw( Display3D & display, const DGtal::HyperRectDomain<TSpace> & );
    // HyperRectDomain
    
    
    // KhalimskyCell
    /**
      * Default drawing style object.
      * @return the dyn. alloc. default style for this object.
      */
    template < Dimension dim, typename TInteger >
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::KhalimskyCell<dim, TInteger> & );
    
    template < Dimension dim, typename TInteger >
    static void draw( Display3D & display, const DGtal::KhalimskyCell<dim, TInteger> & );
    // KhalimskyCell
    
    
    // Object
    template <typename TDigitalTopology, typename TDigitalSet>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::Object<TDigitalTopology, TDigitalSet> & );

    template <typename TDigitalTopology, typename TDigitalSet>
    static void drawWithAdjacencies( Display3D & display, const DGtal::Object<TDigitalTopology, TDigitalSet> & );

    template <typename TDigitalTopology, typename TDigitalSet>
    static void draw( Display3D & display, const DGtal::Object<TDigitalTopology, TDigitalSet> & );
    // Object
    
    
    // PointVector
    /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
    template<Dimension dim, typename TComponent>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::PointVector<dim,TComponent> & );
    
    template<Dimension dim, typename TComponent>
    static void drawAsGrid( Display3D & display, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    static void drawAsPaving( Display3D & display, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    static void drawAsPavingWired( Display3D & display, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    static void draw( Display3D & display, const DGtal::PointVector<dim,TComponent> & );
    
    template<Dimension dim, typename TComponent>
    static void draw( Display3D & display, const DGtal::PointVector<dim,TComponent> & , const DGtal::PointVector<dim,TComponent> & );
    // PointVector
    
    
    // SignedKhalimskyCell
    /**
      * Default drawing style object.
      * @return the dyn. alloc. default style for this object.
      */
    template< Dimension dim, typename TInteger >
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string, const DGtal::SignedKhalimskyCell<dim, TInteger> & );
    
    template< Dimension dim, typename TInteger >
    static void draw( Display3D & display, const DGtal::SignedKhalimskyCell<dim, TInteger> & );
    // SignedKhalimskyCell
   
    // GridCurve
    template< typename TKSpace >
    static void draw( Display3D & display, const DGtal::GridCurve<TKSpace> & );
    // GridCurve 
    
    // SCellsRange
    template < typename TIterator, typename TSCell >
    static void draw( DGtal::Display3D & display, 
          const DGtal::ConstRangeAdapter<TIterator, DGtal::DefaultFunctor, TSCell> & );
    // SCellsRange

    // PointsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToPoint<TKSpace>, typename TKSpace::Point> & );
    // PointsRange

    // MidPointsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToMidPoint<TKSpace>, 
               typename TKSpace::Space::RealPoint> & );
    // MidPointsRange

    // ArrowsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToArrow<TKSpace>, 
               std::pair<typename TKSpace::Point, typename TKSpace::Vector > > & );
    // ArrowsRange

    // InnerPointsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToInnerPoint<TKSpace>, typename TKSpace::Point> & );
    // InnerPointsRange

    // OuterPointsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToOuterPoint<TKSpace>, typename TKSpace::Point> & );
    // OuterPointsRange

    // IncidentPointsRange
    template <typename TIterator, typename TKSpace>
    static void draw( Display3D & display, 
               const DGtal::ConstRangeAdapter<TIterator, SCellToIncidentPoints<TKSpace>, 
               std::pair<typename TKSpace::Point, typename TKSpace::Point > > & );
    // IncidentPointsRange

    //
    
    
    static void draw( Display3D & display, const DGtal::SetMode3D & );
    static void draw( Display3D & display, const DGtal::CustomStyle3D & );
    static void draw( Display3D & display, const DGtal::CustomColors3D & );
    
    static void draw( Display3D & display, const DGtal::ClippingPlane & );
    
    static void draw( Display3D & display, const DGtal::CameraPosition & );    
    static void draw( Display3D & display, const DGtal::CameraDirection & );
    static void draw( Display3D & display, const DGtal::CameraUpVector & );
    static void draw( Display3D & display, const DGtal::CameraZNearFar & );

    static void draw( Display3D & display, const DGtal::TransformedKSSurfel & aTransformedKSSurfel);

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
