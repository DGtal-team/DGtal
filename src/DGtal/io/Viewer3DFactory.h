

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
 * @file   Viewer3DFactory.h
 * @author Aline Martin <aline.martin@insa-lyon.fr>
 * @date   mardi 2 juillet 2013
 *
 * @brief
 *
 * Header file for module Viewer3DFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(Viewer3DFactory_RECURSES)
#error Recursive header files inclusion detected in Viewer3DFactory.h
#else // defined(Viewer3DFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Viewer3DFactory_RECURSES

#if !defined Viewer3DFactory_h
/** Prevents repeated inclusion of headers. */
#define Viewer3DFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include "DGtal/io/Display3DFactory.h"
#include "DGtal/io/DrawWithViewer3DModifier.h"
#include "DGtal/geometry/curves/ArithmeticalDSS3d.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/Object.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/CColorMap.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageAdapter.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
/////////////////////////////////////////////////////////////////////////////
// struct Viewer3DFactory
/**
   * Description of struct 'Viewer3DFactory' <p>
   * \brief Factory for GPL Viewer3D:
   */
struct Viewer3DFactory : public Display3DFactory
{


  static void draw( Viewer3D & board, const DGtal::CameraPosition & );
  static void draw( Viewer3D & board, const DGtal::CameraDirection & );
  static void draw( Viewer3D & board, const DGtal::CameraUpVector & );
  static void draw( Viewer3D & board, const DGtal::CameraZNearFar & );

  //----------------------------------------------------------------------------------------------
  // heritage
  // SphericalAccumulator
  /**
     * Display an spherical accumulator in 3D. Bin values are mapped
     * using a default HueShadeColorMap.
     *
     * @param viewer current viewer
     * @param accumulator the accumulator to viewer
     * @param shift translate vector for viewer purposes (default:
     * zero vector)
     * @param radius scale factor for the unit sphere radius (default:1)
     * @tparam TVector a vector model
     */
  template <typename TVector>
  static void draw( Viewer3D & viewer, const  DGtal::SphericalAccumulator<TVector> & accumulator,
                    const typename DGtal::SphericalAccumulator<TVector>::RealVector & shift =
      typename DGtal::SphericalAccumulator<TVector>::RealVector(0,0,0),
                    const double radius=1.0);
  // SphericalAccumulator

  // Mesh
  template <typename TPoint>
  static void drawAsFaces( Viewer3D & viewer, const DGtal::Mesh<TPoint> & aMesh );

  template <typename TPoint>
  static void draw( Viewer3D & viewer, const  DGtal::Mesh<TPoint> & aMesh );
  // Mesh



  // ArithmeticalDSS3d
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template <typename TIterator, typename TInteger, int connectivity>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity>
  static void drawAsBalls( Viewer3D & viewer, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity>
  static void drawAsBoundingBox( Viewer3D & viewer, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity>
  static void draw( Viewer3D & viewer, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );
  // ArithmeticalDSS3d


  // DigitalSetBySTLSet
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template<typename Domain>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain>
  static void drawAsPavingTransparent( Viewer3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain>
  static void drawAsPaving( Viewer3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain>
  static void drawAsGrid( Viewer3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain>
  static void draw( Viewer3D & viewer, const DGtal::DigitalSetBySTLSet<Domain> & aSet );
  // DigitalSetBySTLSet


  // DigitalSetBySTLVector
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template<typename Domain>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain>
  static void drawAsPavingTransparent( Viewer3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain>
  static void drawAsPaving( Viewer3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain>
  static void drawAsGrid( Viewer3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain>
  static void draw( Viewer3D & viewer, const DGtal::DigitalSetBySTLVector<Domain> & aSet );
  // DigitalSetBySTLVector


  // HyperRectDomain
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template<typename TSpace>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace>
  static void drawAsBoundingBox( Viewer3D & viewer, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace>
  static void drawAsGrid( Viewer3D & viewer, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace>
  static void drawAsPavingBalls( Viewer3D & viewer, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace>
  static void drawAsPaving( Viewer3D & viewer, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace>
  static void draw( Viewer3D & viewer, const DGtal::HyperRectDomain<TSpace> & aDomain );
  // HyperRectDomain


  // KhalimskyCell
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template < Dimension dim, typename TInteger >
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::KhalimskyCell<dim, TInteger> & aCell );

  template < Dimension dim, typename TInteger >
  static void draw( Viewer3D & viewer, const DGtal::KhalimskyCell<dim, TInteger> & aCell );
  // KhalimskyCell


  // Object
  template <typename TDigitalTopology, typename TDigitalSet>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

  template <typename TDigitalTopology, typename TDigitalSet>
  static void drawWithAdjacencies( Viewer3D & viewer, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

  template <typename TDigitalTopology, typename TDigitalSet>
  static void draw( Viewer3D & viewer, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );
  // Object


  // PointVector
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template<Dimension dim, typename TComponent>
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent>
  static void drawAsGrid( Viewer3D & viewer, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent>
  static void drawAsPaving( Viewer3D & viewer, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent>
  static void drawAsPavingWired( Viewer3D & viewer, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent>
  static void draw( Viewer3D & viewer, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent>
  static void draw( Viewer3D & viewer, const DGtal::PointVector<dim,TComponent> & aPoint, const DGtal::PointVector<dim,TComponent> & aPoint2 );
  // PointVector


  // SignedKhalimskyCell
  /**
     * Default drawing style object.
     * @return the dyn. alloc. default style for this object.
     */
  template< Dimension dim, typename TInteger >
  static DGtal::DrawableWithViewer3D * defaultStyle( std::string str, const DGtal::SignedKhalimskyCell<dim, TInteger> & aSCell );

  template< Dimension dim, typename TInteger >
  static void draw( Viewer3D & viewer, const DGtal::SignedKhalimskyCell<dim, TInteger> & aSCell );
  // SignedKhalimskyCell

  // GridCurve
  template< typename TKSpace >
  static void draw( Viewer3D & viewer, const DGtal::GridCurve<TKSpace> & aGrid );
  // GridCurve

  // SCellsRange
  template < typename TIterator, typename TSCell >
  static void draw( DGtal::Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, DGtal::DefaultFunctor, TSCell> & aRangeAdapter );
  // SCellsRange

  // PointsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // PointsRange

  // MidPointsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToMidPoint<TKSpace>, typename TKSpace::Space::RealPoint> & aRangeAdapter );
  // MidPointsRange

  // ArrowsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToArrow<TKSpace>, std::pair<typename TKSpace::Point, typename TKSpace::Vector > > & aRangeAdapter );
  // ArrowsRange

  // InnerPointsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToInnerPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // InnerPointsRange

  // OuterPointsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToOuterPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // OuterPointsRange

  // IncidentPointsRange
  template <typename TIterator, typename TKSpace>
  static void draw( Viewer3D & viewer,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToIncidentPoints<TKSpace>,std::pair<typename TKSpace::Point, typename TKSpace::Point > > & aRangeAdapter );
  // IncidentPointsRange


  // ImageContainerBySTLVector  (2D)
  template <typename TValue>
  static void draw( Viewer3D & viewer, const  ImageContainerBySTLVector<DGtal::Z2i::Domain, TValue>  & anImage );
  // ImageContainerBySTLVector  (2D)

  // ImageContainerBySTLMap  (2D)
  template <typename TValue>
  static void draw( Viewer3D & viewer, const  ImageContainerBySTLMap<DGtal::Z2i::Domain, TValue>  & anImage );
  // ImageContainerBySTLMap  (2D)

  // ConstImageAdapter  (2D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue>
  static void draw( Viewer3D & viewer, const  ConstImageAdapter<TImageContainer, DGtal::Z2i::Domain, TFunctorD, TNewValue, TFunctorValue>  & anImage );
  // ConstImageAdapter  (2D)

  // ImageAdapter  (2D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename TFunctorValueVm1>
  static void draw( Viewer3D & viewer, const  ImageAdapter<TImageContainer, DGtal::Z2i::Domain, TFunctorD,
                    TNewValue, TFunctorValue, TFunctorValueVm1>  & anImage );
  // ImageAdapter  (2D)


  // ImageContainerBySTLVector  (3D)
  template <typename TValue  >
  static void draw( Viewer3D & viewer, const   ImageContainerBySTLVector<DGtal::Z3i::Domain, TValue>  & anImage );
  // ImageContainerBySTLVector (3D)


  // ImageContainerBySTLMap  (3D)
  template <typename TValue  >
  static void draw( Viewer3D & viewer, const   ImageContainerBySTLMap<DGtal::Z3i::Domain, TValue>  & anImage );
  // ImageContainerBySTLMap  (3D)

  // ConstImageAdapter  (3D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue>
  static void draw( Viewer3D & viewer, const  ConstImageAdapter<TImageContainer, DGtal::Z3i::Domain, TFunctorD,
                    TNewValue, TFunctorValue>  & anImage );
  // ConstImageAdapter  (3D)

  // ImageAdapter  (3D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename TFunctorValueVm1>
  static void draw( Viewer3D & viewer, const  ImageAdapter<TImageContainer, DGtal::Z3i::Domain, TFunctorD,
                    TNewValue, TFunctorValue, TFunctorValueVm1>  & anImage );
  // ImageAdapter  (3D)


  template < typename TImageType2D, typename TFunctor >
  static void
  drawImage2D( Viewer3D & viewer, const TImageType2D & anImage, const TFunctor & aFunctor,
               Display3D::TextureMode aTextureMode=Display3D::GrayScaleMode );


  template < typename TImageType3D, typename TFunctor >
  static void
  drawImage3D( Viewer3D & viewer, const TImageType3D & anImage3D, const TFunctor & aFunctor,
               Display3D::TextureMode aTextureMode=Display3D::GrayScaleMode );


  static void draw( Viewer3D & viewer, const DGtal::SetMode3D & aMode);

  static void draw( Viewer3D & viewer, const DGtal::CustomStyle3D & aStyle);
  static void draw( Viewer3D & viewer, const DGtal::CustomColors3D & aColor);

  static void draw( Viewer3D & viewer, const DGtal::ClippingPlane & aClipping);


  // AddTextureImage3DWithFunctor
  template<typename TImageType, typename TFunctor>
  static void draw( Viewer3D & viewer, const DGtal::AddTextureImage3DWithFunctor<TImageType, TFunctor> & aFunctor );
  // AddTextureImage3DWithFunctor

  // AddTextureImage2DWithFunctor
  template<typename TImageType, typename TFunctor>
  static void draw( Viewer3D & viewer, const DGtal::AddTextureImage2DWithFunctor<TImageType, TFunctor> & aFunctor );
  // AddTextureImage2DWithFunctor

  static void draw( Viewer3D & viewer, const DGtal::UpdateImagePosition & anUpdate);
  static void draw( Viewer3D & viewer, const DGtal::UpdateLastImagePosition & anUpdate );

  static void draw( Viewer3D &viewer, const DGtal::Update2DDomainPosition & anUpdate);
  static void draw( Viewer3D &viewer, const DGtal::Translate2DDomain & anTranslation);
  template<typename TImageType, typename TFunctor>
  static void draw( Viewer3D & viewer, const DGtal::UpdateImageData<TImageType, TFunctor> & anUpdate);

  static void draw( Viewer3D & viewer, const DGtal::TransformedSurfelPrism & aTransformedSurfelPrism);

  // end heritage
  //----------------------------------------------------------------------------------------------

}; // end of struct Viewer3DFactory

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "DGtal/io/Viewer3DFactory.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Viewer3DFactory_h

#undef Viewer3DFactory_RECURSES
#endif // else defined(Viewer3DFactory_RECURSES)
