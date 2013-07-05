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
 * @file   Board3DTo2DFactory.h
 * @author Aline Martin <aline.martin@insa-lyon.fr>
 * @date   mardi 2 juillet 2013
 *
 * @brief
 *
 * Header file for module Board3DTo2DFactory
 *
 * This file is part of the DGtal library.
 */

#if defined(Board3DTo2DFactory_RECURSES)
#error Recursive header files inclusion detected in Board3DTo2DFactory.h
#else // defined(Board3DTo2DFactory_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Board3DTo2DFactory_RECURSES

#if !defined Board3DTo2DFactory_h
/** Prevents repeated inclusion of headers. */
#define Board3DTo2DFactory_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include "DGtal/io/Display3DFactory.h"
#include "DGtal/io/DrawWithBoard3DTo2DModifier.h"
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
// struct Board3DTo2DFactory
/**
   * Description of struct 'Board3DTo2DFactory' <p>
   * \brief Factory for GPL Display3D:
   */

struct Board3DTo2DFactory : public Display3DFactory
{


  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CameraPosition & );

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CameraDirection & );

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CameraUpVector & );

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CameraZNearFar & );


  //----------------------------------------------------------------------------------------------
  // heritage
  // SphericalAccumulator
  /**
       * Display an spherical accumulator in 3D. Bin values are mapped
       * using a default HueShadeColorMap.
       *
       * @param board current board
       * @param accumulator the accumulator to board
       * @param shift translate vector for board purposes (default:
       * zero vector)
       * @param radius scale factor for the unit sphere radius (default:1)
       * @tparam TVector a vector model
       */
  template <typename TVector, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  DGtal::SphericalAccumulator<TVector> & accumulator,
                    const typename DGtal::SphericalAccumulator<TVector>::RealVector & shift =
      typename DGtal::SphericalAccumulator<TVector>::RealVector(0,0,0),
                    const double radius=1.0);
  // SphericalAccumulator

  // Mesh
  template <typename TPoint, typename S, typename KS>
  static void
  drawAsFaces( Board3DTo2D<S, KS> & board, const DGtal::Mesh<TPoint> & aMesh );

  template <typename TPoint, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  DGtal::Mesh<TPoint> & aMesh );
  // Mesh



  // ArithmeticalDSS3d
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template <typename TIterator, typename TInteger, int connectivity>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity, typename S, typename KS>
  static void
  drawAsBalls( Board3DTo2D<S, KS> & board, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity, typename S, typename KS>
  static void
  drawAsBoundingBox( Board3DTo2D<S, KS> & board, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );

  template <typename TIterator, typename TInteger, int connectivity, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::ArithmeticalDSS3d<TIterator,TInteger,connectivity> & arithm );
  // ArithmeticalDSS3d


  // DigitalSetBySTLSet
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template<typename Domain>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsPavingTransparent( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsPaving( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsGrid( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLSet<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLSet<Domain> & aSet );
  // DigitalSetBySTLSet


  // DigitalSetBySTLVector
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template<typename Domain>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsPavingTransparent( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsPaving( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  drawAsGrid( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLVector<Domain> & aSet );

  template<typename Domain, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::DigitalSetBySTLVector<Domain> & aSet );
  // DigitalSetBySTLVector


  // HyperRectDomain
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template<typename TSpace>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace, typename S, typename KS>
  static void
  drawAsBoundingBox( Board3DTo2D<S, KS> & board, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace, typename S, typename KS>
  static void
  drawAsGrid( Board3DTo2D<S, KS> & board, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace, typename S, typename KS>
  static void
  drawAsPavingBalls( Board3DTo2D<S, KS> & board, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace, typename S, typename KS>
  static void
  drawAsPaving( Board3DTo2D<S, KS> & board, const DGtal::HyperRectDomain<TSpace> & aDomain );

  template<typename TSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::HyperRectDomain<TSpace> & aDomain );
  // HyperRectDomain


  // KhalimskyCell
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template < Dimension dim, typename TInteger>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::KhalimskyCell<dim, TInteger> & aCell );

  template < Dimension dim, typename TInteger, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::KhalimskyCell<dim, TInteger> & aCell );
  // KhalimskyCell


  // Object
  template <typename TDigitalTopology, typename TDigitalSet>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

  template <typename TDigitalTopology, typename TDigitalSet, typename S, typename KS>
  static void
  drawWithAdjacencies( Board3DTo2D<S, KS> & board, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

  template <typename TDigitalTopology, typename TDigitalSet, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );
  // Object


  // PointVector
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template<Dimension dim, typename TComponent>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent, typename S, typename KS>
  static void
  drawAsGrid( Board3DTo2D<S, KS> & board, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent, typename S, typename KS>
  static void
  drawAsPaving( Board3DTo2D<S, KS> & board, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent, typename S, typename KS>
  static void
  drawAsPavingWired( Board3DTo2D<S, KS> & board, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::PointVector<dim,TComponent> & aPoint );

  template<Dimension dim, typename TComponent, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::PointVector<dim,TComponent> & aPoint, const DGtal::PointVector<dim,TComponent> & aPoint2 );
  // PointVector


  // SignedKhalimskyCell
  /**
       * Default drawing style object.
       * @return the dyn. alloc. default style for this object.
       */
  template< Dimension dim, typename TInteger>
  static DGtal::DrawableWithBoard3DTo2D *
  defaultStyle( std::string str, const DGtal::SignedKhalimskyCell<dim, TInteger> & aSCell );

  template< Dimension dim, typename TInteger, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::SignedKhalimskyCell<dim, TInteger> & aSCell );
  // SignedKhalimskyCell

  // GridCurve
  template< typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::GridCurve<TKSpace> & aGrid );
  // GridCurve

  // SCellsRange
  template < typename TIterator, typename TSCell, typename S, typename KS>
  static void
  draw( DGtal::Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, DGtal::DefaultFunctor, TSCell> & aRangeAdapter );
  // SCellsRange

  // PointsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // PointsRange

  // MidPointsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToMidPoint<TKSpace>, typename TKSpace::Space::RealPoint> & aRangeAdapter );
  // MidPointsRange

  // ArrowsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToArrow<TKSpace>, std::pair<typename TKSpace::Point, typename TKSpace::Vector > > & aRangeAdapter );
  // ArrowsRange

  // InnerPointsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToInnerPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // InnerPointsRange

  // OuterPointsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToOuterPoint<TKSpace>, typename TKSpace::Point> & aRangeAdapter );
  // OuterPointsRange

  // IncidentPointsRange
  template <typename TIterator, typename TKSpace, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board,
                    const DGtal::ConstRangeAdapter<TIterator, SCellToIncidentPoints<TKSpace>,std::pair<typename TKSpace::Point, typename TKSpace::Point > > & aRangeAdapter );
  // IncidentPointsRange


  // ImageContainerBySTLVector  (2D)
  template <typename TValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ImageContainerBySTLVector<DGtal::Z2i::Domain, TValue>  & anImage );
  // ImageContainerBySTLVector  (2D)

  // ImageContainerBySTLMap  (2D)
  template <typename TValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ImageContainerBySTLMap<DGtal::Z2i::Domain, TValue>  & anImage );
  // ImageContainerBySTLMap  (2D)

  // ConstImageAdapter  (2D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ConstImageAdapter<TImageContainer, DGtal::Z2i::Domain, TFunctorD, TNewValue, TFunctorValue>  & anImage );
  // ConstImageAdapter  (2D)

  // ImageAdapter  (2D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename TFunctorValueVm1, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ImageAdapter<TImageContainer, DGtal::Z2i::Domain, TFunctorD,
                    TNewValue, TFunctorValue, TFunctorValueVm1>  & anImage );
  // ImageAdapter  (2D)


  // ImageContainerBySTLVector  (3D)
  template <typename TValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const   ImageContainerBySTLVector<DGtal::Z3i::Domain, TValue>  & anImage );
  // ImageContainerBySTLVector (3D)


  // ImageContainerBySTLMap  (3D)
  template <typename TValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const   ImageContainerBySTLMap<DGtal::Z3i::Domain, TValue>  & anImage );
  // ImageContainerBySTLMap  (3D)

  // ConstImageAdapter  (3D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ConstImageAdapter<TImageContainer, DGtal::Z3i::Domain, TFunctorD,
                    TNewValue, TFunctorValue>  & anImage );
  // ConstImageAdapter  (3D)

  // ImageAdapter  (3D)
  template <typename TImageContainer, typename TFunctorD, typename TNewValue, typename TFunctorValue, typename TFunctorValueVm1, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const  ImageAdapter<TImageContainer, DGtal::Z3i::Domain, TFunctorD,
                    TNewValue, TFunctorValue, TFunctorValueVm1>  & anImage );
  // ImageAdapter  (3D)


  template < typename TImageType2D, typename TFunctor, typename S, typename KS>
  static void
  drawImage2D( Board3DTo2D<S, KS> & board, const TImageType2D & anImage, const TFunctor & aFunctor,
               typename Board3DTo2D<S, KS>::TextureMode aTextureMode=Board3DTo2D<S, KS>::GrayScaleMode );


  template < typename TImageType3D, typename TFunctor, typename S, typename KS>
  static void
  drawImage3D( Board3DTo2D<S, KS> & board, const TImageType3D & anImage3D, const TFunctor & aFunctor,
               typename Board3DTo2D<S, KS>::TextureMode aTextureMode=Board3DTo2D<S, KS>::GrayScaleMode );

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::SetMode3D & aMode);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CustomStyle3D & aStyle);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::CustomColors3D & aColor);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::ClippingPlane & aClipping);


  // AddTextureImage3DWithFunctor
  template<typename TImageType, typename TFunctor, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::AddTextureImage3DWithFunctor<TImageType, TFunctor, S, KS> & aFunctor );
  // AddTextureImage3DWithFunctor

  // AddTextureImage2DWithFunctor
  template<typename TImageType, typename TFunctor, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::AddTextureImage2DWithFunctor<TImageType, TFunctor, S, KS> & aFunctor );
  // AddTextureImage2DWithFunctor

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::UpdateImagePosition<S, KS> & anUpdate);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::UpdateLastImagePosition<S, KS> & anUpdate );

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> &board, const DGtal::Update2DDomainPosition<S, KS> & anUpdate);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> &board, const DGtal::Translate2DDomain & anTranslation);

  template<typename TImageType, typename TFunctor, typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::UpdateImageData<TImageType, TFunctor> & anUpdate);

  template <typename S, typename KS>
  static void
  draw( Board3DTo2D<S, KS> & board, const DGtal::TransformedSurfelPrism & aTransformedSurfelPrism);

  // end heritage
  //----------------------------------------------------------------------------------------------

}; // end of struct Board3DTo2DFactory

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods
#include "DGtal/io/Board3DTo2DFactory.ih"


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Board3DTo2DFactory_h

#undef Board3DTo2DFactory_RECURSES
#endif // else defined(Board3DTo2DFactory_RECURSES)
