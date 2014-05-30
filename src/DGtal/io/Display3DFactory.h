/**
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file Display3DFactory.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date mercredi 21 septembre 2011
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

#include "DGtal/helpers/StdDefs.h"
#include "DGtal/base/Common.h"

#include "DGtal/io/DrawWithDisplay3DModifier.h"
#include "DGtal/geometry/curves/StandardDSS6Computer.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/topology/Object.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/geometry/curves/GridCurve.h"
#include "DGtal/shapes/Mesh.h"
#include "DGtal/geometry/tools/SphericalAccumulator.h"
#include "DGtal/io/colormaps/GradientColorMap.h"
#include "DGtal/io/colormaps/HueShadeColorMap.h"
#include "DGtal/io/colormaps/CColorMap.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerBySTLMap.h"
#include "DGtal/images/ConstImageAdapter.h"
#include "DGtal/images/ImageAdapter.h"
#include "DGtal/helpers/StdDefs.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/dec/VectorField.h"
#include "DGtal/dec/KForm.h"
#include "DGtal/dec/DiscreteExteriorCalculus.h"

//
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  /////////////////////////////////////////////////////////////////////////////
  // struct Display3DFactory
  /**
   * Description of struct 'Display3DFactory' <p>
   * \brief Factory for GPL Display3D:
   */
  template <typename TSpace=Z3i::Space, typename TKSpace=Z3i::KSpace>
  struct Display3DFactory
  {

    typedef TSpace Space;
    typedef TKSpace KSpace;

    // DiscreteExteriorCalculus
    template <Dimension dim, typename TLinearAlgebraBackend, typename TInteger>
    static
    void
    draw(Display3D<Space, KSpace>& display, const DGtal::DiscreteExteriorCalculus<dim, TLinearAlgebraBackend, TInteger>& calculus);
    // DiscreteExteriorCalculus

    // KForm
    template <typename Calculus, DGtal::Order order, DGtal::Duality duality>
    static
    void
    draw(Display3D<Space, KSpace>& display, const DGtal::KForm<Calculus, order, duality>& kform, double cmap_min = 0, double cmap_max = 0);
    // KForm

    // VectorField
    template <typename Calculus, DGtal::Duality duality>
    static
    void
    draw(Display3D<Space, KSpace>& display, const DGtal::VectorField<Calculus, duality>& vector_field, const double& scale = 0.5, const double& epsilon = 1e-8);
    // VectorField

    // SphericalAccumulator
    /**
     * Display an spherical accumulator in 3D. Bin values are mapped
     * using a default HueShadeColorMap.
     *
     * @param display the display where to draw current display
     * @param accumulator the accumulator to display
     * @param shift translate vector for display purposes (default:
     * zero vector)
     * @param radius scale factor for the unit sphere radius (default:1)
     * @tparam TVector a vector model
     */
    template <typename TVector>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::SphericalAccumulator<TVector> & accumulator,
                      const typename DGtal::SphericalAccumulator<TVector>::RealVector &shift =
                      typename DGtal::SphericalAccumulator<TVector>::RealVector(0,0,0),
                      const double radius=1.0);
    // SphericalAccumulator

    // Mesh
    /**
     * @brief drawAsFaces
     * @param display the display where to draw
     * @param aMesh the mesh to draw
     */
    template <typename TPoint>
    static void drawAsFaces( Display3D<Space, KSpace> & display, const DGtal::Mesh<TPoint> & aMesh);

    /**
     * @brief draw
     * @param display the display where to draw
     * @param aMesh the mesh to draw
     */
    template <typename TPoint>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::Mesh<TPoint> & aMesh);
    // Mesh


    // StandardDSS6Computer
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template <typename TIterator, typename TInteger, int connectivity>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::StandardDSS6Computer<TIterator,TInteger,connectivity> & anObject );

    /**
     * @brief drawAsBalls
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator, typename TInteger, int connectivity>
    static void drawAsBalls( Display3D<Space, KSpace> & display, const DGtal::StandardDSS6Computer<TIterator,TInteger,connectivity> & anObject );

    /**
     * @brief drawAsBoundingBox
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator, typename TInteger, int connectivity>
    static void drawAsBoundingBox( Display3D<Space, KSpace> & display, const DGtal::StandardDSS6Computer<TIterator,TInteger,connectivity> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator, typename TInteger, int connectivity>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::StandardDSS6Computer<TIterator,TInteger,connectivity> & anObject );
    // StandardDSS6Computer

    // DigitalSetBySTLSet
    /**
     * @brief defaultStyle
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template<typename Domain, typename Compare>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::DigitalSetBySTLSet<Domain, Compare> & anObject );

    /**
     * @brief drawAsPavingTransparent
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain, typename Compare>
    static void drawAsPavingTransparent( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLSet<Domain, Compare> & anObject );

    /**
     * @brief drawAsPaving
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain, typename Compare>
    static void drawAsPaving( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLSet<Domain, Compare> & anObject );

    /**
     * @brief drawAsGrid
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain, typename Compare>
    static void drawAsGrid( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLSet<Domain, Compare> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain, typename Compare>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLSet<Domain, Compare> & anObject );
    // DigitalSetBySTLSet


    // DigitalSetBySTLVector
    /**
     * @brief Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template<typename Domain>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::DigitalSetBySTLVector<Domain> & anObject );

    /**
     * @brief drawAsPavingTransparent
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain>
    static void drawAsPavingTransparent( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLVector<Domain> & anObject );

    /**
     * @brief drawAsPaving
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain>
    static void drawAsPaving( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLVector<Domain> & anObject );

    /**
     * @brief drawAsGrid
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain>
    static void drawAsGrid( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLVector<Domain> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<typename Domain>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::DigitalSetBySTLVector<Domain> & anObject );
    // DigitalSetBySTLVector


    // HyperRectDomain
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template <typename SpaceDom>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::HyperRectDomain<SpaceDom> & anObject );

    /**
     * @brief drawAsBoundingBox
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename SpaceDom>
    static void drawAsBoundingBox( Display3D<Space, KSpace> & display, const DGtal::HyperRectDomain<SpaceDom> & anObject );

    /**
     * @brief drawAsGrid
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename SpaceDom>
    static void drawAsGrid( Display3D<Space, KSpace> & display, const DGtal::HyperRectDomain<SpaceDom> & anObject );

    /**
     * @brief drawAsPavingBalls
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename SpaceDom>
    static void drawAsPavingBalls( Display3D<Space, KSpace> & display, const DGtal::HyperRectDomain<SpaceDom> & anObject );

    /**
     * @brief drawAsPaving
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename SpaceDom>
    static void drawAsPaving( Display3D<Space, KSpace> & display, const DGtal::HyperRectDomain<SpaceDom> & anObject );


    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename SpaceDom>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::HyperRectDomain<SpaceDom> & anObject );


    // HyperRectDomain


    // KhalimskyCell
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const typename KSpace::Cell & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void draw( Display3D<Space, KSpace> & display, const typename KSpace::Cell & anObject );
    // KhalimskyCell


    // Object
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template <typename TDigitalTopology, typename TDigitalSet>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

    /**
     * @brief drawWithAdjacencies
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TDigitalTopology, typename TDigitalSet>
    static void drawWithAdjacencies( Display3D<Space, KSpace> & display, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TDigitalTopology, typename TDigitalSet>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::Object<TDigitalTopology, TDigitalSet> & anObject );
    // Object


    // PointVector
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    template<Dimension dim, typename TComponent>
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const DGtal::PointVector<dim,TComponent> & anObject );

    /**
     * @brief drawAsGrid
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<Dimension dim, typename TComponent>
    static void drawAsGrid( Display3D<Space, KSpace> & display, const DGtal::PointVector<dim,TComponent> & anObject );

    /**
     * @brief drawAsPaving
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<Dimension dim, typename TComponent>
    static void drawAsPaving( Display3D<Space, KSpace> & display, const DGtal::PointVector<dim,TComponent> & anObject );

    /**
     * @brief drawAsPavingWired
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<Dimension dim, typename TComponent>
    static void drawAsPavingWired( Display3D<Space, KSpace> & display, const DGtal::PointVector<dim,TComponent> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<Dimension dim, typename TComponent>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::PointVector<dim,TComponent> & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template<Dimension dim, typename TComponent>
    static void draw( Display3D<Space, KSpace> & display, const DGtal::PointVector<dim,TComponent> & , const DGtal::PointVector<dim,TComponent> & anObject );
    // PointVector


    // SignedKhalimskyCell
    /**
     * Default drawing style object.
     * @param str the name of the class
     * @param anObject the object to draw
     * @return the dyn. alloc. default style for this object.
     */
    static DGtal::DrawableWithDisplay3D * defaultStyle( std::string str, const typename KSpace::SCell & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void draw( Display3D<Space, KSpace> & display, const typename KSpace::SCell & anObject );
    // SignedKhalimskyCell

    // GridCurve
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void draw( Display3D<Space, KSpace> & display, const DGtal::GridCurve<KSpace> & anObject );
    // GridCurve

    // SCellsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template < typename TIterator, typename TSCell>
    static void draw( DGtal::Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, DGtal::DefaultFunctor, TSCell> & anObject );
    // SCellsRange

    // PointsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator>
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, SCellToPoint<KSpace>, typename TKSpace::Point> & anObject );
    // PointsRange

    // MidPointsRange
    template <typename TIterator>
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, CanonicSCellEmbedder<KSpace>,
                      typename TKSpace::Space::RealPoint> & anObject );
    // MidPointsRange

    // ArrowsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator>
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, SCellToArrow<KSpace>,
                      std::pair<typename TKSpace::Point, typename TKSpace::Vector > > & anObject );
    // ArrowsRange

    // InnerPointsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator>
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, SCellToInnerPoint<KSpace>, typename TKSpace::Point> & anObject );
    // InnerPointsRange

    // OuterPointsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator>
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, SCellToOuterPoint<KSpace>, typename TKSpace::Point> & anObject );
    // OuterPointsRange

    // IncidentPointsRange
    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    template <typename TIterator>
    static void draw( Display3D<Space, KSpace> & display,
                      const DGtal::ConstRangeAdapter<TIterator, SCellToIncidentPoints<KSpace>,
                      std::pair<typename TKSpace::Point, typename TKSpace::Point > > & anObject );
    // IncidentPointsRange

    /**
     *  draw.This function will create new
     * sublists for Display3D models.
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void
    draw( Display3D<Space, KSpace> & display, const DGtal::SetMode3D & anObject );

    /**
     * Draw for CustomStyle class. This function will create new
     * sublists for Display3D models.
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void
    draw( Display3D<Space, KSpace> & display, const DGtal::CustomStyle3D & anObject );

    /**
     * brief draw.This function will create new
     * sublists for Display3D models.
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void
    draw( Display3D<Space, KSpace> & display, const DGtal::CustomColors3D & anObject );

    /**
     * @brief draw
     * @param display the display where to draw
     * @param anObject the object to draw
     */
    static void
    draw( Display3D<Space, KSpace> & display, const DGtal::ClippingPlane & anObject );


    /**
     * Draw a surfel
     * @param display the display where to draw
     * @param aTransformedPrism a transformed surfel prism
     */
    static void
    draw( Display3D<Space, KSpace> & display, const DGtal::TransformedPrism & aTransformedPrism);

  }; // end of struct Display3DFactory

} // namespace DGtal

///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods

#include "DGtal/io/Display3DFactory.ih"

// //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display3DFactory_h

#undef Display3DFactory_RECURSES
#endif // else defined(Display3DFactory_RECURSES)
