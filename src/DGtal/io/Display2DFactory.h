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

#include "DGtal/geometry/2d/ArithmeticalDSS.h"
#include "DGtal/geometry/2d/FreemanChain.h"
//#include "DGtal/geometry/2d/GridCurve.h"
#include "DGtal/geometry/2d/Preimage2D.h"
#include "DGtal/geometry/2d/FP.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/math/AngleLinearMinimizer.h"

#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/images/ImageContainerByHashTree.h"

#include "DGtal/geometry/2d/GeometricalDSS.h"

#include "DGtal/io/boards/Board2D.h"


// TODO: begin

// --- VERIF --- :
// test testDigitalSet.cpp
// test testObject.cpp
// test testObject-benchmark.cpp
// test testObjectBorder
// test testSimpleExpander
// test testBoard2DCustomStyle

// /tests/io/testSimpleBoard --> draw(board, apoint, p2) ??? ligne 82

//modified:   ../../../../src/DGtal/geometry/2d/GridCurveRanges.ih
//ligne 835: //shift.selfDraw( aBoard, pair.first );

//modified:   examples/doc-examples/kernelDomain.cpp
//ligne 127: //shift.selfDraw(board, (*itPrec));

//modified:   examples/doc-examples/khalimskySpaceScanner.cpp
//ligne91://     shift.selfDraw(boardScan1, K.uCoords(prec) );
//ligne115://    shiftq.selfDraw(boardScan2, K.uCoords(precq) );

//---

// --- àfaire --- :
// rechercher tous les selfdraw et defaultStyle
// revoir les // des structs ou alors c'est des #if dehors

//---

// --- DEMANDER A DC --- :
//modified:   tests/geometry/2d/testShapesFromPoints.cpp
//ligne 110,111,112
//    c.drawArc(board, Point(5,10), Point(8,4)); 
//   c.drawSector(board, Point(9,3), Point(10,0) ); 
//    c.drawAnnulus(board, Point(5,-10), Point(2,-4) );
// TODO: end


// TODO new: begin
// 1) -> todo: accessors
// 3) -> DigitalSetBySTLVector: draw with Functor ??? necessary ???

// 4) types de David

// 5) pb avec les flèches (ne se dessinent pas)
// TODO new: end


// TODO new2: begin
// drawImage *2 au lieu de draw pour ImageContainerBySTLVector et pour ImageContainerByHashTree
// ImageContainerByHashTree: pb Node (ligne 590 à 599) de DisplaYFactory.ih

// ranger dans l'ordre les new style et draw

// IMPORTANT -> revoir tous les drawImage
// TODO new2: end


//////////////////////////////////////////////////////////////////////////////


//namespace DGtal
//{

  /////////////////////////////////////////////////////////////////////////////
  // struct Display2DFactory
  /**
   * Description of struct 'Display2DFactory' <p>
   * \brief Factory for Display2D:
   */
  //  namespace Display2DFactory
  // {
   
    // ArithmeticalDSS
    template <typename TIterator, typename TInteger, int connectivity=8>
    void drawAsBoundingBox( DGtal::Board2D & aBoard, const DGtal::ArithmeticalDSS<TIterator,TInteger,connectivity> & );

    template <typename TIterator, typename TInteger, int connectivity=8>
    void drawAsDigitalPoints( DGtal::Board2D & aBoard, const DGtal::ArithmeticalDSS<TIterator,TInteger,connectivity> & );

    template <typename TIterator, typename TInteger, int connectivity=8>
    void draw( DGtal::Board2D & board, const DGtal::ArithmeticalDSS<TIterator,TInteger,connectivity> & );
    // ArithmeticalDSS
    
    
    // FreemanChain
    template <typename TInteger>
    void drawAsGrid( DGtal::Board2D & aBoard, const DGtal::FreemanChain<TInteger> & );
    
    template <typename TInteger>
    void drawAsInterGrid( DGtal::Board2D & aBoard, const DGtal::FreemanChain<TInteger> & );
    
    template <typename TInteger>
    void draw( DGtal::Board2D & aBoard, const DGtal::FreemanChain<TInteger> & );
    // FreemanChain
    
    
    // GridCurve
    // we use selfDraw because of inner classes
    // GridCurve
    
    
    // Preimage2D
    template <typename Shape>
    void draw( DGtal::Board2D & aBoard, const DGtal::Preimage2D<Shape> & );
    // Preimage2D
    
    
    // PointVector
    template<Dimension dim, typename TComponent>
    void drawAsPaving( DGtal::Board2D & board, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    void drawAsGrid( DGtal::Board2D & board, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    void draw( DGtal::Board2D & board, const DGtal::PointVector<dim,TComponent> & );

    template<Dimension dim, typename TComponent>
    void draw( DGtal::Board2D & board, const DGtal::PointVector<dim,TComponent> &, const DGtal::PointVector<dim,TComponent> & );
    // PointVector
    
    
    // HyperRectDomain
    template<typename TSpace>
    void drawAsGrid( DGtal::Board2D & aboard, const DGtal::HyperRectDomain<TSpace> & );

    template<typename TSpace>
    void drawAsPaving( DGtal::Board2D & aboard, const DGtal::HyperRectDomain<TSpace> & );

    template<typename TSpace>
    void draw( DGtal::Board2D & board, const DGtal::HyperRectDomain<TSpace> & );
    // HyperRectDomain
    
    
    // DigitalSetBySTLSet
    template<typename Domain>
    void draw( DGtal::Board2D & board, const DGtal::DigitalSetBySTLSet<Domain> & );
    // DigitalSetBySTLSet
    
    
    // DigitalSetBySTLVector
    template<typename Domain>
    void draw( DGtal::Board2D & board, const DGtal::DigitalSetBySTLVector<Domain> & );
    // DigitalSetBySTLVector
    
    
    // Object
    template <typename TDigitalTopology, typename TDigitalSet>
    void drawWithAdjacencies( DGtal::Board2D & aBoard, const DGtal::Object<TDigitalTopology, TDigitalSet> & );

    template <typename TDigitalTopology, typename TDigitalSet>
    void draw( DGtal::Board2D & board, const DGtal::Object<TDigitalTopology, TDigitalSet> & );
    // Object
    
    
    // KhalimskyCell
    template < Dimension dim, typename TInteger >
    void draw( DGtal::Board2D & board, const DGtal::KhalimskyCell<dim, TInteger> & );
    // KhalimskyCell
    
    // SignedKhalimskyCell
    template < Dimension dim, typename TInteger >
    void draw( DGtal::Board2D & board, const DGtal::SignedKhalimskyCell<dim, TInteger> & );
    // SignedKhalimskyCell
    
    
    // AngleLinearMinimizer
    void draw( DGtal::Board2D & board, const DGtal::AngleLinearMinimizer & );
    // AngleLinearMinimizer
    
    
    // FP
    template <typename TIterator, typename TInteger, int connectivity>
    void drawAsPolygon( DGtal::Board2D & aBoard, const DGtal::FP<TIterator,TInteger,connectivity> & );

    template <typename TIterator, typename TInteger, int connectivity>
    void draw( DGtal::Board2D & board, const DGtal::FP<TIterator,TInteger,connectivity> & );
    // FP
  
    
    // ImageContainerByHashTree
    template <typename C, typename Domain, typename Value, typename HashKey>
    void drawImageRecursive( DGtal::Board2D & aBoard, const experimental::ImageContainerByHashTree<Domain, Value, HashKey> & i,
        HashKey key,
        const double p[2],
        const double len,
        LibBoard::Board & board,
        const C& cmap );

    template <typename C, typename Domain, typename Value, typename HashKey>
    void drawImage( Board2D & board, const experimental::ImageContainerByHashTree<Domain, Value, HashKey> &, const Value &, const Value & );
    // ImageContainerByHashTree


    // ImageContainerBySTLVector
    template <typename Colormap, typename D, typename V>
    void drawImage( DGtal::Board2D & board, const DGtal::ImageContainerBySTLVector<D, V> &, const V &, const V & );
    // ImageContainerBySTLVector
    
    
    // GeometricalDSS
    template <typename TConstIterator>
    void draw(DGtal::Board2D & aBoard, const DGtal::GeometricalDSS<TConstIterator> & );
    // GeometricalDSS
    
    //
    
    void draw( DGtal::Board2D & board, const DGtal::SetMode & );
    void draw( DGtal::Board2D & board, const DGtal::CustomStyle & );
    // void draw( Display3D & display, const DGtal::CustomColors3D & );

  //  }; // end of struct Display2DFactory

  //} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions/methods 
#include "DGtal/io/Display2DFactory.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Display2DFactory_h

#undef Display2DFactory_RECURSES
#endif // else defined(Display2DFactory_RECURSES)
