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
#include "DGtal/geometry/2d/GridCurve.h"
#include "DGtal/geometry/2d/Preimage2D.h"
#include "DGtal/geometry/2d/FP.h"
#include "DGtal/kernel/PointVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/kernel/sets/DigitalSetBySTLSet.h"
#include "DGtal/kernel/sets/DigitalSetBySTLVector.h"
#include "DGtal/topology/Object.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/math/AngleLinearMinimizer.h"

// TODO: begin
// remettre testGridCurve
// remettre testPreimage

//modified:   examples/doc-examples/kernelDomain.cpp
//ligne 127: //shift.selfDraw(board, (*itPrec));

//modified:   examples/doc-examples/khalimskySpaceScanner.cpp
//ligne91://     shift.selfDraw(boardScan1, K.uCoords(prec) );
//ligne115://    shiftq.selfDraw(boardScan2, K.uCoords(precq) );

//modified:   tests/geometry/2d/testShapesFromPoints.cpp
//ligne 110,111,112
//    c.drawArc(board, Point(5,10), Point(8,4)); 
//   c.drawSector(board, Point(9,3), Point(10,0) ); 
//    c.drawAnnulus(board, Point(5,-10), Point(2,-4) );

// test testHashTree.cpp
//tests/kernel/testHashTree.cpp:175:11: error: ‘class testGetSetVal()::Image’ has no member named ‘selfDraw’
//tests/kernel/testHashTree.cpp:178:12: error: ‘class testGetSetVal()::ImageVector’ has no member named ‘selfDraw’

// test /home/user/Desktop/DGtal.git/DGtal/tests/geometry/nd/testDistanceTransformation.cpp
//ligne 111 138 185 - 221 273 283 - 293 337 363 535 - 553 568

//home/user/Desktop/DGtal.git/DGtal/tests/io/writers/testPNMRawWriter.cpp
//ligne 99

//home/user/Desktop/DGtal.git/DGtal/examples/geometry/distancetransform2D.cpp
//ligne 99 130 135 140

// test testDigitalSet.cpp
//tests/kernel/testDigitalSet.cpp:116:27: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/kernel/testDigitalSet.cpp:122:29: error: ‘DrawDomainPaving’ was not declared in this scope
//tests/kernel/testDigitalSet.cpp:234:10: error: ‘class testDigitalSetDraw()::Domain’ has no member named ‘selfDrawAsGrid’

// test testObject.cpp
//tests/topology/testObject.cpp:485:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject.cpp:493:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject.cpp:494:15: error: ‘class testDraw()::ObjectType’ has no member named ‘selfDrawWithAdjacencies’
//tests/topology/testObject.cpp:501:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject.cpp:502:16: error: ‘class testDraw()::ObjectType84’ has no member named ‘selfDrawWithAdjacencies’

// test testObject-benchmark.cpp
//tests/topology/testObject-benchmark.cpp:482:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject-benchmark.cpp:490:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject-benchmark.cpp:491:15: error: ‘class testDraw()::ObjectType’ has no member named ‘selfDrawWithAdjacencies’
//tests/topology/testObject-benchmark.cpp:498:10: error: ‘class testDraw()::DomainType’ has no member named ‘selfDrawAsGrid’
//tests/topology/testObject-benchmark.cpp:499:16: error: ‘class testDraw()::ObjectType84’ has no member named ‘selfDrawWithAdjacencies’

// test testObjectBorder
//tests/topology/testObjectBorder.cpp:170:29: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testObjectBorder.cpp:173:36: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testObjectBorder.cpp:195:12: error: ‘class testObjectBorder()::Domain’ has no member named ‘selfDrawAsGrid’
//et ligne 197 aussi
//tests/topology/testObjectBorder.cpp:216:45: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testObjectBorder.cpp:218:45: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testObjectBorder.cpp:291:29: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testObjectBorder.cpp:298:43: error: ‘DrawObjectAdjacencies’ was not declared in this scope

// test testSimpleExpander
//tests/topology/testSimpleExpander.cpp:138:28: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:139:35: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:155:28: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:156:34: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:172:28: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:173:34: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//et lignes 189-190 aussi
//tests/topology/testSimpleExpander.cpp:267:28: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:268:35: error: ‘DrawObjectAdjacencies’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:290:27: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/topology/testSimpleExpander.cpp:291:34: error: ‘DrawObjectAdjacencies’ was not declared in this scope

// test testBoard2DCustomStyle
//tests/io/testBoard2DCustomStyle.cpp:94:27: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/io/testBoard2DCustomStyle.cpp:101:27: error: ‘DrawDomainGrid’ was not declared in this scope

// testSimpleBoard
//tests/io/testSimpleBoard.cpp:82:10: error: ‘class testSimpleBoard()::Point2D’ has no member named ‘selfDraw’
//tests/io/testSimpleBoard.cpp:106:29: error: ‘DrawDomainGrid’ was not declared in this scope
//tests/io/testSimpleBoard.cpp:111:28: error: ‘DrawDomainPaving’ was not declared in this scope


// rechercher tous les selfdraw
// revoir les // des structs ou alors c'est des #if dehors
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
    template <typename TInteger>
    static void draw( Board2D & board, const DGtal::FreemanChain<TInteger> & );
    // FreemanChain
    
    
    // GridCurve
    template <typename TKSpace>
    static void draw( Board2D & board, const GridCurve<TKSpace> & );
    
    template <typename TKSpace>
    static void draw( Board2D & board, 
		      const typename GridCurve<TKSpace>::SCellsRange & );
    

    template <typename TKSpace>
    static void draw( Board2D & board, 
		      const typename GridCurve<TKSpace>::IncidentPointsRange &);

    template <typename T>
    static void draw( Board2D & board, 
		      const T &);
    // GridCurve
    
    
    // Preimage2D
    template <typename Shape>
    static void draw( Board2D & board, const DGtal::Preimage2D<Shape> & );
    // Preimage2D
    
    
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
    template < Dimension dim, typename TInteger >
    static void draw( Board2D & board, const DGtal::KhalimskyCell<dim, TInteger> & );
    // KhalimskyCell
    
    // SignedKhalimskyCell
    template < Dimension dim, typename TInteger >
    static void draw( Board2D & board, const DGtal::SignedKhalimskyCell<dim, TInteger> & );
    // SignedKhalimskyCell
    
    // AngleLinearMinimizer
    static void draw( Board2D & board, const DGtal::AngleLinearMinimizer & );
    // AngleLinearMinimizer
    
    // FP
    template <typename TIterator, typename TInteger, int connectivity>
    static void draw( Board2D & board, const DGtal::FP<TIterator,TInteger,connectivity> & );
    // FP
    
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
