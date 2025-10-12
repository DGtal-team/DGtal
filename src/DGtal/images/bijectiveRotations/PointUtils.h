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
* @file PointUtils.h
 * @author S. Breuils, J.O. Lachaud, D. Coeurjolly
 *
 * @date 2024/07/9
 *
 * This file is part of the DGtal library.
 */

#if defined(PointUtils_RECURSES)
#error Recursive header files inclusion detected in PointUtils.h
#else // defined(PointUtils_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PointUtils_RECURSES

#if !defined PointUtils_h
/** Prevents repeated inclusion of headers. */
#define PointUtils_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <vector>
#include "DGtal/base/Common.h"

namespace DGtal{
 /// @return the determinant of p and q, seen as 2D vectors
 template<typename TPointType>
 typename TPointType::Coordinate det( const TPointType& p, const TPointType& q ) {
  return p[ 0 ] * q[ 1 ] - p[ 1 ] * q[ 0 ];
 }

 /// Defines an order between points, which is
 /// p < q <=> less( p, q ) <=> det( p, q ) > 0
 template<typename TPointType>
 bool less( const TPointType& p, const TPointType& q ) {
  return ( p[ 0 ] * q[ 1 ] - p[ 1 ] * q[ 0 ] ) > 0;
 }



 /// Defines an order between points, which is
 /// p < q <=> less( p, q ) <=> det( p, q ) > 0
 template<typename TPointType>
 bool less( const TPointType& p, const TPointType& q );

 /// @param p any point
 /// @return the points (q_i) that are 8-connected to p and such that
 /// `less( p, q_i)`
 template<typename TPointType>
 std::vector< TPointType > nextNeighbors( TPointType p ) {
  std::vector< TPointType > V;
  TPointType zero( 0, 0 );
  TPointType p0( p[ 0 ]+1, p[ 1 ] );
  if ( less( p, p0 ) ) V.push_back( p0 );
  TPointType p1( p[ 0 ]+1, p[ 1 ]+1 );
  if ( less( p, p1 ) ) V.push_back( p1 );
  TPointType p2( p[ 0 ], p[ 1 ]+1 );
  if ( less( p, p2 ) ) V.push_back( p2 );
  TPointType p3( p[ 0 ]-1, p[ 1 ]+1 );
  if ( less( p, p3 ) ) V.push_back( p3 );
  TPointType p4( p[ 0 ]-1, p[ 1 ] );
  if ( less( p, p4 ) ) V.push_back( p4 );
  TPointType p5( p[ 0 ]-1, p[ 1 ]-1 );
  if ( less( p, p5 ) ) V.push_back( p5 );
  TPointType p6( p[ 0 ], p[ 1 ]-1 );
  if ( less( p, p6 ) ) V.push_back( p6 );
  TPointType p7( p[ 0 ]+1, p[ 1 ]-1 );
  if ( less( p, p7 ) ) V.push_back( p7 );
  return V;
 }

 /// @return the squared distance between points p and q
 template<typename TPointType>
 typename TPointType::Coordinate distance2( TPointType p, TPointType q ) {
  typename TPointType::Coordinate d2 = (p[ 0 ] - q[ 0 ])*(p[ 0 ] - q[ 0 ])+(p[ 1 ] - q[ 1 ])*(p[ 1 ] - q[ 1 ]);
  return d2;
 }



 /// @return the squared distance between points p and q
 template<typename TPointType1,typename TPointType2>
  typename TPointType2::Coordinate distance2( TPointType1 p, TPointType2 q ) {
  typename TPointType2::Coordinate d2 = (p[ 0 ] - q[ 0 ])*(p[ 0 ] - q[ 0 ])+(p[ 1 ] - q[ 1 ])*(p[ 1 ] - q[ 1 ]);
  return d2;
 }


}
#endif //PointUtils
#undef PointUtils_RECURSES
#endif // else defined(PointUtils_RECURSES)