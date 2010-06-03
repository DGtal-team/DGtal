/* -*- mode: c++ -*- */
/**
 * @file   Rect.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  
 * @copyright
 */
#ifndef _BOARD_RECT_H_
#define _BOARD_RECT_H_

#include <iostream>
#include "board/Point.h"

namespace LibBoard {

/**
 * The Rectangle structure.
 * @brief Struct representing a rectangle on the plane. 
 */
struct Rect {
  double left;			/**< Coordinate of the left side. */
  double top;			/**< Coordinate of the upper side. */
  double width;			/**< Width of the rectangle. */
  double height;		/**< Height of the rectangle. */
  
  /** 
   * Rect constructor.
   * 
   * @param left 
   * @param top 
   * @param width 
   * @param height 
   * 
   * @return 
   */
  Rect( double left = 0.0f, double top = 0.0f, double width = 0.0f, double height = 0.0f )
    :left( left ), top( top ), width( width ), height( height ) { } 

  Point topLeft() { return Point( left, top ); }
  Point topRight() { return Point( left + width, top ); }
  Point bottomLeft() { return Point( left, top - height ); }
  Point bottomRight() { return Point( left + width, top - height ); }
};

/** 
 * Computes the bounding box of two bounding boxes.
 * 
 * @param rectA A first rectangle.
 * @param rectB A second rectangle.
 * 
 * @return The smallest rectangle that contains both rectA and rectB.
 */
Rect operator||( const Rect & rectA, const Rect & rectB );

/** 
 * Computes the intersection of two bounding boxes.
 * 
 * @param rectA A first rectangle.
 * @param rectB A second rectangle.
 * 
 * @return The intersecting rectangle of two bounding boxes.
 */
Rect operator&&( const Rect & rectA, const Rect & rectB );


} // mamespace BoardLib

/** 
 * Stream output operator for Rect structure.
 * 
 * @param out An output stream.
 * @param rect A Rect struct.
 * 
 * @return The output stream.
 */
std::ostream & operator<<( std::ostream & out, const LibBoard::Rect & rect );

#endif // _RECT_H_
