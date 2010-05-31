/* -*- mode: c++ -*- */
/**
 * @file   Rect.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  
 * @copyright
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
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
  float left;			/**< Coordinate of the left side. */
  float top;			/**< Coordinate of the upper side. */
  float width;			/**< Width of the rectangle. */
  float height;		/**< Height of the rectangle. */
  
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
  Rect( float left = 0.0f, float top = 0.0f, float width = 0.0f, float height = 0.0f )
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
