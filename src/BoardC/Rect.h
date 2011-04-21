/* -*- mode: c++ -*- */
/**
 * @file   Rect.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_RECT_H_
#define _BOARDCAIRO_RECT_H_

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

} // namespace

/*
 * Inline methods
 */
//#include "Rect.ih"

#endif

