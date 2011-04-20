/* -*- mode: c++ -*- */
/**
 * @file   Point.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_POINT_H_
#define _BOARDCAIRO_POINT_H_

namespace LibBoard {

/**
 * The Point structure.
 * @brief Struct representing a 2D point. 
 */
struct Point {
  
  double x;			/**< The point's first coordinate */
  double y;			/**< The point's second coordinate */
  
  /** 
   * Point constructor.
   * 
   * @param xc The point's first coordinate.
   * @param yc The point's second coordinate.
   */
  Point( double xc, double yc ):x(xc),y(yc) { }
  
};

} // namespace

/*
 * Inline methods
 */
//#include "Point.ih"

#endif

