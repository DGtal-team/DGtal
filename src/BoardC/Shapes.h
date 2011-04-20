/* -*- mode: c++ -*- */
/**
 * @file   Shapes.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_SHAPES_H_
#define _BOARDCAIRO_SHAPES_H_

namespace LibBoard {

/**
 * The Shape structure.
 * @brief Abstract structure for a 2D shape.
 */
struct Shape {
  
  enum LineCap { ButtCap = 0, RoundCap, SquareCap };
  enum LineJoin { MiterJoin = 0, RoundJoin, BevelJoin };
  enum LineStyle { SolidStyle = 0,
		   DashStyle, 
		   DotStyle,
		   DashDotStyle,
		   DashDotDotStyle,
		   DashDotDotDotStyle };
		   
};

} // namespace

/*
 * Inline methods
 */
//#include "Shapes.ih"

#endif

