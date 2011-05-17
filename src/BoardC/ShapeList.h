/* -*- mode: c++ -*- */
/**
 * @file   ShapeList.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_SHAPELIST_H_
#define _BOARDCAIRO_SHAPELIST_H_

#include <vector>
#include <limits>

#include "Shapes.h"
#include "Rect.h"
#include "Color.h"

namespace LibBoard {

/**
 * The ShapeList structure.
 * @brief A list of shapes.
 */
struct ShapeList : public Shape {
  
  inline ShapeList( int depth = -1 );
  
  ~ShapeList();
  
  ShapeList & clear();
  
  Rect boundingBox() const;
  
#ifdef WITH_CAIRO
  void flushCairo( cairo_t *cr,
		 const TransformCairo & transform ) const;
#endif
  
protected:
  
  std::vector<Shape*> _shapes;	/**< The vector of shapes. */
  int _nextDepth;		/**< The depth of the next figure to be added.  */
  
  /** 
   * Free the memory used by the shapes in the shape vector.
   */
  void free();
  
};

ShapeList::ShapeList( int depthValue )
  : Shape( Color::None, Color::None, 1.0, SolidStyle, ButtCap, MiterJoin, depthValue ),
    _nextDepth( std::numeric_limits<int>::max() - 1 )
{ }

} // namespace

/*
 * Inline methods
 */
//#include "ShapeList.ih"

#endif

