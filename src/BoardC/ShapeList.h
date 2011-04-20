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

#include "Shapes.h"
#include "Rect.h"

namespace LibBoard {

/**
 * The ShapeList structure.
 * @brief A list of shapes.
 */
struct ShapeList : public Shape {
  
  Rect boundingBox() const;
  
};

} // namespace

/*
 * Inline methods
 */
//#include "ShapeList.ih"

#endif

