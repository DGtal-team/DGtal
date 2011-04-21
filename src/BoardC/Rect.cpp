/* -*- mode: c++ -*- */
/**
 * @file   Rect.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include "Rect.h"

namespace LibBoard {

Rect
operator||( const Rect & rectA, const Rect & rectB )
{
  Rect rect;
  rect.top = ( rectA.top > rectB.top ) ? rectA.top : rectB.top;
  rect.left = (rectA.left < rectB.left) ? rectA.left : rectB.left;
  if ( rectA.left + rectA.width > rectB.left + rectB.width )
    rect.width = rectA.left + rectA.width - rect.left;
  else
    rect.width = rectB.left + rectB.width - rect.left;
  if ( rectA.top - rectA.height < rectB.top - rectB.height )
    rect.height = rect.top - ( rectA.top - rectA.height );
  else
    rect.height = rect.top - ( rectB.top - rectB.height );
  return rect;
}

} // namespace
