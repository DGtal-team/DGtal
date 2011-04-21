/* -*- mode: c++ -*- */
/**
 * @file   ShapeList.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include <algorithm>

#include "ShapeList.h"

namespace LibBoard {

ShapeList::~ShapeList()
{
    free();
}

ShapeList &
ShapeList::clear()
{
    free();
    _shapes.clear();
    _nextDepth = std::numeric_limits<int>::max() - 1;
    return *this;
}

void
ShapeList::free()
{
    std::vector<Shape*>::const_iterator i = _shapes.begin();
    std::vector<Shape*>::const_iterator end = _shapes.end();
    while ( i != end ) {
        delete *i;
        ++i;
    }
}

Rect
ShapeList::boundingBox() const
{
    Rect r;
    std::vector< Shape* >::const_iterator i = _shapes.begin();
    std::vector< Shape* >::const_iterator end = _shapes.end();
    if ( i == end ) return r;
    r = (*i)->boundingBox();
    ++i;
    while ( i != end ) {
        r = r || (*i)->boundingBox();
        ++i;
    }
    return r;
}

#ifdef WITH_CAIRO
void
ShapeList::flushCairo( cairo_t *cr/*,
		 const TransformCairo & transform*/ ) const // TODO
{
    std::vector< Shape* > shapes = _shapes;
    stable_sort( shapes.begin(), shapes.end(), shapeGreaterDepth );
    std::vector< Shape* >::const_iterator i = shapes.begin();
    std::vector< Shape* >::const_iterator end = shapes.end();

    while ( i != end ) {
        (*i)->flushCairo( cr/*, transform*/ ); // TODO
        ++i;
    }
}
#endif

} // namespace
