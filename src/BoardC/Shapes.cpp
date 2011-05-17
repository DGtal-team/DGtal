/* -*- mode: c++ -*- */
/**
 * @file   Shapes.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include <assert.h>

#include "Rect.h"
#include "Shapes.h"

namespace {
#ifdef WITH_CAIRO
// cairo
cairo_line_cap_t cairoLineCap[] = {
    CAIRO_LINE_CAP_BUTT,
    CAIRO_LINE_CAP_ROUND,
    CAIRO_LINE_CAP_SQUARE
};

cairo_line_join_t cairoLineJoin[] = {
    CAIRO_LINE_JOIN_MITER,
    CAIRO_LINE_JOIN_ROUND,
    CAIRO_LINE_JOIN_BEVEL
};

#define ARRAY_SIZE(A) (sizeof(A)/sizeof(A[0]))

const double cairoSolidStyle[] = {}; const double cairoSolidStyle_offset = 0;
const double cairoDashStyle[] = {1, 1}; const double cairoDashStyle_offset = 0;
const double cairoDotStyle[] = {1.5, 4.5}; const double cairoDotStyle_offset = 45;
const double cairoDashDotStyle[] = {4.5, 2.3, 1.5, 2.3}; const double cairoDashDotStyle_offset = 0;
const double cairoDashDotDotStyle[] = {4.5, 2.0, 1.5, 1.5, 1.5, 2.0}; const double cairoDashDotDotStyle_offset = 0;
const double cairoDashDotDotDotStyle[] = {4.5, 1.8, 1.5, 1.4, 1.5, 1.4, 1.5, 1.8}; const double cairoDashDotDotDotStyle_offset = 0;
// cairo
#endif
}

//todo cairo: gouraudtriangle, text (?)
//todo cairo: group (?)

namespace LibBoard {
  
bool
shapeGreaterDepth( const Shape *s1, const Shape *s2 )
{
    return s1->depth() > s2->depth();
}
  
#ifdef WITH_CAIRO
void
Shape::setCairoDashStyle(cairo_t *cr, LineStyle type) const
{
    switch (type)
    {
      case SolidStyle:
	cairo_set_dash (cr, cairoSolidStyle, ARRAY_SIZE(cairoSolidStyle), cairoSolidStyle_offset); break;
      case DashStyle:
	cairo_set_dash (cr, cairoDashStyle, ARRAY_SIZE(cairoDashStyle), cairoDashStyle_offset); break;
      case DotStyle:
	cairo_set_dash (cr, cairoDotStyle, ARRAY_SIZE(cairoDotStyle), cairoDotStyle_offset); break;
      case DashDotStyle:
	cairo_set_dash (cr, cairoDashDotStyle, ARRAY_SIZE(cairoDashDotStyle), cairoDashDotStyle_offset); break;
      case DashDotDotStyle:
	cairo_set_dash (cr, cairoDashDotDotStyle, ARRAY_SIZE(cairoDashDotDotStyle), cairoDashDotDotStyle_offset); break;
      case DashDotDotDotStyle:
	cairo_set_dash (cr, cairoDashDotDotDotStyle, ARRAY_SIZE(cairoDashDotDotDotStyle), cairoDashDotDotDotStyle_offset); break;
	
      default: // SolidStyle
	cairo_set_dash (cr, cairoSolidStyle, ARRAY_SIZE(cairoSolidStyle), cairoSolidStyle_offset);
    }
}
#endif

Rect
Line::boundingBox() const
{
    Rect rect;
    if ( _x1 > _x2 ) {
        rect.width = _x1 - _x2;
        rect.left = _x2;
    } else {
        rect.width = _x2 - _x1;
        rect.left = _x1;
    }
    if ( _y1 > _y2 ) {
        rect.top = _y1;
        rect.height = _y1 - _y2;
    } else {
        rect.top = _y2;
        rect.height = _y2 - _y1;
    }
    return rect;
}

#ifdef WITH_CAIRO
void
Line::flushCairo( cairo_t *cr,
                const TransformCairo & transform ) const
{
    cairo_save (cr);
    
      cairo_set_source_rgba (cr, _penColor.red()/255.0, _penColor.green()/255.0, _penColor.blue()/255.0, 1.);
      
      cairo_move_to (cr, transform.mapX( _x1 ), transform.mapY( _y1 ));
      cairo_line_to (cr, transform.mapX( _x2 ), transform.mapY( _y2 ));
      
      cairo_set_line_width (cr, _lineWidth);
      cairo_set_line_cap (cr, cairoLineCap[_lineCap]);
      cairo_set_line_join (cr, cairoLineJoin[_lineJoin]);
      setCairoDashStyle (cr, _lineStyle);

      cairo_stroke (cr);
    
    cairo_restore (cr);
}
#endif

} // namespace
