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

#ifdef WITH_CAIRO
// cairo
#include <cairo.h>
#endif

#include "Rect.h"
#include "Color.h"
#include "Transforms.h"

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
		   
  /** 
   * Shape constructor.
   * 
   * @param penColor The pen color of the shape.
   * @param fillColor The fill color of the shape.
   * @param lineWidth The line thickness.
   * @param depth The depth of the shape.
   */
  inline Shape( Color penColor, Color fillColor,
		double lineWidth, 
		LineStyle style,
		const LineCap cap,
		const LineJoin join,
		int depth );
  
  /** 
   * Returns the bounding box of the figure.
   *
   * @return The rectangle of the bounding box.
   */
  virtual Rect boundingBox() const = 0;
  
#ifdef WITH_CAIRO
  /** 
   * Writes the cairo code of the shape in a cairo drawing context according
   * to a transform.
   * 
   * @param cr The cairo drawing context.
   * @param transform A 2D transform to be applied.
   */
  virtual void flushCairo( cairo_t *cr,
			 const TransformCairo & transform ) const = 0;
#endif

  inline int depth() const;
  
protected:

  int _depth;    		/**< The depth of the shape. */
  Color _penColor;		/**< The color of the shape. */
  Color _fillColor;		/**< The color of the shape. */
  double _lineWidth;		/**< The line thickness. */
  LineStyle _lineStyle;		/**< The line style (solid, dashed, etc.). */
  LineCap _lineCap;		/**< The linecap attribute. (The way line terminates.) */
  LineJoin _lineJoin;		/**< The linejoin attribute. (The shape of line junctions.) */

#ifdef WITH_CAIRO
  // cairo
  /** 
   * Set a cairo dash style.
   * @param cr The cairo drawing context.
   * @param type LineStyle: SolidStyle, DashStyle, DotStyle, DashDotStyle, DashDotDotStyle, DashDotDotDotStyle.
   */
  void setCairoDashStyle(cairo_t *cr, LineStyle type) const;
#endif
};

Shape::Shape( Color pen,
	      Color fill,
	      double lineWidth, 
	      LineStyle style,
	      const LineCap cap,
	      const LineJoin join,
	      int depthValue )
  : _depth( depthValue ),
    _penColor( pen ),
    _fillColor( fill ), 
    _lineWidth( lineWidth ),
    _lineStyle( style ),
    _lineCap( cap ), 
    _lineJoin( join )
{
}

inline int
Shape::depth() const
{
  return _depth;
}

/** 
 * Compares two shapes according to their depths.
 * 
 * @param s1 A pointer to a first shape.
 * @param s2 A pointer to a second shape.
 * 
 * @return 
 */
bool shapeGreaterDepth( const Shape *s1, const Shape *s2 );

/**
 * The line structure.
 * @brief A line between two points.
 */
struct Line : public Shape { 
  
  /** 
   * Constructs a line.
   * 
   * @param x1 First coordinate of the start point.
   * @param y1 Second coordinate of the start point.
   * @param x2 First coordinate of the end point.
   * @param y2 Second coordinate of the end point.
   * @param color The color of the line.
   * @param lineWidth The line thickness.
   * @param depth The depth of the line.
   */
  inline Line( double x1, double y1, double x2, double y2, 
	       Color color, 
	       double lineWidth,
	       const LineStyle style = SolidStyle,
	       const LineCap cap = ButtCap,
	       const LineJoin join = MiterJoin,
	       int depth = -1 );
  
  Rect boundingBox() const;
  
#ifdef WITH_CAIRO
  void flushCairo( cairo_t *cr,
		 const TransformCairo & transform ) const;
#endif

protected:
  double _x1;			/**< First coordinate of the start point. */
  double _y1;			/**< Second coordinate of the start point. */
  double _x2; 			/**< First coordinate of the end point. */
  double _y2;			/**< Second coordinate of the end point. */
};

Line::Line( double x1, double y1, double x2, double y2, 
	    Color color, 
	    double lineWidth,
	    const LineStyle style,
	    const LineCap cap,
	    const LineJoin join,
	    int depthValue )
  : Shape( color, Color::None, lineWidth, style, cap, join, depthValue ),
    _x1( x1 ), _y1( y1 ), _x2( x2 ), _y2( y2 )
{
}

} // namespace

/*
 * Inline methods
 */
//#include "Shapes.ih"

#endif

