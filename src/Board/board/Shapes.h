/* -*- mode: c++ -*- */
/**
 * @file   Shapes.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  
 * @copyright
 */
#ifndef _BOARD_SHAPES_H_
#define _BOARD_SHAPES_H_

#include "board/Point.h"
#include "board/Rect.h"
#include "board/Path.h"
#include "board/Color.h"
#include "board/Transforms.h"
#include "board/PSFonts.h"
#include "board/Tools.h"
#include <string>
#include <vector>
#include <iostream>
#include <map>
#include <cmath>

#ifndef M_PI
#define M_PI           3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2         1.57079632679489661923
#endif

namespace LibBoard {

/**
 * Shape structure.
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
   * Shape destructor.
   */
  virtual ~Shape() { }

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  virtual const std::string & name() const;

  /** 
   * Return a copy of the shape.
   * 
   * @return 
   */
  virtual Shape * clone() const = 0;
  
  /** 
   * Checks whether a shape is filled with a color or not.
   * 
   * @return true if the shape is filled.
   */
  inline bool filled() const { return _fillColor != Color::None; }
  
  /** 
   * Returns the gravity center of the shape.
   * 
   * @return The center of the shape.
   */
  virtual Point center() const = 0;
  
  /** 
   * Rotate the shape around a given center of rotation.
   * 
   * @param angle The rotation angle in radian.
   * @param center The center of rotation.
   * 
   * @return A reference to the shape itself.
   */
  virtual Shape & rotate( double angle, const Point & center ) = 0;

  /** 
   * Rotate the shape around its center.
   * 
   * @param angle The rotation angle in radian.
   * 
   * @return A reference to the shape itself.
   */
  virtual Shape & rotate( double angle ) = 0;

  /** 
   * Rotate the shape around a given center of rotation.
   * 
   * @param angle The rotation angle in degree.
   * @param center The center of rotation.
   * 
   * @return A reference to the shape itself.
   */
  inline Shape & rotateDeg( double angle, const Point & center );
  
  /** 
   * Rotate the shape around its center.
   * 
   * @param angle The rotation angle in degree.
   * @param center The center of rotation.
   * 
   * @return A reference to the shape itself.
   */
  inline Shape & rotateDeg( double angle );

  /** 
   * Translate the shape by a given offset.
   * 
   * @param dx The x offset.
   * @param dy The y offset.
   * 
   * @return A reference to the shape itself.
   */
  virtual Shape & translate( double dx, double dy ) = 0;

  /** 
   * Scale the shape along the x an y axis.
   * 
   * @param sx The scale factor along the x axis.
   * @param sy The scale factor along the y axis.
   * 
   * @return The shape itself.
   */
  virtual Shape & scale( double sx, double sy ) = 0;

  /** 
   * Scale the shape along both axis.
   * 
   * @param s The scale factor along both axis.
   * 
   * @return The shape itself.
   */
  virtual Shape & scale( double s ) = 0;
 
  /** 
   * Returns the bounding box of the figure.
   *
   * @return The rectangle of the bounding box.
   */
  virtual Rect boundingBox() const = 0;

  
  /** 
   * Returns the bounding box of the figure. (Convenience method to call "boundingBox" with a short name.)
   * 
   */
  inline Rect bbox();
  

  /** 
   * Decrement the depth of the shape. (Pull the shape toward the foreground.)
   * 
   * @return 
   */
  inline Shape & operator--();

  /** 
   * Increment the depth of the shape. (Push the shape toward the background.)
   * 
   * @return 
   */
  inline Shape & operator++();


  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  virtual void scaleAll( double s ) = 0;


  /** 
   * Writes the EPS code of the shape in a stream according
   * to a transform.
   * 
   * @param stream The output stream.
   * @param transform A 2D transform to be applied.
   */
  virtual void flushPostscript( std::ostream & stream,
				const TransformEPS & transform ) const = 0;

  /** 
   * Writes the FIG code of the shape in a stream according
   * to a transform.
   * 
   * @param stream The output stream.
   * @param transform A 2D transform to be applied.
   */
  virtual void flushFIG( std::ostream & stream,
			 const TransformFIG & transform,
			 std::map<Color,int> & colormap ) const = 0;

  /** 
   * Writes the SVG code of the shape in a stream according
   * to a transform.
   * 
   * @param stream The output stream.
   * @param transform A 2D transform to be applied.
   */
  virtual void flushSVG( std::ostream & stream,
			 const TransformSVG & transform ) const = 0;

  inline int depth() const;

  virtual void depth( int );

  virtual void shiftDepth( int shift );
  
  inline const Color & penColor() const;

  inline const Color & fillColor() const;

private:
  static const std::string _name; /**< The generic name of the shape. */
  
protected:

  int _depth;    		/**< The depth of the shape. */
  Color _penColor;		/**< The color of the shape. */
  Color _fillColor;		/**< The color of the shape. */
  double _lineWidth;	/**< The line thickness. */
  LineStyle _lineStyle;	/**< The line style (solid, dashed, etc.). */
  LineCap _lineCap;		/**< The linecap attribute. (The way line terminates.) */
  LineJoin _lineJoin;	/**< The linejoin attribute. (The shape of line junctions.) */

  /** 
   * Return a string of the svg properties lineWidth, opacity, penColor, fillColor,
   * lineCap, and lineJoin.
   * 
   * @return A string of the properties suitable for inclusion in an svg tag.
   */
  std::string svgProperties( const TransformSVG & transform ) const;

  
  /** 
   * Return a string of the properties lineWidth, penColor, lineCap, and lineJoin
   * as Postscript commands.
   * @return A string of the Postscript commands.
   */
  std::string postscriptProperties() const;

};

  
inline Rect
Shape::bbox()
{
  return this->boundingBox();
}
  
  
inline Shape &
Shape::operator++()
{
  ++_depth;
  return *this;
}

inline Shape &
Shape::operator--()
{
  --_depth;
  return *this;
}


inline int
Shape::depth() const
{
  return _depth;
}

inline const Color &
Shape::penColor() const
{
  return _penColor;
}

const Color &
Shape::fillColor() const
{
  return _fillColor;
}

Shape &
Shape::rotateDeg( double angle, const Point & center )
{
  return rotate( angle * ( M_PI / 180.0 ), center ); 
}

Shape &
Shape::rotateDeg( double angle )
{
  return rotate( angle * ( M_PI / 180.0 ), center() ); 
}

/**
 * The dot structure. This primitive ha been reported as "missing" by
 * Manuel Peinado from Spain. Manuel also suggested
 * that rendering a dot consists in drawing a line
 * from a point to the same point with a roundcap style.
 * 
 * @brief A line between two points.
 */
struct Dot : public Shape { 
  
  inline Dot( double x, double y,
	      Color color, 
	      double lineWidth,
	      int depth = -1 );

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;
  
  Point center() const;

  /** 
   * Rotates the dot around a given center of rotation.
   * 
   * @param angle The rotation angle.
   * @param center The center of rotation.
   * 
   * @return A reference to the Dot itself.
   */
  Dot & rotate( double angle, const Point & center );

  /** 
   * Returns a rotated copy of the dot around a given rotation center.
   * 
   * @param angle Th rotation angle.
   * @param center The center of rotation.
   * 
   * @return A rotated copy of the Dot.
   */
  Dot rotated( double angle, const Point & center ) const;

  /** 
   * Rotates the dot around its center (actually does nothing!).
   * 
   * @param angle The rotation angle.
   * 
   * @return A reference to the Dot itself.
   */
  Dot & rotate( double angle );

  /** 
   * Returns a copy of the dot rotated around its center (i.e. left unchanged!)
   * 
   * @param angle The rotation angle.
   * 
   * @return A copy of the dot.
   */
  Dot rotated( double angle ) const;

  /** 
   * 
   * 
   * @param dx 
   * @param dy 
   * 
   * @return 
   */
  Dot & translate( double dx, double dy );

  /** 
   * Returns a translated copy of the dot.
   * 
   * @param dx Shift of the first coordinate.
   * @param dy Shift of the second coordinate.
   * 
   * @return A copy of the Dot, translated.
   */
  Dot translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  /** 
   * Returns a scaled copy of the dot, i.e. the dot itself. 
   * (Dots are not actually scaled since their diameters is given by
   * the line width.)
   * 
   * @param sx The scale factor around the x axis.
   * @param sy The scale factor around the y axis.
   * 
   * @return A scaled copy of the Dot, i.e. the Dot itself.
   */
  Dot scaled( double sx, double sy ) const;

  Dot scaled( double s ) const;

  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;
  
  Rect boundingBox() const;

  Dot * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  double _x;			/**< First coordinate of the dot. */
  double _y;			/**< Second coordinate of the dot. */
};

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

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  Line & rotate( double angle, const Point & center );

  /** 
   * Returns a copy of the line, rotated around a given rotation center.
   * 
   * @param angle The rotation angle.
   * @param center The center of rotation.
   * 
   * @return The rotated copy of the line.
   */
  Line rotated( double angle, const Point & center ) const;

  Line & rotate( double angle );

  /** 
   * Returns a copy of the line, rotated around its center.
   * 
   * @param angle The angle of rotation.
   * 
   * @return A copy of the lines, rotated around its center by the given angle.
   */
  Line rotated( double angle ) const;

  Line & translate( double dx, double dy );

  /** 
   * Returns a translated copy of the line.
   * 
   * @param dx The shift along the x axis.
   * @param dy The shift along the y axis.
   * 
   * @return A translated copy of the line.
   */
  Line translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  /** 
   * Returns a scaled copy of the line.
   * 
   * @param sx Scale factor along the x axis.
   * @param sy Scale factor along the y axis.
   * 
   * @return A scaled copy of the line.
   */
  Line scaled( double sx, double sy ) const;

  Line scaled( double s ) const;
  
  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rect boundingBox() const;

  Line * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  double _x1;			/**< First coordinate of the start point. */
  double _y1;			/**< Second coordinate of the start point. */
  double _x2; 			/**< First coordinate of the end point. */
  double _y2;			/**< Second coordinate of the end point. */
};

/**
 * The arrow structure.
 * @brief A line between two points with an arrow at one extremity.
 */
struct Arrow : public Line { 

  /** 
   * Constructs an arrow.
   * 
   * @param x1 First coordinate of the start point.
   * @param y1 Second coordinate of the start point.
   * @param x2 First coordinate of the end point.
   * @param y2 Second coordinate of the end point.
   * @param penColor The color of the line.
   * @param fillColor The fill color of the sharp end.
   * @param lineWidth The line thickness.
   * @param depth The depth of the line.
   */
  inline Arrow( double x1, double y1, double x2, double y2,
		Color penColor, Color fillColor,
		double lineWidth, 
		const LineStyle style = SolidStyle,
		const LineCap cap = ButtCap,
		const LineJoin join = MiterJoin,
		int depth = -1 );

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;
  
  /** 
   * Returns a copy of the arrow, rotated around a given rotation center.
   * 
   * @param angle The rotation angle.
   * @param center The center of rotation.
   * 
   * @return The rotated copy of the line.
   */
  Arrow rotated( double angle, const Point & center ) const;
  
  /** 
   * Returns a copy of the arrow, rotated around its center.
   * 
   * @param angle The angle of rotation.
   * 
   * @return A copy of the lines, rotated around its center by the given angle.
   */
  Arrow rotated( double angle ) const;

 /** 
   * Returns a translated copy of the arrow.
   * 
   * @param dx The shift along the x axis.
   * @param dy The shift along the y axis.
   * 
   * @return A translated copy of the line.
   */
  Arrow translated( double dx, double dy ) const;

  /** 
   * Returns a scaled copy of the arrow.
   * 
   * @param sx Scale factor along the x axis.
   * @param sy Scale factor along the y axis.
   * 
   * @return A scaled copy of the arrow.
   */
  Arrow scaled( double sx, double sy ) const;

  Arrow scaled( double s ) const;

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;
  
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;
  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Arrow * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */
};

/**
 * The polyline structure.
 * @brief A polygonal line described by a series of 2D points.
 */
struct Polyline : public Shape { 
  inline Polyline( const std::vector<Point> & points, 
		   bool closed,
		   Color penColor, Color fillColor,
		   double lineWidth,
		   const LineStyle lineStyle = SolidStyle,
		   const LineCap cap = ButtCap,
		   const LineJoin join = MiterJoin,
		   int depth = -1 );

  inline Polyline( const Path & path, 
		   Color penColor, Color fillColor,
		   double lineWidth,
		   const LineStyle lineStyle = SolidStyle,
		   const LineCap cap = ButtCap,
		   const LineJoin join = MiterJoin,
		   int depth = -1 );

  inline Polyline( bool closed, Color penColor, Color fillColor,
		   double lineWidth,
		   const LineStyle lineStyle = SolidStyle,
		   const LineCap cap = ButtCap,
		   const LineJoin join = MiterJoin,
		   int depth = -1 );
	    
  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  /** 
   * Add a point to the polyline.
   * 
   * @param p 
   * 
   * @return 
   */
  Polyline & operator<<( const Point & p );

  /** 
   * Returns the n-th point of the polyline.
   * 
   * @param i 
   * 
   * @return 
   */
  Point & operator[]( const unsigned int n ) {
    return _path[ n ];
  }


  Polyline & rotate( double angle, const Point & center );

  /** 
   * 
   * 
   * @param angle 
   * @param center 
   * 
   * @return 
   */
  Polyline rotated( double angle, const Point & center ) const;

  Polyline & rotate( double angle );
  
  /** 
   * 
   * 
   * @param angle 
   * 
   * @return 
   */
  Polyline rotated( double angle ) const;

  Polyline & translate( double dx, double dy );
  
  /** 
   * 
   * 
   * @param dx 
   * @param dy 
   * 
   * @return 
   */
  Polyline translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );
  
  /** 
   * 
   * 
   * @param sx 
   * @param sy 
   * 
   * @return 
   */
  Polyline scaled( double sx, double sy )  const;

  Polyline scaled( double s )  const;

  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;

  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rect boundingBox() const;

  Polyline * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  Path _path;
};

/**
 * The rectangle structure.
 * @brief A rectangle.
 */
struct Rectangle : public Polyline {

  inline Rectangle( double x, double y, double width, double height,
	     Color penColor, Color fillColor,
	     double lineWidth, 
	     const LineStyle style = SolidStyle,
	     const LineCap cap = ButtCap,
	     const LineJoin join = MiterJoin,
		    int depth = -1 );

  inline Rectangle( const Rect & rect,
		    Color penColor, Color fillColor,
		    double lineWidth,
		    const LineStyle style = SolidStyle,
		    const LineCap cap = ButtCap,
		    const LineJoin join = MiterJoin,
		    int depth = -1 );

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  double x() const { return _path[0].x; }
  double y() const { return _path[0].y; }
  double width() { return (_path[1] - _path[0]).norm(); }
  double height() { return (_path[0] - _path[3]).norm(); }
  Point topLeft() { return Point( _path[0].x, _path[0].y ); }
  Point topRight() { return Point( _path[1].x, _path[1].y ); }
  Point bottomLeft() { return Point( _path[3].x, _path[3].y ); }
  Point bottomRight() { return Point( _path[2].x, _path[2].y ); }


  /** 
   * Returns a copy of the arrow, rotated around a given rotation center.
   * 
   * @param angle The rotation angle.
   * @param center The center of rotation.
   * 
   * @return The rotated copy of the line.
   */
  Rectangle rotated( double angle, const Point & center )  const;
  
  /** 
   * Returns a copy of the arrow, rotated around its center.
   * 
   * @param angle The angle of rotation.
   * 
   * @return A copy of the lines, rotated around its center by the given angle.
   */
  Rectangle rotated( double angle ) const;

 /** 
   * Returns a translated copy of the arrow.
   * 
   * @param dx The shift along the x axis.
   * @param dy The shift along the y axis.
   * 
   * @return A translated copy of the line.
   */
  Rectangle translated( double dx, double dy ) const;

  /** 
   * Returns a scaled copy of the arrow.
   * 
   * @param sx Scale factor along the x axis.
   * @param sy Scale factor along the y axis.
   * 
   * @return A scaled copy of the arrow.
   */
  Rectangle scaled( double sx, double sy ) const;

  Rectangle scaled( double s ) const;
 
  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rectangle * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  bool _isRectangle;
};


/**
 * The Triangle structure.
 * @brief A triangle. Basically a Polyline with a convenient constructor.
 */
struct Triangle : public Polyline {

  Triangle( const Point & p1, const Point & p2, const Point & p3,
	    Color penColor, Color fillColor,
	    double lineWidth,
	    const LineStyle style = SolidStyle,
	    const LineCap cap = ButtCap,
	    const LineJoin join = MiterJoin,
	    int depth = -1 )
    : Polyline( std::vector<Point>(), true, penColor, fillColor, lineWidth, style, cap, join, depth ) {
    _path << p1;
    _path << p2;
    _path << p3;
  }

  Triangle( const double x1, const double y1, 
	    const double x2, const double y2, 
	    const double x3, const double y3,
	    Color penColor, Color fillColor,
	    double lineWidth,
	    const LineStyle style = SolidStyle,
	    const LineCap cap = ButtCap,
	    const LineJoin join = MiterJoin,
	    int depth = -1 )
    : Polyline( std::vector<Point>(), true, penColor, fillColor, lineWidth, style, cap, join, depth ) {
    _path << Point( x1, y1 );
    _path << Point( x2, y2 );
    _path << Point( x3, y3 );
  }

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Triangle rotated( double angle ) const;

 /** 
   * Returns a translated copy of the triangle.
   * 
   * @param dx The shift along the x axis.
   * @param dy The shift along the y axis.
   * 
   * @return A translated copy of the line.
   */
  Triangle translated( double dx, double dy ) const;

  /** 
   * Returns a scaled copy of the triangle.
   * 
   * @param sx Scale factor along the x axis.
   * @param sy Scale factor along the y axis.
   * 
   * @return A scaled copy of the arrow.
   */
  Triangle scaled( double sx, double sy ) const;

  Triangle scaled( double s ) const;

  Triangle * clone() const;
  
private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
};


/**
 * The GouraudTriangle structure.
 * @brief A triangle with shaded filling according to colors given for each vertex. 
 */
struct GouraudTriangle : public Polyline {


  GouraudTriangle( const Point & p0, const Color & color0,
		   const Point & p1, const Color & color1,
		   const Point & p2, const Color & color2,
		   int subdivisions,
		   int depth = -1 );

  GouraudTriangle( const Point & p0, float brightness0,
		   const Point & p1, float brightness1,
		   const Point & p2, float brightness2,
		   const Color & fillColor,
		   int subdivisions,
		   int depth = -1 );

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  GouraudTriangle & rotate( double angle, const Point & center );  

  GouraudTriangle rotated( double angle, const Point & center ) const;  

  GouraudTriangle & rotate( double angle );

  GouraudTriangle rotated( double angle ) const;

 /** 
   * Returns a translated copy of the triangle.
   * 
   * @param dx The shift along the x axis.
   * @param dy The shift along the y axis.
   * 
   * @return A translated copy of the line.
   */
  GouraudTriangle translated( double dx, double dy ) const;

  /** 
   * Returns a scaled copy of the triangle.
   * 
   * @param sx Scale factor along the x axis.
   * @param sy Scale factor along the y axis.
   * 
   * @return A scaled copy of the arrow.
   */
  GouraudTriangle scaled( double sx, double sy ) const;

  GouraudTriangle scaled( double s ) const;


  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  /** 
   * Sends the triangle to a Postscript document.
   * 
   * @param stream
   * @param transform 
   */
  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;

  /** 
   * Sends the Triangle to a FIG file format stream.
   * <p><b>Warning!</b> Because shading would generally require
   * more colors in the colormap than allowed by the FIG file format, 
   * rendering a Gouraud triangle in an XFig file is the same as rendering
   * a simple triangle whose filling color is the average of the vertex colors.
   * 
   * @param stream 
   * @param transform 
   * @param Color 
   * @param colormap 
   */
  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  GouraudTriangle * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  Color _color0;
  Color _color1;
  Color _color2;
  int _subdivisions;
};

/**
 * The ellipse structure.
 * @brief An ellipse.
 */
struct Ellipse : public Shape {
  
  Ellipse( double x, double y, 
	   double xRadius, double yRadius, 
	   Color penColor, Color fillColor,
	   double lineWidth,
	   const LineStyle lineStyle = SolidStyle,
	   int depth = -1 )
    : Shape( penColor, fillColor,
	     lineWidth, lineStyle, ButtCap, MiterJoin, depth ),
      _center( x, y ), _xRadius( xRadius ), _yRadius( yRadius ),
      _angle( 0.0 ),
      _circle( false ) {
    while ( _angle > M_PI_2 ) _angle -= M_PI;
    while ( _angle < -M_PI_2 ) _angle += M_PI;
  }

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  Ellipse & rotate( double angle, const Point & center );

  /** 
   * 
   * 
   * @param angle 
   * @param center 
   * 
   * @return 
   */
  Ellipse rotated( double angle, const Point & center ) const;

  Ellipse & rotate( double angle );

  /** 
   * 
   * 
   * @param angle 
   * 
   * @return 
   */
  Ellipse rotated( double angle ) const;
  
  Ellipse & translate( double dx, double dy );

  /** 
   * 
   * 
   * @param dx 
   * @param dy 
   * 
   * @return 
   */
  Ellipse translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  /** 
   * 
   * 
   * @param sx 
   * @param sy 
   * 
   * @return 
   */
  Ellipse scaled( double sx, double sy ) const;

  Ellipse scaled( double s ) const;
  
  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;

  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rect boundingBox() const;

  Ellipse * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */
 
protected:
  Point _center;
  double _xRadius; 
  double _yRadius;
  double _angle;
  bool _circle;
};

/**
 * The circle structure.
 * @brief A circle.
 */
struct Circle : public Ellipse {

  Circle( double x, double y, double radius, 
	  Color penColor, Color fillColor,
	  double lineWidth,
	  const LineStyle style = SolidStyle,
	  int depth = -1 )
    : Ellipse( x, y, radius, radius, penColor, fillColor, lineWidth, style, depth )
  { _circle = true; }

  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  Circle & rotate( double angle, const Point & center );

  Circle rotated( double angle, const Point & center ) const;

  Circle & rotate( double angle );

  Circle rotated( double angle ) const;
  
  Circle & translate( double dx, double dy );

  Circle translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  Circle scaled( double sx, double sy ) const;

  Circle scaled( double s ) const;

  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Circle * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */
};

/**
 * The text structure.
 * @brief A piece of text.
 */
struct Text : public Shape {
  
  /** 
   * Create a Text sctucture.
   * 
   * @param x 
   * @param y 
   * @param text 
   * @param font
   * @param size The font size expressed in 1/72 inches.
   * @param color 
   * @param depth 
   * 
   * @return 
   */
  Text( double x, double y,
	const std::string & text,
	const Fonts::Font font,
	double size,
	Color color = Color::Black,
	int depth = -1 )
    : Shape( color, Color::None, 1.0, SolidStyle, ButtCap, MiterJoin, depth ),
      _position( x, y ), _text( text ), _font( font ),
      _angle( 0.0 ), _size( size ),
      _xScale( 1.0 ), _yScale( 1.0 ) { }

  
  /** 
   * Create a Text sctucture.
   * 
   * @param x 
   * @param y 
   * @param text 
   * @param font
   * @param svgFont The font family for an SVG file. (E.g. "Verdana, Arial" or "'Time New Roman', Serif" )
   * @param size The font size expressed in 1/72 inches.
   * @param color 
   * @param depth 
   * 
   * @return 
   */
  Text( double x, double y,
	const std::string & text,
	const Fonts::Font font,
	const std::string & svgFont,
	double size,
	Color color = Color::Black,
	int depth = -1 )
    : Shape( color, Color::None, 1.0, SolidStyle, ButtCap, MiterJoin, depth ),
      _position( x, y ),
      _text( text ), _font( font ), _svgFont( svgFont ),
      _angle( 0.0 ),
      _size( size ),
      _xScale( 1.0 ), _yScale( 1.0 ) { }
  
  /** 
   * Returns the generic name of the shape (e.g., Circle, Rectangle, etc.)
   * 
   * @return 
   */
  const std::string & name() const;

  Point center() const;

  Text & rotate( double angle, const Point & center );

  Text rotated( double angle, const Point & center ) const;

  Text & rotate( double angle );

  Text rotated( double angle ) const;
  
  Text & translate( double dx, double dy );

  Text translated( double dx, double dy ) const;

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  Text scaled( double sx, double sy ) const;
 
  Text scaled( double s ) const;
  
  /** 
   * Scales all the values (positions, dimensions, etc.) associated
   * with the shape.
   * 
   * @param s The scaling factor.
   */
  void scaleAll( double s );

  void flushPostscript( std::ostream & stream,
			const TransformEPS & transform ) const;

  void flushFIG( std::ostream & stream,
		 const TransformFIG & transform,
		 std::map<Color,int> & colormap ) const;

  void flushSVG( std::ostream & stream,
		 const TransformSVG & transform ) const;

  Rect boundingBox() const;

  Text * clone() const;

private:
  static const std::string _name; /**< The generic name of the shape. */

protected:
  Point _position;
  std::string _text;
  Fonts::Font _font;
  std::string _svgFont;
  double _angle;
  double _size;
  double _xScale;
  double _yScale;
};

/** 
 * Compares two shapes according to their depths.
 * 
 * @param s1 A pointer to a first shape.
 * @param s2 A pointer to a second shape.
 * 
 * @return 
 */
bool shapeGreaterDepth( const Shape *s1, const Shape *s2 );


} // namespace LibBoard

/*
 * Inline methods
 */
#include "Shapes.ih"


#endif /* _SHAPE_H_ */

