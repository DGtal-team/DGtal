/* -*- mode: c++ -*- */
/**
 * @file   Board.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  
 * @copyright
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 * This source code is part of the Board project, a C++ library whose
 * purpose is to allow simple drawings in EPS, FIG or SVG files.
 * Copyright (C) 2007 Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb/>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>. 
 */
#ifndef _BOARD_BOARD_H_
#define _BOARD_BOARD_H_

#include <ostream>
#include <string>
#include <vector>

#include "board/Point.h"
#include "board/Path.h"
#include "board/Shapes.h"
#include "board/ShapeList.h"

namespace LibBoard {

/** 
 * The Board class.
 * @brief Class for EPS, FIG or SVG drawings.
 * @version 0.5
 */
class Board : public ShapeList {
  
public:

  enum PageSize { BoundingBox, A4, Letter };
  enum Unit { PT, IN, CM, MM };
  static const double Degree;
  
  /** 
   * Constructs a new board and sets the background color, if any.
   * 
   * @param backgroundColor A color for the drawing's background.
   */
  Board( const Color & backgroundColor = Color::None );

  /** 
   * Copy constructor.
   * 
   * @param other The object to be copied.
   */
  Board( const Board & other );

  ~Board();

  /** 
   * The operator =
   * 
   * @param other The object to be copied.
   * 
   * @return A reference to the left operand.
   */
  Board & operator=( const Board & other );


  /** 
   * Add a shape to the board, using the current unit factor.
   * 
   * @param shape A shape. (Might be a list of shapes, actually!)
   * 
   * @return The board itself, as a ShapeList.
   */
  Board & operator<<( const Shape & shape );

  /** 
   * Overloaded operator to set the current unit.
   * 
   * @param unit The unit to be used in next drawing functions and shape insertions.
   * 
   * @return The board itself, as a ShapeList.
   */
  Board & operator<<( Unit unit );
  
  /** 
   * Clears the board with a given background color.
   * 
   * @param color The board background color (may be Color::None).
   */
  void clear( const Color & color = Color::None );

  /** 
   * Clears the board and set the background color from an RGB triple.
   * 
   * @param red 
   * @param green 
   * @param blue 
   */
  inline void clear( unsigned char red, unsigned char green, unsigned char blue );



  Shape & rotate( double angle, const Point & center );

  Shape & rotate( double angle );

  Shape & translate( double dx, double dy );

  Shape & scale( double sx, double sy );

  Shape & scale( double s );

  Board rotated( double angle, const Point & center );

  Board rotated( double angle );

  Board translated( double dx, double dy );

  Board scaled( double sx, double sy );

  Board scaled( double s );



  /** 
   * Set the unit used by the drawSomething methods.
   * 
   * @param unit The unit to be used in { PT, IN, CM, MM }.
   */
  void setUnit( Unit unit );

  /** 
   * Set the unit used by the drawSomething methods.
   * 
   * @param factor The factor of the unit.
   * @param unit The unit to be used in { PT, IN, CM, MM }.
   */
  void setUnit( float factor, Unit unit );

  /** 
   * Draws a dot at coordinates (x,y).
   * 
   * @param x First coordinate of the dot.
   * @param y Second coordinate of the dot.
   * @param depth Depth of the line.
   */
  void drawDot( float x, float y, int depth = -1 );

  /** 
   * Draws a line from (x1,y1) to (x2,y2).
   * 
   * @param x1 First coordinate of the first extremity.
   * @param y1 Second coordinate of the first extremity.
   * @param x2 First coordinate of the second extremity.
   * @param y2 Second coordinate of the second extremity.
   * @param depth Depth of the line.
   */
  void drawLine( float x1, float y1, float x2, float y2, 
		 int depth = -1 );

  /** 
   * Draws a line from (x1,y1) to (x2,y2) with an arrow at (x2,y2).
   * 
   * @param x1 First coordinate of the first extremity.
   * @param y1 Second coordinate of the first extremity.
   * @param x2 First coordinate of the second extremity.
   * @param y2 Second coordinate of the second extremity.
   * @param filled Whether or not the arrow is filled.
   * @param depth Depth of the line.
   */
  void drawArrow( float x1, float y1, float x2, float y2,
		  bool filled = true,
		  int depth = -1 );

  /** 
   * Draws a triangle.
   * 
   * @param x1 First coordinate of the first vertex.
   * @param y1 Second coordinate of the first vertex.
   * @param x2 First coordinate of the second vertex.
   * @param y3 Second coordinate of the second vertex.
   * @param x3 First coordinate of the third vertex.
   * @param y3 Second coordinate of the third vertex.
   * @param depth Depth of the triangle.
   */
  void drawTriangle( float x1, float y1, 
		     float x2, float y2, 
		     float x3, float y3, 
		     int depth = -1 );

  /** 
   * Draws a triangle.
   * 
   * @param p1 First vertex.
   * @param p2 Second vertex.
   * @param p3 Third vertex.
   * @param depth Depth of the triangle.
   */
  void drawTriangle( const Point & p1,
		     const Point & p2, 
		     const Point & p3, 
		     int depth = -1 );

  /** 
   * Draws a filled triangle.
   * 
   * @param x1 First coordinate of the first vertex.
   * @param y1 Second coordinate of the first vertex.
   * @param x2 First coordinate of the second vertex.
   * @param y3 Second coordinate of the second vertex.
   * @param x3 First coordinate of the third vertex.
   * @param y3 Second coordinate of the third vertex.
   * @param depth Depth of the triangle.
   */
  void fillTriangle( float x1, float y1, 
		     float x2, float y2, 
		     float x3, float y3, 
		     int depth = -1 );

  /** 
   * Draws a triangle with Gouraud-like shaded colors. 
   * 
   * @param p1 
   * @param color1 
   * @param p2 
   * @param color2 
   * @param p3 
   * @param color3 
   * @param divisions number of triangle subdivisions.
   * @param depth The depth of the triangle.
   */
  void fillGouraudTriangle( const Point & p1,
			    const Color & color1,
			    const Point & p2,
			    const Color & color2,
			    const Point & p3,
			    const Color & color3,
			    unsigned char divisions = 3,
			    int depth = -1 );

  /** 
   * Draws a triangle with Gouraud-like shaded colors. 
   * 
   * @param x1 
   * @param y1 
   * @param color1 
   * @param x2 
   * @param y2 
   * @param color2 
   * @param x3 
   * @param y3 
   * @param color3 
   * @param divisions 
   * @param depth 
   */
  inline void fillGouraudTriangle( const float x1, const float y1,
				   const Color & color1,
				   const float x2, const float y2, 
				   const Color & color2,
				   const float x3, const float y3,
				   const Color & color3,
				   unsigned char divisions = 3,
				   int depth = -1 );

  /** 
   * Draws a triangle with a Gouraud-like shaded color according to
   * the current fill color and a brightness value given for each vertex. 
   * @param p1 
   * @param brightness1
   * @param p2 
   * @param brightness2 
   * @param p3 
   * @param brightness3
   * @param divisions number of triangle subdivisions.
   * @param depth The depth of the triangle.
   */
  void fillGouraudTriangle( const Point & p1,
			    const float brightness1,
			    const Point & p2,
			    const float brightness2,
			    const Point & p3,
			    const float brightness3,
			    unsigned char divisions = 3,
			    int depth = -1 );

  /** 
   * Draws a triangle with a Gouraud-like shaded color according to
   * the current fill color and a brightness value given for each vertex. 
   * 
   * @param x1 
   * @param y1 
   * @param brightness1
   * @param x2 
   * @param y2 
   * @param brightness2
   * @param x3 
   * @param y3 
   * @param brightness3
   * @param divisions
   * @param depth 
   */
  inline void fillGouraudTriangle( const float x1, const float y1,
				   const float brightness1,
				   const float x2, const float y2, 
				   const float brightness2,
				   const float x3, const float y3,
				   const float brightness3,
				   unsigned char divisions = 3,
				   int depth = -1 );


  /** 
   * Draws a filled triangle.
   * 
   * @param p1 First vertex.
   * @param p2 Second vertex.
   * @param p3 Third vertex.
   * @param depth Depth of the triangle.
   */
  void fillTriangle( const Point & p1,
		     const Point & p2, 
		     const Point & p3, 
		     int depth = -1 );
  
  /** 
   * Draws a rectangle.
   * 
   * @param x First coordinate of the upper left corner.
   * @param y Second coordinate of the upper left corner.
   * @param width Width of the rectangle.
   * @param height Height of the rectangle.
   * @param depth Depth of the rectangle.
   */
  void drawRectangle( float x, float y, 
		      float width, float height,
		      int depth = -1 );

  /** 
   * Draws a rectangle filled with the current pen color.
   * 
   * @param x First coordinate of the upper left corner.
   * @param y Second coordinate of the upper left corner.
   * @param width Width of the rectangle.
   * @param height Height of the rectangle.
   * @param depth Depth of the rectangle.
   */
  void fillRectangle( float x, float y,
		      float width, float height,
		      int depth = -1 );

  /** 
   * Draws a circle.
   * 
   * @param x First coordinate of the circle's center.
   * @param y Second coordinate of the circle's center.
   * @param radius Radius of the circle.
   * @param depth Depth of the circle.
   */
  void drawCircle( float x, float y, float radius,
		   int depth = -1 );
 
  /** 
   * Draws a circle filled with the current pen color.
   * 
   * @param x First coordinate of the circle's center.
   * @param y Second coordinate of the circle's center.
   * @param radius Radius of the circle.
   * @param depth Depth of the circle.
   */
  void fillCircle( float x, float y, float radius,
		   int depth = -1);

  /** 
   * Draws an ellipse.
   * 
   * @param x First coordinate of the circle's center.
   * @param y Second coordinate of the circle's center.
   * @param radius Radius of the circle.
   * @param depth Depth of the circle.
   */
  void drawEllipse( float x, float y, 
		    float xRadius, float yRadius,
		    int depth = -1);
 
  /** 
   * Draws an ellipse filled with the current pen color.
   * 
   * @param x First coordinate of the circle's center.
   * @param y Second coordinate of the circle's center.
   * @param xRadius X axis radius of the ellipse.
   * @param yRadius Y axis radius of the ellipse.
   * @param depth Depth of the circle.
   */
  void fillEllipse( float x, float y, 
		    float xRadius, float yRadius,
		    int depth = -1);

  /** 
   * Draws a polygonal line.
   * 
   * @param points A vector of points.
   * @param depth The depth of the polyline.
   */
  void drawPolyline( const std::vector<Point> & points,
		     int depth = -1 );
  
  /** 
   * Draws a closed polygonal line.
   * 
   * @param points A vector of points.
   * @param depth The depth of the polyline.
   */
  void drawClosedPolyline( const std::vector<Point> & points,
			   int depth = -1 );

  /** 
   * Draws a filled polygon.
   * 
   * @param points A vector of points.
   * @param depth The depth of the polygon.
   */
  void fillPolyline( const std::vector<Point> & points,
		     int depth = -1 );
    
  /** 
   * Draws a string of text.
   * 
   * @param x The first coordinates of the lower left corner.
   * @param y The second coordinates of the lower left corner.
   * @param text The text. 
   * @param depth The depth of the text.
   */
  void drawText( float x, float y, const char * text, 
		 int depth = -1 );

  /** 
   * Draws a string of text.
   * 
   * @param x The first coordinates of the lower left corner.
   * @param y The second coordinates of the lower left corner.
   * @param text The text. 
   * @param depth The depth of the text.
   */
  void drawText( float x, float y, const std::string & str, 
		 int depth = -1 );

  /** 
   * Changes the current font and font size.
   *
   * @param font The name of the font.
   * @param fontSize The new font size. (The unit is 1pt = 1/72 in).
   * @return The board itself.
   */
  Board & setFont( const Fonts::Font font, float fontSize );
  
  /** 
   * Changes the font size.
   * 
   * @param fontSize The new font size.  (The unit is 1pt = 1/72 in).
   * @return The board itself.
   */
  Board & setFontSize( float fontSize ); 

  /** 
   * Changes the current pen color.
   * 
   * @param red Red component.
   * @param green Green component.
   * @param blue Blue component.
   * @return The board itself.
   */
  Board & setPenColorRGBi( unsigned char red,
			   unsigned char green,
			   unsigned char blue,
			   unsigned char alpha = 255 );

  /** 
   * Changes the current pen color.
   * 
   * @param red Red
   * @param green 
   * @param blue 
   * @param alpha 
   * @return The board itself.
   */  
  Board & setPenColorRGBf(  float red,
			    float green,
			    float blue, 
			    float alpha = 1.0f );

  /** 
   * Changes the current pen color.
   *
   * In order to use no pen, one may set the pen color to Color::None. 
   * @param color The pen color.
   * @return The board itself.
   */
  Board & setPenColor( const Color & color );
  

  /** 
   * Changes the current fill color.
   * 
   * @param red Red component.
   * @param green Green component.
   * @param blue Blue component.
   * @param alpha The opacity. 
   * @return The board itself.
   */
  Board & setFillColorRGBi( unsigned char red,
			    unsigned char green,
			    unsigned char blue,
			    unsigned char alpha = 255 );
  
  /** 
   * Changes the current fill color.
   * 
   * @param red Red component.
   * @param green Green component.
   * @param blue Blue component.
   * @param alpha The opacity.
   * @return The board itself.
   */
  Board & setFillColorRGBf( float red, float green, float blue, float alpha = 1.0f );

  /** 
   * Changes the current fill color.
   * 
   * In order to use no fill color, one may set this color to Color::None. 
   * @param color The fill color.
   * @return The board itself.
   */
  Board & setFillColor( const Color & color );
  
  /** 
   * Changes the current line thickness (1/72 inche unit).
   * 
   * @param width The new line thickness.
   * @return The board itself.
   */
  Board & setLineWidth( float width );
  
  /** 
   * Set the line cap style. 
   * 
   * @param cap The cap-style which can be Shape::ButtCap, 
   * Shape::RoundCap or Shape::SquareCap.
   * 
   * @return The board itself.
   */  
  inline Board & setLineCap( Shape::LineCap cap ); 
 
  /** 
   * Set the line joine style. 
   * 
   * @param cap The join-style which can be Shape::MiterJoin, 
   * Shape::RoundJoin or Shape::BevelJoin.
   * 
   * @return The board itself.
   */  
  inline Board & setLineJoin( Shape::LineJoin join );

  /** 
   * Changes the background color of the whole drawing.
   * 
   * @param color A color (may be Color::None).
   */
  void backgroundColor( const Color & color );

  /** 
   * Draws the current drawing's bounding box as a rectangle.
   * 
   * @param depth The depth of the rectangle.
   */
  void drawBoundingBox( int depth = -1 );


  /** 
   * Define a clipping rectangle for the whole drawing.
   * 
   * @param x 
   * @param y 
   * @param width 
   * @param height 
   */
  void setClippingRectangle(  float x, float y, 
			      float width, float height );

  /** 
   * Define a clipping path for the whole drawing.
   * 
   * @param points A path.
   */
  void setClippingPath(  const std::vector<Point> & points  );

  /** 
   * Define a clipping path for the whole drawing.
   * 
   * @param points A path.
   */
  void setClippingPath( const Path & path );

  /** 
   * Insert duplicates of a shape, n times, starting at its current position
   * and iterating given translations and scalings.
   * 
   * @param shape The shape to be duplicated.
   * @param times The number of duplicates.
   * @param dx The x shift.
   * @param dy The y shift.
   * @param scale A scale factor between each copy.
   */
  void addDuplicates( const Shape & shape,
		      unsigned int times,
		      double dx, double dy, double scale = 1.0 );

  /** 
   * Insert duplicates of a shape, n times, starting at its current position
   * and iterating a given translation.
   * 
   * @param shape The shape to be duplicated.
   * @param times The number of duplicates.
   * @param dx The x shift.
   * @param dy The y shift.
   * @param scaleX An x scale factor between each copy.
   * @param scaleY A y scale factor between each copy.
   * @param scaleY A y scale factor between each copy.
   * @param angle An angular increment.
   */
  void addDuplicates( const Shape & shape,
		      unsigned int times,
		      double dx, double dy,
		      double scaleX, 
		      double scaleY,
		      double angle = 0.0 );

  /** 
   * Save the drawing in an EPS, XFIG of SVG file depending 
   * on the filename extension. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename Path of the file to be created.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void save( const char * filename, PageSize size = Board::BoundingBox, float margin = 10.0 ) const; 
  
  /** 
   * Save the drawing in an EPS, XFIG of SVG file depending 
   * on the filename extension. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename Path of the file to be created.
   * @param pageWidth Width of the page in millimeters.
   * @param pageHeight Height of the page in millimeters.
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void save( const char * filename, float pageWidth, float pageHeight, float margin = 10.0 ) const; 

  /** 
   * Saves the drawing in an EPS file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The EPS file name.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveEPS( const char * filename, PageSize size = Board::BoundingBox, float margin = 10.0 ) const ;

  /** 
   * Saves the drawing in an EPS file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The EPS file name.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param pageWidth Width of the page in millimeters.
   * @param pageHeight Height of the page in millimeters.
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveEPS( const char * filename, float pageWidth, float pageHeight, float margin = 10.0 ) const ;

  /** 
   * Saves the drawing in an XFig file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the FIG file.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveFIG( const char * filename, PageSize size = Board::BoundingBox, float margin = 10.0 ) const;

  /** 
   * Saves the drawing in an XFig file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The XFig file name.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param pageWidth Width of the page in millimeters.
   * @param pageHeight Height of the page in millimeters.
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveFIG( const char * filename, float pageWidth, float pageHeight, float margin = 10.0 ) const ;

  /** 
   * Save the drawing in an SVG file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the file.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveSVG( const char * filename, PageSize size = Board::BoundingBox, float margin = 10.0 ) const;

  /** 
   * Saves the drawing in an SVG file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The SVG file name.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param pageWidth Width of the page in millimeters.
   * @param pageHeight Height of the page in millimeters.
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveSVG( const char * filename, float pageWidth, float pageHeight, float margin = 10.0 ) const ;

 protected:

  /**
   * Current graphical state for drawings made by the drawSomething() methods.
   * 
   */
  struct State {
    Color penColor;		    /**< The pen color. */
    Color fillColor;	   	    /**< The fill color. */
    float lineWidth;		    /**< The line thickness. */
    Shape::LineCap lineCap;	    /**< The type of line extremities. */
    Shape::LineJoin lineJoin; 	    /**< The type of line junction. */
    Fonts::Font font;               /**< The font. */
    float fontSize;	   	    /**< The font size. */
    float unitFactor;		    /**< The factor to be applied to arguments of the drawSomething() family. */
    State();
    float unit( const float & x ) { return x * unitFactor; }
    Point unit( const Point & p ) { return Point( p.x * unitFactor, p.y * unitFactor); }
    void unit( Shape & shape ) {  shape.scaleAll( unitFactor ); }
  };
  State _state;			 /**< The current state. */
  Color _backgroundColor;	 /**< The color of the background. */
  Path _clippingPath;
};

inline void
Board::clear( unsigned char red, unsigned char green, unsigned char blue )
{
  clear( Color( red, green, blue ) );
}

inline Board &
Board::setLineCap( Shape::LineCap cap )
{
  _state.lineCap = cap;
  return *this;
}
  
inline Board &
Board::setLineJoin( Shape::LineJoin join )
{
  _state.lineJoin = join;
  return *this;
}

inline void
Board::fillGouraudTriangle( const float x1, const float y1,
			    const Color & color1,
			    const float x2, const float y2, 
			    const Color & color2,
			    const float x3, const float y3,
			    const Color & color3,
			    unsigned char divisions,
			    int depth /* = -1 */ )
{
  fillGouraudTriangle( Point( x1, y1 ), color1,
		       Point( x2, y2 ), color2,
		       Point( x3, y3 ), color3,
		       divisions, depth );		       
}

void
Board::fillGouraudTriangle( const float x1, const float y1,
			    const float brightness1,
			    const float x2, const float y2, 
			    const float brightness2,
			    const float x3, const float y3,
			    const float brightness3,
			    unsigned char divisions,
			    int depth /* = -1 */ )
{
  fillGouraudTriangle( Point( x1, y1 ), brightness1,
		       Point( x2, y2 ), brightness2,
		       Point( x3, y3 ), brightness3,
		       divisions, depth );
}

} // namespace Board

#endif
