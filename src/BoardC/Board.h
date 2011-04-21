/* -*- mode: c++ -*- */
/**
 * @file   Board.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_BOARD_H_
#define _BOARDCAIRO_BOARD_H_

#include <vector>
#include <string>

#include "ShapeList.h"
#include "Color.h"
#include "Point.h"
#include "PSFonts.h"

namespace LibBoard {

/** 
 * The Board class.
 * @brief Class for Cairo drawings.
 * @version 0.1
 */
class Board : public ShapeList {

public:
  
  // cairo
  enum CairoType { CairoPDF, CairoPNG, CairoPS, CairoSVG };
  
  enum PageSize { BoundingBox, A4, Letter };
  enum Unit { UPoint, UInche, UCentimeter, UMillimeter };
  
  /** 
   * Constructs a new board and sets the background color, if any.
   * 
   * @param backgroundColor A color for the drawing's background.
   */
  Board( const Color & backgroundColor = Color::None );
  
  /** 
   * Clears the board with a given background color.
   * 
   * @param color The board background color (may be Color::None).
   */
  void clear( const Color & color = Color::None );
  
  Shape & translate( double dx, double dy );
  
  Shape & scale( double s );
  
  /** 
   * Set the unit used by the drawSomething methods.
   * 
   * @param unit The unit to be used in { PT, IN, CM, MM }.
   */
  void setUnit( Unit unit );
  
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
   * In order to use no fill color, one may set this color to Color::None. 
   * @param color The fill color.
   * @return The board itself.
   */
  Board & setFillColor( const Color & color );
  
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
   * In order to use no pen, one may set the pen color to Color::None. 
   * @param color The pen color.
   * @return The board itself.
   */
  Board & setPenColor( const Color & color );
  
  /** 
   * Changes the current line thickness (1/72 inche unit).
   * 
   * @param width The new line thickness.
   * @return The board itself.
   */
  Board & setLineWidth( double width );

  /** 
   * Changes the current line style.
   * 
   * @param style The new line style.
   * @return The board itself.
   */
  Board & setLineStyle( Shape::LineStyle style );
  
  /** 
   * Set the line cap style. 
   * 
   * @param cap The cap-style which can be Shape::ButtCap, 
   * Shape::RoundCap or Shape::SquareCap.
   * 
   * @return The board itself.
   */  
  Board & setLineCap( Shape::LineCap cap ); 
 
  /** 
   * Set the line joine style. 
   * 
   * @param cap The join-style which can be Shape::MiterJoin, 
   * Shape::RoundJoin or Shape::BevelJoin.
   * 
   * @return The board itself.
   */  
  Board & setLineJoin( Shape::LineJoin join );
  
  /** 
   * Draws a line from (x1,y1) to (x2,y2).
   * 
   * @param x1 First coordinate of the first extremity.
   * @param y1 Second coordinate of the first extremity.
   * @param x2 First coordinate of the second extremity.
   * @param y2 Second coordinate of the second extremity.
   * @param depthValue Depth of the line.
   */
  void drawLine( double x1, double y1, double x2, double y2, 
		 int depthValue = -1 );
  
  /** 
   * Draws a circle filled with the current pen color.
   * 
   * @param x First coordinate of the circle's center.
   * @param y Second coordinate of the circle's center.
   * @param radius Radius of the circle.
   * @param depthValue Depth of the circle.
   */
  void fillCircle( double x, double y, double radius,
		   int depthValue = -1);
  
  /** 
   * Draws a rectangle.
   * 
   * @param x First coordinate of the upper left corner.
   * @param y Second coordinate of the upper left corner.
   * @param width Width of the rectangle.
   * @param height Height of the rectangle.
   * @param depthValue Depth of the rectangle.
   */
  void drawRectangle( double x, double y, 
		      double width, double height,
		      int depthValue = -1 );
  
  /** 
   * Draws a line from (x1,y1) to (x2,y2) with an arrow at (x2,y2).
   * 
   * @param x1 First coordinate of the first extremity.
   * @param y1 Second coordinate of the first extremity.
   * @param x2 First coordinate of the second extremity.
   * @param y2 Second coordinate of the second extremity.
   * @param filled Whether or not the arrow is filled.
   * @param depthValue Depth of the line.
   */
  void drawArrow( double x1, double y1, double x2, double y2,
		  bool filled = true,
		  int depthValue = -1 );
  
  /** 
   * Draws a closed polygonal line.
   * 
   * @param points A vector of points.
   * @param depthValue The depth of the polyline.
   */
  void drawClosedPolyline( const std::vector<Point> & points,
			   int depthValue = -1 );
  
  /** 
   * Draws a polygonal line.
   * 
   * @param points A vector of points.
   * @param depthValue The depth of the polyline.
   */
  void drawPolyline( const std::vector<Point> & points,
		     int depthValue = -1 );
  
  /** 
   * Changes the current font and font size.
   *
   * @param font The name of the font.
   * @param fontSize The new font size. (The unit is 1pt = 1/72 in).
   * @return The board itself.
   */
  Board & setFont( const Fonts::Font font, double fontSize );
  
  /** 
   * Draws a string of text.
   * 
   * @param x The first coordinates of the lower left corner.
   * @param y The second coordinates of the lower left corner.
   * @param text The text. 
   * @param depthValue The depth of the text.
   */
  void drawText( double x, double y, const std::string & str, 
		 int depthValue = -1 );
  
  /** 
   * Save the drawing in an SVG file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the file.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveSVG( const char * filename, PageSize size = Board::BoundingBox, double margin = 10.0 ) const;
  
  /** 
   * Saves the drawing in an EPS file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The EPS file name.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveEPS( const char * filename, PageSize size = Board::BoundingBox, double margin = 10.0 ) const ;
  
  /** 
   * Saves the drawing in an XFig file. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the FIG file.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveFIG( const char * filename, PageSize size = Board::BoundingBox, double margin = 10.0 ) const;
  
#ifdef WITH_CAIRO
  // cairo
  /** 
   * Save the drawing with cairo. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the file.
   * @param type Output type: CairoPDF, CairoPNG, CairoPS, CairoSVG.
   * @param size Page size (Either BoundingBox (default), A4 or Letter).
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveCairo( const char * filename, CairoType type = CairoPNG, PageSize size = Board::BoundingBox, double margin = 10.0 ) const;
  
  /** 
   * Save the drawing with cairo. When a size is given (not BoundingBox), the drawing is
   * scaled (up or down) so that it fits within the dimension while keeping its aspect ratio.
   * 
   * @param filename The name of the file.
   * @param type Output type: CairoPDF, CairoPNG, CairoPS, CairoSVG.
   * @param pageWidth Width of the page in millimeters.
   * @param pageHeight Height of the page in millimeters.
   * @param margin Minimal margin around the figure in the page, in millimeters.
   */
  void saveCairo( const char * filename, CairoType type, double pageWidth, double pageHeight, double margin = 10.0 ) const ;
#endif
  
protected:

  /**
   * Current graphical state for drawings made by the drawSomething() methods.
   * 
   */
  struct State {
    Color penColor;		        /**< The pen color. */
    Color fillColor;	   	    	/**< The fill color. */
    double lineWidth;		    	/**< The line thickness. */
    Shape::LineStyle lineStyle;		/**< The line style. */
    Shape::LineCap lineCap;	    	/**< The type of line extremities. */
    Shape::LineJoin lineJoin; 		/**< The type of line junction. */
    Fonts::Font font;           	/**< The font. */
    double fontSize;	   	        /**< The font size. */
    double unitFactor;		    	/**< The factor to be applied to arguments of the drawSomething() family. */
    State();
    // TODO * 3
    double unit( const double & x ) { return x * unitFactor; }
    //Point unit( const Point & p ) { return Point( p.x * unitFactor, p.y * unitFactor); }
    //void unit( Shape & shape ) {  shape.scaleAll( unitFactor ); }
  };
  
  State _state;			 	/**< The current state. */
  Color _backgroundColor;	 	/**< The color of the background. */
};

inline
Board &
Board::setLineStyle( Shape::LineStyle style )
{
  _state.lineStyle = style;
  return *this;
}

inline
Board &
Board::setLineCap( Shape::LineCap cap )
{
  _state.lineCap = cap;
  return *this;
}
  
inline
Board &
Board::setLineJoin( Shape::LineJoin join )
{
  _state.lineJoin = join;
  return *this;
}

} // namespace

/*
 * Inline methods
 */
//#include "Board.ih"

#endif
