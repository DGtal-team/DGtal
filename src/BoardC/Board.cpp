/* -*- mode: c++ -*- */
/**
 * @file   Board.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include <algorithm>

#include "Board.h"

#ifdef WITH_CAIRO
// cairo
#include <cairo.h>
#include <cairo-pdf.h>
#include <cairo-ps.h>
#include <cairo-svg.h>
// cairo
#endif

namespace {
  const float pageSizes[3][2] = { { 0.0f, 0.0f }, // BoundingBox
				  { 210.0f, 297.0f },
				  { 8.5f*25.4f, 11.0f*25.4f } };
  const float ppmm = 720.0f / 254.0f;
}

namespace LibBoard {
  
Board::State::State()
{
  penColor = Color::Black;
  fillColor =  Color::None;
  lineWidth = 0.5;
  lineStyle = Shape::SolidStyle;
  lineCap = Shape::ButtCap;
  lineJoin = Shape::MiterJoin;
  font = Fonts::TimesRoman;
  fontSize = 11.0;
  unitFactor = 1.0;
}

Board::Board( const Color & bgColor )
  : _backgroundColor( bgColor )
{
}

void
Board::clear( const Color & color )
{
  ShapeList::clear();
  _backgroundColor = color;
}

Shape &
Board::translate( double dx, double dy )
{
  // TODO:
  /*ShapeList::translate( dx, dy );
  _clippingPath.translate( dx, dy );*/
  return (*this);
}

Shape &
Board::scale( double s )
{
  // TODO:
  /*Point delta = _clippingPath.center() - center();
  delta *= s;
  _clippingPath.scale( s );
  ShapeList::scale( s );
  delta = ( center() + delta ) - _clippingPath.center();
  _clippingPath.translate( delta.x, delta.y );*/
  return (*this);
}

void
Board::setUnit( Unit unit )
{
  switch ( unit ) {
  case UPoint:
    _state.unitFactor = 1.0;
    break;
  case UInche:
    _state.unitFactor = 25.4f * ppmm;
    break;
  case UCentimeter:
    _state.unitFactor = 10.0f * ppmm;
    break;
  case UMillimeter:
    _state.unitFactor = ppmm;
    break;
  }
}

void
Board::setUnit( double factor, Unit unit )
{
  switch ( unit ) {
  case UPoint:
    _state.unitFactor = factor;
    break;
  case UInche:
    _state.unitFactor = 720.0f * factor; // TODO: strange ??? ---> 25.4f * ppmm * factor ???
    break;
  case UCentimeter:
    _state.unitFactor = 10.0f * ppmm * factor;
    break;
  case UMillimeter:
    _state.unitFactor = ppmm * factor;
    break;
  }
}

Board &
Board::setFillColorRGBi( unsigned char red,
			 unsigned char green,
			 unsigned char blue,
			 unsigned char alpha )
{
  _state.fillColor.setRGBi( red, green, blue, alpha );
  return *this;
}

Board &
Board::setFillColor( const Color & color )
{
  _state.fillColor = color;
  return *this;
}

Board &
Board::setPenColorRGBi( unsigned char red,
			unsigned char green,
			unsigned char blue, 
			unsigned char alpha )
{
  _state.penColor.setRGBi( red, green, blue, alpha );
  return *this;
}

Board &
Board::setPenColor( const Color & color )
{
  _state.penColor = color;
  return *this;
}

Board &
Board::setLineWidth( double width )
{
  _state.lineWidth = width;
  return *this;
}

void
Board::drawLine( double x1, double y1, double x2, double y2, 
		 int depthValue /* = -1 */  )
{
  if ( depthValue != -1 ) 
    _shapes.push_back( new Line( _state.unit(x1), _state.unit(y1),
				 _state.unit(x2), _state.unit(y2),
				 _state.penColor, _state.lineWidth,
				 _state.lineStyle, _state.lineCap, _state.lineJoin, depthValue ) );
  else
    _shapes.push_back( new Line( _state.unit(x1), _state.unit(y1),
				 _state.unit(x2), _state.unit(y2),
				 _state.penColor, _state.lineWidth,
				 _state.lineStyle, _state.lineCap, _state.lineJoin, _nextDepth-- ) );
}

void
Board::fillCircle( double x, double y,
		   double radius,
		   int depthValue /* = -1 */ )
{
  // TODO
  /*int d = (depthValue != -1) ? depthValue : _nextDepth--;
  _shapes.push_back( new Circle( _state.unit(x), _state.unit(y), _state.unit(radius), 
				 Color::None, _state.penColor, // MT: _state.fillColor ???
				 0.0f, _state.lineStyle, d ) );*/
}

void
Board::drawRectangle( double x, double y, 
		      double width, double height,
		      int depthValue /* = -1 */ )
{
  // TODO
  /*int d = (depthValue != -1) ? depthValue : _nextDepth--;
  _shapes.push_back( new Rectangle( _state.unit(x), _state.unit(y), _state.unit(width), _state.unit(height), 
				    _state.penColor, _state.fillColor,
				    _state.lineWidth, _state.lineStyle, _state.lineCap, _state.lineJoin, d ) );*/
}

void
Board::drawArrow( double x1, double y1, double x2, double y2, 
		  bool filledArrow /* = false */,
		  int depthValue /* = -1 */  )
{
  // TODO
  /*if ( depthValue != -1 )
    _shapes.push_back( new Arrow( _state.unit(x1), _state.unit(y1),
				  _state.unit(x2), _state.unit(y2),
				  _state.penColor, filledArrow ? _state.penColor : Color::None, // MT: _state.fillColor ???
				  _state.lineWidth, _state.lineStyle, _state.lineCap, _state.lineJoin, depthValue ) );
  else
    _shapes.push_back( new Arrow( _state.unit(x1), _state.unit(y1),
				  _state.unit(x2), _state.unit(y2),
				  _state.penColor, filledArrow ? _state.penColor : Color::None, // MT: _state.fillColor ???
				  _state.lineWidth, _state.lineStyle, _state.lineCap, _state.lineJoin, _nextDepth-- ) );*/
}

void
Board::drawClosedPolyline( const std::vector<Point> & points,
			   int depthValue /* = -1 */ )
{
  // TODO
  /*int d = (depthValue != -1) ? depthValue : _nextDepth--;
  std::vector<Point> v = points;
  std::vector<Point>::iterator it = v.begin();
  std::vector<Point>::iterator end = v.end();
  while ( it != end ) {
    (*it) = _state.unit( *it );
    ++it;
  }
  _shapes.push_back( new Polyline( v, true, _state.penColor, _state.fillColor,
				   _state.lineWidth,
				   _state.lineStyle,
				   _state.lineCap,
				   _state.lineJoin,
				   d ) );*/
}

void
Board::drawPolyline( const std::vector<Point> & points,
		     int depthValue /* = -1 */ )
{
  // TODO
  /*int d = (depthValue != -1) ? depthValue : _nextDepth--;  
  std::vector<Point> v = points;
  std::vector<Point>::iterator it = v.begin();
  std::vector<Point>::iterator end = v.end();
  while ( it != end ) {
    (*it) = _state.unit( *it );
    ++it;
  }
  _shapes.push_back( new Polyline( v, false, _state.penColor, _state.fillColor,
				   _state.lineWidth,
				   _state.lineStyle,
				   _state.lineCap,
				   _state.lineJoin,
				   d ) );*/
}

Board &
Board::setFont( const Fonts::Font font, double fontSize )
{
  _state.font = font;
  _state.fontSize = fontSize;
  return *this;
}

void
Board::drawText( double x, double y, const std::string & str, int depthValue /* = -1 */ )
{
  // TODO
  /*int d = (depthValue != -1) ? depthValue : _nextDepth--;
  _shapes.push_back( new Text( _state.unit(x), _state.unit(y), str,
			       _state.font, _state.fontSize, _state.penColor, d ) );*/
}

void
Board::saveSVG( const char * filename, PageSize size, double margin ) const
{
  // TODO: saveSVG( filename, pageSizes[size][0], pageSizes[size][1], margin );
}

void
Board::saveEPS( const char * filename, PageSize size, double margin ) const
{
  // TODO: saveEPS( filename, pageSizes[size][0], pageSizes[size][1], margin );
}

void
Board::saveFIG( const char * filename, PageSize size, double margin ) const
{
  // TODO: saveFIG( filename, pageSizes[size][0], pageSizes[size][1], margin );
}

#ifdef WITH_CAIRO
void
Board::saveCairo( const char * filename, CairoType type, PageSize size, double margin ) const
{
  saveCairo( filename, type, pageSizes[size][0], pageSizes[size][1], margin );
}

void
Board::saveCairo( const char * filename, CairoType type, double pageWidth, double pageHeight, double margin ) const
{
  cairo_surface_t *surface;
  cairo_t *cr;
  
  double cairoWidth, cairoHeight;
  
  // TODO: TransformCairo transform;
  Rect box = boundingBox();

  // TODO:
  /*bool clipping = _clippingPath.size() > 2;
  if ( clipping )
    box = box && _clippingPath.boundingBox();*/
  // TODO: transform.setBoundingBox( box, pageWidth, pageHeight, margin );
  
  if ( pageWidth > 0 && pageHeight > 0 )
  {
    cairoWidth = pageWidth;
    cairoHeight = pageHeight;
  }
  else
  {
    cairoWidth = box.width;
    cairoHeight = box.height;
  }
  
  switch (type)
  {
    case CairoPDF:
      surface = cairo_pdf_surface_create (filename, cairoWidth, cairoHeight); break;
    case CairoPS:
      surface = cairo_ps_surface_create (filename, cairoWidth, cairoHeight); break;
    case CairoEPS:
      surface = cairo_ps_surface_create (filename, cairoWidth, cairoHeight); 
      cairo_ps_surface_set_eps(surface, true); break;
    case CairoSVG:
      surface = cairo_svg_surface_create (filename, cairoWidth, cairoHeight); break;
    case CairoPNG:
    default:
      surface = cairo_image_surface_create (CAIRO_FORMAT_ARGB32, cairoWidth, cairoHeight);
  }
  
  cr = cairo_create (surface);
  
  /* For 1.0 x 1.0 coordinate space */
  //cairo_scale (cr, cairoWidth, cairoHeight);
  
  //temp: http://zetcode.com/tutorials/cairographicstutorial/basicdrawing/
  //temp: http://www.graphviz.org/pub/scm/graphviz-cairo/plugin/cairo/gvrender_cairo.c
      
  // Draw the background color if needed.
  // TODO:
  /*if ( _backgroundColor != Color::None ) { 
    Rectangle r( box, Color::None, _backgroundColor, 0.0 );
    r.flushCairo( cr, transform );
  }*/
  
  // Draw the shapes.
  std::vector< Shape* > shapes = _shapes;
  stable_sort( shapes.begin(), shapes.end(), shapeGreaterDepth );
  std::vector< Shape* >::const_iterator i = shapes.begin();
  std::vector< Shape* >::const_iterator end = shapes.end();
  while ( i != end ) {
    (*i)->flushCairo( cr/*, transform*/ ); // TODO
    ++i;
  }
  
  if (type==CairoPNG)
    cairo_surface_write_to_png (surface, filename);
      
  cairo_destroy (cr);
  cairo_surface_destroy (surface);
}
#endif

} // namespace