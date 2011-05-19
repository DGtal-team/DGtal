/* -*- mode: c++ -*- */
/**
 * @file   Transforms.cpp
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#include "Board/Rect.h"
#include "Board/Transforms.h"

namespace {
  const float ppmm = 720.0f / 254.0f;
}

namespace LibBoard {

double
Transform::rounded( double x ) const
{
  return Transform::round( 1000000*x ) / 1000000;
} 

double
Transform::mapX( double x ) const
{
  return rounded( x * _scale + _deltaX );
}

double
Transform::scale( double x ) const
{
  return rounded( x * _scale );
}

void
Transform::apply( double & x, double & y ) const
{
  x = mapX( x );
  y = mapY( y );
}

#ifdef WITH_CAIRO
double
TransformCairo::rounded( double x ) const
{
  return Transform::round( 100*x ) / 100.0f;
} 

double
TransformCairo::mapY( double y ) const
{
  return rounded( _height - ( y * _scale + _deltaY ) );
}

double
TransformCairo::mapWidth( double width ) const
{
  return Transform::round( 1000 * width  / ppmm  ) / 1000.0;
}

void
TransformCairo::setBoundingBox( const Rect & rect,
			      const double pageWidth,
			      const double pageHeight,
			      const double margin )  
{
  if ( pageWidth <= 0 || pageHeight <= 0 ) {
    _scale = 1.0f;
    _deltaX = - rect.left;
    _deltaY = - ( rect.top - rect.height );
    _height = rect.height;
  } else {
    const double w = pageWidth - 2 * margin;
    const double h = pageHeight - 2 * margin;
    if ( rect.height / rect.width > ( h / w ) ) {
      _scale = h * ppmm / rect.height;
    } else {
      _scale = w * ppmm / rect.width;
    }
    _deltaX = 0.5 * pageWidth * ppmm - _scale * ( rect.left + 0.5 * rect.width );
    _deltaY = 0.5 * pageHeight * ppmm - _scale * ( rect.top - 0.5 * rect.height );
    _height = pageHeight * ppmm;
  }
}
#endif

} // namespace
