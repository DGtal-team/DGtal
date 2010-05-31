/* -*- mode: c++ -*- */
/**
 * @file   Transforms.cpp
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

#include "board/Rect.h"
#include "board/Shapes.h"
#include "board/ShapeList.h"
#include "board/Transforms.h"
#include <cmath>

namespace {
  const float ppmm = 720.0f / 254.0f;
  const float fig_ppmm = 1143 / 25.4f;
}

namespace LibBoard {

//
// Transform
// 

float
Transform::rounded( float x ) const
{
  return Transform::round( 1000000*x ) / 1000000;
} 

float
Transform::mapX( float x ) const
{
  return rounded( x * _scale + _deltaX );
}

float
Transform::scale( float x ) const
{
  return rounded( x * _scale );
}

void
Transform::apply( float & x, float & y ) const
{
  x = mapX( x );
  y = mapY( y );
}

//
// TransformEPS
// 

float
TransformEPS::mapY( float y ) const
{
  return rounded( y * _scale + _deltaY );
}

void
TransformEPS::setBoundingBox( const Rect & rect,
			      const float pageWidth,
			      const float pageHeight,
			      const float margin )
{
  if ( pageWidth <= 0 || pageHeight <= 0 ) {
    _scale = 1.0f;
    // _deltaX = - rect.left;
    _deltaX = 0.5 * 210 * ppmm - ( rect.left + 0.5 * rect.width );
    // _deltaY = - ( rect.top - rect.height );
    _deltaY = 0.5 * 297 * ppmm - ( rect.top - 0.5 * rect.height );  
    _height = rect.height;
  } else {
    const float w = pageWidth - 2 * margin;
    const float h = pageHeight - 2 * margin;
    if ( ( rect.height / rect.width ) > ( h / w ) ) {
      _scale = h * ppmm / rect.height;
    } else {
      _scale = w * ppmm / rect.width;
    }
    _deltaX = 0.5 * pageWidth * ppmm - _scale * ( rect.left + 0.5 * rect.width );
    _deltaY = 0.5 * pageHeight * ppmm - _scale * ( rect.top - 0.5 * rect.height );  
    _height = pageHeight * ppmm;
  }
}

//
// TransformFIG
// 

float
TransformFIG::rounded( float x ) const
{
  return Transform::round( x );
}

float
TransformFIG::mapY( float y ) const
{
  // float ppmm = 1200/25.4;
  return rounded( _height - ( y * _scale + _deltaY ) );
}

int
TransformFIG::mapWidth( float width ) const
{
  // FIG width unit is 1/160 inch
  // Postscript points are 1/72 inch
  if ( width == 0.0 ) return 0;
  int result = static_cast<int>( Transform::round( 160 * ( width / 72.0 ) ) );
  return result>0?result:1;
}

void
TransformFIG::setBoundingBox( const Rect & rect,
			      const float pageWidth,
			      const float pageHeight,
			      const float margin )
{
  if ( pageWidth <= 0 || pageHeight <= 0 ) {
    _scale = fig_ppmm / ppmm;
    _deltaX = 0.5 * 210 * fig_ppmm - _scale * ( rect.left + 0.5 * rect.width );
    //_deltaX = - rect.left;
    _deltaY = 0.5 * 297 * fig_ppmm - _scale * ( rect.top - 0.5 * rect.height );  
    // _deltaY = - rect.top;
    // _deltaY = - ( rect.top - rect.height );
    //_height = rect.height;
    _height = 297 * fig_ppmm;
  } else {
    const float w = pageWidth - 2 * margin;
    const float h = pageHeight - 2 * margin;
    if ( rect.height / rect.width > ( h / w ) ) {
      _scale = ( h * fig_ppmm ) / rect.height;
    } else {
      _scale = ( w * fig_ppmm ) / rect.width;
    }
    _deltaX = 0.5 * pageWidth * fig_ppmm - _scale * ( rect.left + 0.5 * rect.width );
    _deltaY = 0.5 * pageHeight * fig_ppmm - _scale * ( rect.top - 0.5 * rect.height );  
    _height = pageHeight * fig_ppmm;
  }
  // float ppmm = (1200/25.4);
}

void
TransformFIG::setDepthRange( const ShapeList & shapes )
{
  _maxDepth = shapes.maxDepth();
  _minDepth = shapes.minDepth();
}

int
TransformFIG::mapDepth( int depth ) const
{
  if ( depth > _maxDepth ) return 999;
  if ( _maxDepth - _minDepth > 998 ) {
    double range = _maxDepth - _minDepth;
    int r = static_cast<int>( 1 + Transform::round( ( ( depth - _minDepth ) / range ) * 998 ) );
    return r>=0?r:0;
  } else {
    int r = 1 + depth - _minDepth;
    return r>=0?r:0;
  }
}

//
// TransformSVG
// 

float
TransformSVG::rounded( float x ) const
{
  return Transform::round( 100*x ) / 100.0f;
} 

float
TransformSVG::mapY( float y ) const
{
  return rounded( _height - ( y * _scale + _deltaY ) );
}

float
TransformSVG::mapWidth( float width ) const
{
  // return Transform::round( 1000 * width / ppmm ) / 1000.0;
  return Transform::round( 1000 * width  / ppmm  ) / 1000.0;
}

void
TransformSVG::setBoundingBox( const Rect & rect,
			      const float pageWidth,
			      const float pageHeight,
			      const float margin )  
{
  if ( pageWidth <= 0 || pageHeight <= 0 ) {
    _scale = 1.0f;
    // _deltaX = 0.5 * 210 * ppmm - ( rect.left + 0.5 * rect.width );
    _deltaX = - rect.left;
    // _deltaY = 0.5 * 297 * ppmm - ( rect.top - 0.5 * rect.height );
    _deltaY = - ( rect.top - rect.height );
    // _height = 297 * fig_ppmm;
    _height = rect.height;
  } else {
    const float w = pageWidth - 2 * margin;
    const float h = pageHeight - 2 * margin;
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

} // namespace LibBoard
