/* -*- mode: c++ -*- */
/**
 * @file   Point.h
 * @author Sebastien Fourey <http://www.greyc.ensicaen.fr/~seb>
 * @date   Sat Aug 18 2007
 * 
 * @brief  The Point structure.
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
#ifndef _BOARD_POINT_H_
#define _BOARD_POINT_H_

#include <cmath>

namespace LibBoard {

/**
 * The Point structure.
 * @brief Struct representing a 2D point. 
 */
struct Point {
  double x;			/**< The point's first coordinate */
  double y;			/**< The point's second coordinate */

  /** 
   * Point constructor.
   * 
   * @param x The point's first coordinate.
   * @param y The point's second coordinate.
   */
  Point():x(0.0),y(0.0) { } 

  /** 
   * Point constructor.
   * 
   * @param x The point's first coordinate.
   * @param y The point's second coordinate.
   */
  Point( const Point & other ):x(other.x),y(other.y) { } 

  /** 
   * Point constructor.
   * 
   * @param x The point's first coordinate.
   * @param y The point's second coordinate.
   */
  Point( double x, double y ):x(x),y(y) { } 
  
  /** 
   * Rotate the point with a given angle around the origin (0,0)
   * 
   * @param angle The rotation angle.
   */
  inline Point & rotate( double angle );

  /** 
   * Get the values of the x and y values.
   * 
   * @param x Reference to the x to be set.
   * @param y Reference to the y to be set.
   */
  inline void get( double & x, double & y ) const;
  
  /** 
   * Rotate the point with a given angle around the origin (0,0)
   * 
   * @param angle The rotation angle.
   * 
   * @return The rotated point.
   */
  inline Point rotated( double angle ) const;

  inline Point & rotate( double angle, const Point & center );
  
  inline Point & rotated( double angle, const Point & center ) const;

  inline Point & operator+=( const Point & other );

  inline Point & operator-=( const Point & other );

  inline Point & operator*=( double s );
  
  inline Point & operator/=( double s );

  inline Point operator-();
  
  inline double norm() const;
  
};

inline void
Point::get( double & x, double & y ) const
{
  x = Point::x;
  y = Point::y;
}

inline Point
operator+( const Point & a, const Point & b )
{
  return Point( a.x + b.x, a.y + b.y );
}

inline Point
operator-( const Point & a, const Point & b )
{
  return Point( a.x - b.x, a.y - b.y );
}

inline double
operator*( const Point & a, const Point & b )
{
  return a.x * b.x + a.y * b.y;
}

inline Point
operator*( const Point & p, double s  )
{
  return Point( p.x * s, p.y * s );
}

inline Point
operator*( double s, const Point & p )
{
  return Point( s * p.x, s * p.y );
}

inline Point
operator/( const Point & p, double s )
{
  return Point( p.x / s, p.y / s );
}

inline Point &
Point::operator+=( const Point & other )
{
  x += other.x;
  y += other.y;
  return *this;
}

inline Point &
Point::operator-=( const Point & other )
{
  x -= other.x;
  y -= other.y;
  return *this;
}

inline Point &
Point::operator*=( double s )
{
  x *= s;
  y *= s;
  return *this;
}

inline Point &
Point::operator/=( double s )
{
  x /= s;
  y /= s;
  return *this;
}

inline bool
operator==( const Point & a, const Point & b )
{
  return ( a.x == b.x ) && ( a.y == b.y ) ;
}

inline bool
operator!=( const Point & a, const Point & b )
{
  return ( a.x != b.x ) || ( a.y != b.y ) ;
}

Point &
Point::rotate( double angle )
{
  double x = cos( angle ) * Point::x - sin( angle ) * Point::y; 
  double y = sin( angle ) * Point::x + cos( angle ) * Point::y;
  Point::x = x;
  Point::y = y;
  return *this;
}

Point
Point::rotated( double angle ) const
{
  return Point(*this).rotate( angle );
}

Point &
Point::rotate( double angle, const Point & center )
{
  (*this) -= center;
  (*this).rotate( angle );
  (*this) += center;
  return *this;
}

Point &
Point::rotated( double angle, const Point & center ) const
{
  return Point(*this).rotate( angle, center );
}

double
Point::norm() const
{
  return sqrt( x*x + y*y );
}

Point Point::operator-()
{
  return Point( -x, -y );
}

} // mamespace BoardLib

#endif // _POINT_H_

