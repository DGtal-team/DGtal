/* -*- mode: c++ -*- */
/**
 * @file   Transforms.h
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
#ifndef _BOARD_TRANSFORMS_H_
#define _BOARD_TRANSFORMS_H_

#include <limits>
#include <vector>
#include <cmath>

namespace LibBoard {

struct Rect;
struct Shape;
struct ShapeList;

/**
 * The base class for transforms.
 * @brief 
 */
struct Transform {
public:
  Transform() : _scale(1.0), _deltaX(0.0), _deltaY(0.0), _height(0.0) { }
  virtual ~Transform() { };
  virtual float mapX( float x ) const;
  virtual float mapY( float y ) const = 0;
  virtual void apply( float & x, float & y ) const;
  virtual float scale( float x ) const;
  virtual float rounded( float x ) const;
  virtual void setBoundingBox( const Rect & rect,
			       const float pageWidth,
			       const float pageHeight,
			       const float margin ) = 0;

  static inline float round( const float & x );
  static inline double round( const double & x );

protected:
  float _scale;
  float _deltaX;
  float _deltaY;
  float _height;
};

/**
 * The TransformEPS structure.
 * @brief Structure representing a scaling and translation
 * suitable for an EPS output.
 */
struct TransformEPS : public Transform {
public:
  float mapY( float y ) const;
  void setBoundingBox( const Rect & rect,
		       const float pageWidth,
		       const float pageHeight,
		       const float margin );
};

/**
 * The TransformFIG structure.
 * @brief Structure representing a scaling and translation
 * suitable for an XFig output.
 */
struct TransformFIG : public Transform {
public:
  TransformFIG():_maxDepth(std::numeric_limits<int>::max()),_minDepth(0) { }
  float rounded( float x ) const;
  float mapY( float y ) const;
  int mapWidth( float width ) const; 
  void setBoundingBox( const Rect & rect,
		       const float pageWidth,
		       const float pageHeight,
		       const float margin );
  void setDepthRange( const ShapeList & shapes );
  int mapDepth( int depth ) const;
private:
  int _maxDepth;
  int _minDepth;
};

/**
 * The TransformSVG structure.
 * @brief Structure representing a scaling and translation
 * suitable for an SVG output.
 */
struct TransformSVG : public Transform {
public:
  float rounded( float x ) const;
  float mapY( float y ) const;
  float mapWidth( float width ) const; 
  void setBoundingBox( const Rect & rect,
		       const float pageWidth,
		       const float pageHeight,
		       const float margin );
};

inline float Transform::round( const float & x )
{
  return static_cast<float>( std::floor( x + 0.5f ) );
}

inline double Transform::round( const double & x )
{
  return std::floor( x + 0.5 );
}

} // namespace LibBoard

#endif /* _TRANSFORMS_H_ */
