/* -*- mode: c++ -*- */
/**
 * @file   Transforms.h
 * @author Martial Tola <http://liris.cnrs.fr/martial.tola/>
 * @date   mercredi 20 avril 2011
 * 
 * @brief  
 */

#ifndef _BOARDCAIRO_TRANSFORMS_H_
#define _BOARDCAIRO_TRANSFORMS_H_

#include <cmath>

namespace LibBoard {

struct Rect;

/**
 * The base class for transforms.
 * @brief 
 */
struct Transform {
public:
  inline Transform();
  virtual ~Transform() { };
  virtual double mapX( double x ) const;
  virtual double mapY( double y ) const = 0;
  virtual void apply( double & x, double & y ) const;
  virtual double scale( double x ) const;
  virtual double rounded( double x ) const;
  virtual void setBoundingBox( const Rect & rect,
			       const double pageWidth,
			       const double pageHeight,
			       const double margin ) = 0;

  static inline double round( const double & x );

protected:
  double _scale;
  double _deltaX;
  double _deltaY;
  double _height;
};

Transform::Transform() 
  : _scale(1.0), _deltaX(0.0), _deltaY(0.0), _height(0.0)
{ }

double Transform::round( const double & x )
{
  return std::floor( x + 0.5 );
}

#ifdef WITH_CAIRO
/**
 * The TransformCairo structure.
 * @brief Structure representing a scaling and translation
 * suitable for an Cairo output.
 */
struct TransformCairo : public Transform {
public:
  double rounded( double x ) const;
  double mapY( double y ) const;
  double mapWidth( double width ) const; 
  void setBoundingBox( const Rect & rect,
		       const double pageWidth,
		       const double pageHeight,
		       const double margin );
};
#endif

//#include "Transforms.ih"

} // namespace

#endif
