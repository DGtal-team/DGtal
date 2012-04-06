/**
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as
 *  published by the Free Software Foundation, either version 3 of the
 *  License, or  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 **/

#pragma once

/**
 * @file cone.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/02/24
 *
 * Header file for module cone.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(cone_RECURSES)
#error Recursive header files inclusion detected in cone.h
#else // defined(cone_RECURSES)
/** Prevents recursive inclusion of headers. */
#define cone_RECURSES

#if !defined cone_h
/** Prevents repeated inclusion of headers. */
#define cone_h


namespace DGtal
{

class cone {


  struct Tools
  {
    static bool isBetween(double i, double a, double b, double n)
    {
      // if a<=b, a simple comparison enables to conclude
      if(a<=b)
	if(i>=a && i<=b)
	  return true;
	else
	  return false;
      else
	{
	  //otherwise, translate the points such that a->0
	  int tmp = a;
	  a = fmod(a+n-tmp,n);
	  b = fmod(b+n-tmp,n);
	  i = fmod(i+n-tmp,n);
	  return isBetween(i,a,b,n); 
	}
    }
    
    // Code by Tim Voght
    // http://local.wasp.uwa.edu.au/~pbourke/geometry/2circle/tvoght.c
    
    /* circle_circle_intersection() *
     * Determine the points where 2 circles in a common plane intersect.
     *
     * int circle_circle_intersection(
     *                                // center and radius of 1st circle
     *                                double x0, double y0, double r0,
     *                                // center and radius of 2nd circle
     *                                double x1, double y1, double r1,
     *                                // 1st intersection point
     *                                double *xi, double *yi,              
     *                                // 2nd intersection point
     *                                double *xi_prime, double *yi_prime)
     *
     * This is a public domain work. 3/26/2005 Tim Voght
     *
     */
    static int  circle_circle_intersection(double x0, double y0, double r0,
					   double x1, double y1, double r1,
					   double *xi, double *yi,
					   double *xi_prime, double *yi_prime)
    {
      double a, dx, dy, d, h, rx, ry;
      double x2, y2;
      
      /* dx and dy are the vertical and horizontal distances between
       * the circle centers.
       */
      dx = x1 - x0;
      dy = y1 - y0;
      
      /* Determine the straight-line distance between the centers. */
      //d = sqrt((dy*dy) + (dx*dx));
      d = hypot(dx,dy); // Suggested by Keith Briggs
      
      /* Check for solvability. */
      if (d > (r0 + r1))
	{
	  /* no solution. circles do not intersect. */
	  std::cerr  << "Warning : the two circles do not intersect -> should never happen" << std::endl;
	  return 0; //I. Sivignon 05/2010 should never happen for our specific use.
	}
      if (d < fabs(r0 - r1))
	{
	  /* no solution. one circle is contained in the other */
	  return 0;
	}
      
      /* 'point 2' is the point where the line through the circle
       * intersection points crosses the line between the circle
       * centers.  
       */
      
      /* Determine the distance from point 0 to point 2. */
      a = ((r0*r0) - (r1*r1) + (d*d)) / (2.0 * d) ;
      
      /* Determine the coordinates of point 2. */
      x2 = x0 + (dx * a/d);
      y2 = y0 + (dy * a/d);
      
      /* Determine the distance from point 2 to either of the
       * intersection points.
       */
      h = sqrt((r0*r0) - (a*a));
      
      /* Now determine the offsets of the intersection points from
       * point 2.
       */
      rx = -dy * (h/d);
      ry = dx * (h/d);
      
      /* Determine the absolute intersection points. */
      *xi = x2 + rx;
      *xi_prime = x2 - rx;
      *yi = y2 + ry;
      *yi_prime = y2 - ry;
      
      //   if(fabs(rx) < 0.00001 && fabs(ry) < 0.00001)
      //     std::cerr << "un seul point" << std::endl;
      
      return 1;
    }
    
    static int circleTangentPoints(double x, double y, double x1, double y1, double r1, double *xi, double *yi,
				   double *xi_prime, double *yi_prime)
    {
      double x0 = (x+x1)/2;
      double y0 = (y+y1)/2;
      double r0 = sqrt((x-x1)*(x-x1) + (y-y1)*(y-y1))/2;
      
      int res =
	circle_circle_intersection(x0,y0,r0,x1,y1,r1,xi,yi,xi_prime,yi_prime);
      
      return res;
      
    }
    
    // compute the angle of the line passing through two points. Angle in [0,2pi]
    static double computeAngle(double x0, double y0, double x1, double y1)
    {
      double x = x1-x0;
      double y = y1-y0;
      
      if(x!=0)
	{
	  double alpha = y/x;
	  
	  if(x>0 && y>=0)
	    return atan(alpha);
	  else
	    if(x>0 && y<0)
	      return atan(alpha)+2*M_PI;
	    else
	      if(x<0)
		return atan(alpha)+M_PI;
	}
      else
	{
	  if(y>0)
	    return M_PI_2;
	  else
	    return 3*M_PI_2;
	}
    }
    
    
  };
  
 public:
  double myMin;
  double myMax;
  double myB;
  bool myInf; // true if the cone is infinite (the whole plane)
  
  cone();
  cone(double m, double mm, bool a);
  cone(double a0, double a1);
  cone(double x, double y, double x0, double y0, double x1, double y1);
  bool isEmpty();
  cone& operator=(const cone& c);
  void intersectCones(cone c);
  cone intersectConesSimple(cone c);
  cone symmetricalCone();

    /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out) ;


  
};


  /**
   * Overloads 'operator<<' for displaying objects of class 'FrechetShortcut'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FrechetShortcut' to write.
   * @return the output stream after the writing.
   */
  
std::ostream&
operator<< ( std::ostream & out, cone & object )
{      
  object.selfDisplay( out);
  return out;
}
  
} //namespace DGtal  


#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/representation/cone.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined cone_h

#undef cone_RECURSES
#endif // else defined(cone_RECURSES)


