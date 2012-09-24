
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
 * @file FrechetShortcut.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/02/24
 *
 * Header file for module FrechetShortcut.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(FrechetShortcut_RECURSES)
#error Recursive header files inclusion detected in FrechetShortcut.h
#else // defined(FrechetShortcut_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FrechetShortcut_RECURSES

#if !defined FrechetShortcut_h
/** Prevents repeated inclusion of headers. */
#define FrechetShortcut_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include "DGtal/base/Common.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/kernel/PointVector.h"
#include <boost/icl/interval_set.hpp>
#include <map>

//////////////////////////////////////////////////////////////////////////////

#include "DGtal/geometry/curves/SegmentComputerUtils.h"

namespace DGtal
{
  
  /////////////////////////////////////////////////////////////////////////////
  // class FrechetShortcut
  /**
   * Description of class 'FrechetShortcut' <p>
   * \brief Aim:
   * Computation of the longest shortcut according to the Fréchet distance for a given error
   * This class is a model of the concept CForwardSegmentComputer
   */
  
  template <typename TIterator,typename TInteger = typename IteratorCirculatorTraits<TIterator>::Value::Coordinate>
    class FrechetShortcut
    {
      // ----------------------- Standard services ------------------------------
    public:
    
    //entier
    
    BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
    typedef TInteger Integer;
    
    
    
    //typedef DGtal::PointVector<2,Integer> Point;
    //typedef DGtal::PointVector<2,Integer> Vector;
    
    //required types
    typedef TIterator ConstIterator;
    typedef FrechetShortcut<ConstIterator,Integer> Self; 
    typedef FrechetShortcut<std::reverse_iterator<ConstIterator>,Integer> Reverse;
    
    //2D point and 2D vector
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Point; 
    typedef typename IteratorCirculatorTraits<ConstIterator>::Value Vector; 
    

    // Class backpath: data structures and methods to handle the backpath update
    class backpath
    {
    private:
      /**
       * Pointer to the FrechetShortcut
       */
      const FrechetShortcut<ConstIterator,Integer> *myS;
      
    protected: 
      
      typedef struct occulter_attributes{
	double angle_min; // 
	double angle_max; // 
      } occulter_attributes;
      
      
      typedef map <ConstIterator,occulter_attributes > occulter_list;
      
    public:
      friend class FrechetShortcut<ConstIterator,Integer>;
      
      
    public:
      
      int myQuad; // quadrant
      
      bool myFlag; // current state myFlag=true if we are on a backpath, false otherwise 
      
      occulter_list myOcculters;
      
      boost::icl::interval_set<double> myForbiddenIntervals;
      
      ConstIterator myIt; // pointer to the next point to be scanned: set to myEnd + 1
      
    // Default constructor
      backpath();
      
      backpath(const FrechetShortcut<ConstIterator,Integer> *s ,int q);
      //backpath(int q, double error);
      ~backpath();
      
      void reset();
      void addPositivePoint();
      void addNegativePoint();
      void updateBackPathFirstQuad(int d, const ConstIterator&);
      //void updateBackPathFirstQuad(int d);
      void updateOcculters();
      void updateIntervals();
      
   
    }; // End of class backpath


    // Class cone: data structures and methods to handle the cone update
    // used to test if the width of the shortcut is lower than the error 
    class cone{
      
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
    
    

    
    struct Tools
    {
      static  bool isBetween(double i, double a, double b, double n)
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
      

      static int scalar_product(Vector u, Vector v)
      {
	return u[0]*v[0]+u[1]*v[1];
      }
      
      static int determinant(Vector u, Vector v)
      {
	return u[0]*v[1]-u[1]*v[0];
      }
      
      
      static double angleVectVect(Vector u, Vector v)
      {
	return acos((double)scalar_product(u,v)/(u.norm()*v.norm()));
      }
	



    static  int computeChainCode(Point p, Point q)
    {
      int d;
      int x2 = q[0];
      int y2 = q[1];
      int x1 = p[0];
      int y1 = p[1];
      
      if(x2-x1==0)
	if(y2-y1==1)
	  d=2;
	else
	  d=6;
      else
	if(x2-x1==1)
	  if(y2-y1==0)
	    d=0;
	  else
	    if(y2-y1==1)
	      d=1;
	    else
	      d=7;
	else
	  if(y2-y1==0)
	    d=4;
	  else
	    if(y2-y1==1)
	      d=3;
	    else
	      d=5;
      return d;
    }
    
    // compute the quadrant of the direction pq
    static int computeQuadrant(Point p, Point q)
    {
      int d;
      int x = q[0]-p[0];
      int y = q[1]-p[1];
      
      if(x>=0)
	if(y>=0)
	  if(x>y)
	    d=0; // 0 <= y < x  
	  else
	    {
	      if(x!=0)
		d=1; // 0 <= x <= y
	    }
	else
	  if(x>=abs(y)) 
	    d=7; // 0 < abs(y) <= x 
	  else
	    d=6; // 0 <= x < abs(y)
      
      if(x<=0)
	if(y>0)
	  if(abs(x)>=y)
	    d=3; // 
	  else
	    d=2;
	else
	  if(abs(x)>abs(y))
	    d=4;
	  else
	    if(x!=0)
	      d=5;
      return d;
      
    }
    
    static vector <double> rot_vect(vector <double> v, int quad)
    {
      double angle = -quad*M_PI/4;
      vector <double> res(2);
      res[0] = v[0]*cos(angle)-v[1]*sin(angle);
      res[1] = v[0]*sin(angle)+v[1]*cos(angle);
      
      return res;
    }
    
    static int rot(int d, int quad)
    {
      return (d-quad+8)%8;
    }
    
    static Vector chainCode2Vect(int d)
    {
      Vector v;
      switch(d){
	
      case 0:{
	v[0] = 1;
	v[1] = 0;
	break;
      }
      case 1:{
	v[0] = 1;
	v[1] = 1;
	break;
      }
      case 2:{
	v[0] = 0;
	v[1] = 1;
	break;
      }
      case 3:{
	v[0] = -1;
	v[1] = 1;
	break;
      }
      case 4:{
	v[0] = -1;
	v[1] = 0;
	break;
      }
      case 5:{
	v[0] = -1;
	v[1] = -1;
	break;
      }
      case 6:{
	v[0] = 0;
	v[1] = -1;
	break;
      }
      case 7:{
	v[0] = 1;
	v[1] = -1;
	break;
      } 
	
      }
      
      return v;
    }
    
    
  };
  
    


  /**
   * Default constructor.
   * not valid
   */
  FrechetShortcut();
  
  /**
   * Constructor with initialisation
   * @param it an iterator on 2D points
   */
  FrechetShortcut(double error);

  /**
   * Initialisation.
   * @param it an iterator on 2D points
   */

  void init(const ConstIterator& it);
  
  FrechetShortcut getSelf();

  /**
   * Copy constructor.
   * @param other the object to clone.
   */
  FrechetShortcut ( const Self & other );
  
  /**
   * Assignment.
   * @param other the object to copy.
   * @return a reference on 'this'.
   */
  FrechetShortcut & operator= ( const Self & other );

    /**
     * @return a reverse version of '*this'.
     */
  Reverse getReverse() const;
  
  /**
   * Equality operator.
   * @param other the object to compare with.
   * @return 'true' either if the leaning points perfectly match
   * or if the first leaning points match to the last ones
   * (same DSS scanned in the reverse way) 
   * and 'false' otherwise
   */
  
  bool operator==( const Self & other ) const;
    
    /**
     * Difference operator.
     * @param other the object to compare with.
     * @return 'false' if equal
     * 'true' otherwise
     */
  bool operator!=( const Self & other ) const;
  

  
  /**
   * Destructor.
   */
  ~FrechetShortcut(){};

    // ----------------------- Interface --------------------------------------
public:

  /**
   * Tests whether the FrechetShortcut can be extended at the front.
   *  
   * @return 'true' if yes, 'false' otherwise.
   */
  bool isExtendableForward();
  
  /**
   * Tests whether the FrechetShortcut can be extended at the front.
   * Extend the FrechetShortcut if yes.
   * @return 'true' if yes, 'false' otherwise.
   */
  bool extendForward();
    
  
  // ---------------------------- Accessors ----------------------------------
  
  
  /**
   *  
   * @return begin iterator of the FrechetShortcut range.
     */
  ConstIterator begin() const;
  /**
   * @return end iterator of the FrechetShortcut range.
   */
  ConstIterator end() const;
  

  

  public:  
  
  /**
   * @return the name of the class.
   */
  std::string className() const;
  
  
  /**
   * Checks the validity/consistency of the object.
   * @return 'true' if the object is valid, 'false' otherwise.
   */
  bool isValid() const;
  
  // ------------------------- Protected Datas ------------------------------
 protected:
  
    double myError;
    
    vector <backpath> myBackpath;
    
    cone myCone;
    
    /**
     * ConstIterator pointing to the back of the DSS
     */
    ConstIterator myBegin;
    /**
     * ConstIterator pointing to the front of the DSS
     */
    ConstIterator myEnd;
  

  
  // ------------------------- Private Datas --------------------------------
 private:

    
  
  // ------------------------- Hidden services ------------------------------
 
 public:

  bool updateBackpath(); 
  
  bool testUpdateBackpath();
  
  bool isBackpathOk();
  
  void resetBackpath();
  
  void resetCone();
  
  bool testUpdateWidth();
  
  cone computeNewCone();
  
  bool updateWidth();
  
  bool isWidthOk();
  

    
  /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const ; 
  
  
  
  // ------------------------- Internals ------------------------------------
 private:
  
  }; // end of class FrechetShortcut
  

  // Utils
  
  
  /**
   * Overloads 'operator<<' for displaying objects of class 'FrechetShortcut'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FrechetShortcut' to write.
   * @return the output stream after the writing.
   */
  template <typename TIterator,typename TInteger>
  std::ostream&
    operator<< ( std::ostream & out, const FrechetShortcut<TIterator,TInteger> & object );


  
  
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/FrechetShortcut.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FrechetShortcut_h

#undef FrechetShortcut_RECURSES
#endif // else defined(FrechetShortcut_RECURSES)
