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
#include <iostream>
#include "DGtal/base/Common.h"
#include <boost/static_assert.hpp>
#include "DGtal/base/CowPtr.h"
#include "DGtal/base/ConceptUtils.h"
#include "DGtal/kernel/PointVector.h"
#include "backpath.h"
#include "cone.h"
//#include "circle.h"
//////////////////////////////////////////////////////////////////////////////


#include "DGtal/helpers/StdDefs.h"

using namespace DGtal::Z2i;

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

  template <typename TIterator, typename TInteger>
class FrechetShortcut
{
    // ----------------------- Standard services ------------------------------
public:

  //entier
  
  BOOST_CONCEPT_ASSERT(( CInteger<TInteger> ) );
  typedef TInteger Integer;

  //required types
  typedef TIterator ConstIterator;
  typedef FrechetShortcut<ConstIterator,TInteger> Self; 
  typedef FrechetShortcut<std::reverse_iterator<ConstIterator>,TInteger> Reverse;
  
  typedef backpath<TIterator> Backpath; 
  
  
  //typedef vector<Integer> Point;
  
  struct Tools
  {
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
  

  
  /**
   * Writes/Displays the object on an output stream.
   * @param out the output stream where the object is written.
   */
  void selfDisplay ( std::ostream & out ) const;
  

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

  vector <Backpath> myBackpath;

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
  void selfDisplay ( std::ostream & out ) ; 
  
  
  
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
  
  template <typename TIterator, typename TInteger>
    std::ostream&
    operator<< ( std::ostream & out, FrechetShortcut<TIterator,TInteger> & object )
  {      
    object.selfDisplay( out);
    return out;
  }
  
  
  
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/representation/FrechetShortcut.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FrechetShortcut_h

#undef FrechetShortcut_RECURSES
#endif // else defined(FrechetShortcut_RECURSES)
