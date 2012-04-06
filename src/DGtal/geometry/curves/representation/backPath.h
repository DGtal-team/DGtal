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
 * @file backpath.h
 * @author Isabelle Sivignon (\c isabelle.sivignon@gipsa-lab.grenoble-inp.fr )
 * gipsa-lab Grenoble Images Parole Signal Automatique (CNRS, UMR 5216), CNRS, France
 *
 * @date 2012/02/24
 *
 * Header file for module backpath.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(backpath_RECURSES)
#error Recursive header files inclusion detected in backpath.h
#else // defined(backpath_RECURSES)
/** Prevents recursive inclusion of headers. */
#define backpath_RECURSES

#if !defined backpath_h
/** Prevents repeated inclusion of headers. */
#define backpath_h



//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <boost/icl/interval_set.hpp>
#include <map>
#include "DGtal/kernel/PointVector.h"

//////////////////////////////////////////////////////////////////////////////



namespace DGtal
{
  
template <typename TIterator>
class backpath {
  
  
  typedef DGtal::PointVector<2,int> Point;
  typedef DGtal::PointVector<2,int> Vector;
  
  
 protected: 
  
  typedef struct occulter_attributes{
    double angle_min; // 
    double angle_max; // 
  } occulter_attributes;
  
  
  typedef TIterator ConstIterator;
  typedef map <ConstIterator,occulter_attributes > occulter_list;
  
  
 public:
  
  struct Tools
  {
    static int scalar_product(Vector u, Vector v)
    {
      return u[0]*v[0]+u[1]*v[1];
    }
    
    static int determinant(Vector u, Vector v)
    {
      return u[0]*v[1]-u[1]*v[0];
    }
    
    /* static double norm(Point u) */
    /* { */
    /*   return sqrt((double)u[0]*u[0]+(double) u[1]*u[1]); */
    /* } */
    
    
    static double angleVectVect(Vector u, Vector v)
    {
      return acos((double)scalar_product(u,v)/(u.norm()*v.norm()));
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
  
  
  
  int myQuad; // quadrant
  
  bool myFlag; // current state myFlag=true if we are on a backpath, false otherwise 
  
  occulter_list myOcculters;
  
  boost::icl::interval_set<double> myForbiddenIntervals;
  
  ConstIterator myIt;
  
  double myError;
  
  backpath(int q, double error);
  ~backpath();
  
  void reset();
  void addPositivePoint();
  void addNegativePoint();
  void updateBackPathFirstQuad(int d, const ConstIterator&);
  void updateOcculters();
  void updateIntervals();
  
  
};

} //namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#if !defined(BUILD_INLINE)
#include "DGtal/geometry/curves/representation/backpath.ih"
#endif


//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined backpath_h

#undef backpath_RECURSES
#endif // else defined(backpath_RECURSES)


