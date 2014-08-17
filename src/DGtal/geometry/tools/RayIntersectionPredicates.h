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
 * @file RayIntersectionPredicates.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2014/08/14
 *
 * Header file for module RayIntersectionPredicates.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(RayIntersectionPredicates_RECURSES)
#error Recursive header files inclusion detected in RayIntersectionPredicates.h
#else // defined(RayIntersectionPredicates_RECURSES)
/** Prevents recursive inclusion of headers. */
#define RayIntersectionPredicates_RECURSES

#if !defined RayIntersectionPredicates_h
/** Prevents repeated inclusion of headers. */
#define RayIntersectionPredicates_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  template <typename TPoint>
  struct RayIntersectionPredicate
  {
    
    ///Type of point
    typedef TPoint Point;
    
    ///Type of vector
    typedef TPoint Vector;
    
    ///Type of point coordinates
    typedef typename TPoint::Component Component;

    /** 
     * Constructor from a ray
     * 
     * @pre dest vector must be not null.
     * 
     * @param origin Origin of the ray 
     * @param dest vector indicating the direction of the ray
     * 
     */    
    RayIntersectionPredicate( const Point &origin, 
                              const Vector &dest)
      : myOrigin(origin), myDest(dest) 
    {
      ASSERT_MSG( dest.norm1() != NumberTraits<Component>::ZERO, "Direction must be non-null vector"); 
    }

    /** 
     * Ray-Triangle intersection predicate
     * 
     * @param v1 first vertex of the triangle
     * @param v2 second vertex of the triangle
     * @param v3 third vertex of the triangle
     *
     * @return  true if the ray intersects the triangle (v1,v2,v3)
     */
    bool operator()(const Point &v1, 
                    const Point &v2,
                    const Point &v3)
    {
      Point e1, e2;  //Edge1, Edge2
      Point P, Q, T;
      Component det, u, v;
 
      //Find vectors for two edges sharing V1
      e1 = v2 - v1;
      e2 = v3 - v1;

      //Begin calculating determinant - also used to calculate u parameter
      P = myDest.crossProduct( e2 );
      
      //if determinant is near zero, ray lies in plane of triangle
      det = e1.dot( P );
      if(det == NumberTraits<Component>::ZERO) 
        {
          return false;
        }
      
      //calculate distance from V1 to ray origin
      T =  myOrigin -  v1;
      
      //Calculate u parameter and test bound
      u = T.dot( P );  //* inv_det;
      
      if (det >  NumberTraits<Component>::ZERO)
        {
          if ((u < NumberTraits<Component>::ZERO) ||
              ( u > det))
            {
              return false;
            }
        }
      else
        {
          if ((u > NumberTraits<Component>::ZERO) ||
              ( u < det))
            {
              return false;
            }
        }
 
      //Prepare to test v parameter
      Q = T.crossProduct( e1 );
      
      //Calculate V parameter and test bound
      v = myDest.dot( Q ); 
      
      //The intersection lies outside of the triangle
      if (det >  NumberTraits<Component>::ZERO)
        {
          if ((v < NumberTraits<Component>::ZERO)  ||
              ((u+v) > det)) 
            {
              return false;
            }       
        }
      else
        {
          if ((v > NumberTraits<Component>::ZERO)  ||
              ((u+v) < det)) 
            {
              return false;
            }       
        }

      //distance to triangle must be positive
      Component t = e2.dot( Q ) ;
      if (t*det < NumberTraits<Component>::ZERO)
        return false;
       
      return true;
    }

    /** 
     * Ray-Quad intersection predicate
     * (calls two ray-triangle intersections).
     * 
     * @param v1 first vertex of the quad
     * @param v2 second vertex of the quad
     * @param v3 third vertex of the quad
     * @param v4 fourth vertex of the quad
     *
     * @return  true if the ray intersects the
     * quad (v1,v2,v3,v4)
     */
    bool operator()(const Point &v1, 
                    const Point &v2,
                    const Point &v3,
                    const Point &v4)
    {
      return (this->operator()(v1,v2,v3) ||
              this->operator()(v1,v4,v3) );
    }

    /** 
     * Ray-Surfel intersection predicate
     * (calls two ray-triangle intersections).
     * 
     * @warning The point type @a Point must be defined on "double"
     * coordinate type.
     *
     * @param aSurfel a Khalimsky surfel
     * @param kspace a Kspace defining the surfel
     *
     * @return  true if the ray intersects the surfel @a aSurfel
     */
    template < typename KSpace >
    bool operator()(const typename KSpace::Surfel &aSurfel,
                    const KSpace &kspace)
    {
      double x1,x2,x3,x4;
      double y1,y2,y3,y4;
      double z1,z2,z3,z4;
      
      Point baseQuadCenter = CanonicSCellEmbedder<KSpace>(kspace).embed( aSurfel );
      
      bool xodd = ( NumberTraits<typename KSpace::Integer>::castToInt64_t(aSurfel.myCoordinates[ 0 ]) & 1 );
      bool yodd = ( NumberTraits<typename KSpace::Integer>::castToInt64_t(aSurfel.myCoordinates[ 1 ]) & 1 );
      bool zodd = ( NumberTraits<typename KSpace::Integer>::castToInt64_t(aSurfel.myCoordinates[ 2 ]) & 1 ); 
      
      if(!zodd)
        {
          x1= baseQuadCenter[0]-0.5; y1= baseQuadCenter[1]-0.5; z1= baseQuadCenter[2]-0.5;
          x2= baseQuadCenter[0]+0.5; y2= baseQuadCenter[1]-0.5; z2= baseQuadCenter[2]-0.5;
          x3= baseQuadCenter[0]+0.5; y3= baseQuadCenter[1]+0.5; z3= baseQuadCenter[2]-0.5;
          x4= baseQuadCenter[0]-0.5; y4= baseQuadCenter[1]+0.5; z4= baseQuadCenter[2]-0.5;
        }
      else if(!yodd)
        {
          x1= baseQuadCenter[0]-0.5; y1= baseQuadCenter[1]-0.5; z1= baseQuadCenter[2]-0.5;
          x2= baseQuadCenter[0]-0.5; y2= baseQuadCenter[1]-0.5; z2= baseQuadCenter[2]+0.5;
          x3= baseQuadCenter[0]+0.5; y3= baseQuadCenter[1]-0.5; z3= baseQuadCenter[2]+0.5;
          x4= baseQuadCenter[0]+0.5; y4= baseQuadCenter[1]-0.5; z4= baseQuadCenter[2]-0.5;
        }
      else
        {
          x1= baseQuadCenter[0]-0.5; y1= baseQuadCenter[1]-0.5; z1= baseQuadCenter[2]-0.5;
          x2= baseQuadCenter[0]-0.5; y2= baseQuadCenter[1]+0.5; z2= baseQuadCenter[2]-0.5;
          x3= baseQuadCenter[0]-0.5; y3= baseQuadCenter[1]+0.5; z3= baseQuadCenter[2]+0.5;
          x4= baseQuadCenter[0]-0.5; y4= baseQuadCenter[1]-0.5; z4= baseQuadCenter[2]+0.5;
        }
      
      trace.info() << "Surfel quad = "<<Point(x1, y1, z1)<<" "<< Point(x2 ,y2, z2)<<" "
                   << Point(x3, y3, z3)<<" " << Point(x4, y4, z4)<< std::endl;
      return this->operator()(Point(x1, y1, z1), Point(x2 ,y2, z2),
                              Point(x3, y3, z3), Point(x4, y4, z4)); 
    }
    
    Point myOrigin;
    Point myDest;
    
  }; 

 

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined RayIntersectionPredicates_h

#undef RayIntersectionPredicates_RECURSES
#endif // else defined(RayIntersectionPredicates_RECURSES)
