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
 * @file ExactPredicateLpWeightedSeparableMetric.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/11/02
 *
 * Header file for module ExactPredicateLpWeightedSeparableMetric.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(ExactPredicateLpWeightedSeparableMetric_RECURSES)
#error Recursive header files inclusion detected in ExactPredicateLpWeightedSeparableMetric.h
#else // defined(ExactPredicateLpWeightedSeparableMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define ExactPredicateLpWeightedSeparableMetric_RECURSES

#if !defined ExactPredicateLpWeightedSeparableMetric_h
/** Prevents repeated inclusion of headers. */
#define ExactPredicateLpWeightedSeparableMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"
#include "DGtal/math/BasicMathFunctions.h"
#include "DGtal/base/CQuantity.h"
#include "DGtal/kernel/CSpace.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  
/////////////////////////////////////////////////////////////////////////////
// template class ExactPredicateLpWeightedSeparableMetric
/**
 * Description of template class 'ExactPredicateLpWeightedSeparableMetric' <p>
   * \brief Aim:
   */
  template <typename TSpace, DGtal::uint32_t p,  
            typename TPromoted=DGtal::int64_t>
  class ExactPredicateLpWeightedSeparableMetric
  {
    // ----------------------- Standard services ------------------------------
  public:
    

    ///Copy the space type
    typedef TSpace Space;
    BOOST_CONCEPT_ASSERT(( CSpace<TSpace> ));

    ///Type for points
    typedef typename Space::Point Point;
    ///Type for points
    typedef typename Point::Coordinate Abscissa;
    ///Type for vectors
    typedef typename Space::Vector Vector;
    
    ///Type for internal distance values
    typedef TPromoted Promoted;
   
    ///Type for internal distance values
    typedef TPromoted Weight;
    BOOST_CONCEPT_ASSERT(( CSignedInteger<Promoted> ));
   
    ///Type for Value (alias)
    typedef TPromoted Value;
 
    /**
     * Constructor.
     */
    ExactPredicateLpWeightedSeparableMetric();

   
    /**
     * Destructor.
     */
    ~ExactPredicateLpWeightedSeparableMetric();

  /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ExactPredicateLpWeightedSeparableMetric ( const ExactPredicateLpWeightedSeparableMetric & other ) {}
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ExactPredicateLpWeightedSeparableMetric & operator= ( const ExactPredicateLpWeightedSeparableMetric & other ) { return *this;}

    // ----------------------- Interface --------------------------------------
  public:

    // ----------------------- CPowerMetric --------------------------------------
  

    /** 
     *  Return the power distance of a point @a aPoint and a weighted
     *  point (@a aQ,@a aWq) 
     * 
     * @param aPoint 
     * @param aQ 
     * @param aWeight 
     * 
     * @return 
     */
    Weight powerDistance(const Point &aPoint, 
                         const Point &aQ, 
                         const Weight &aWq) const;


    /** 
     * Given an origin and two points, this method decides which one
     * is closest to the origin. This method should be faster than
     * comparing distance values.
     * 
     * @param origin the origin
     * @param wO the origin weight
     * @param first  the first point
     * @param wF the first point weight
     * @param second the second point
     * @param wS the second point weight
     $
     * @return a Closest enum: FIRST, SECOND or BOTH.
     */  
    DGtal::Closest closestWeighted(const Point &origin,
                                   const Point &first,
                                   const Weight &wF,
                                   const Point &second,
                                   const Weight &wS) const;
    
    // ----------------------- CPowerSeparableMetric --------------------------------------
    /** 
     * Given three sites (a,b,c) and a straight segment
     * [startingPoint,endPoint] along dimension dim, we detect if the
     * voronoi cells of a and c @e hide the voronoi cell of c on the
     * straight line.
     *
     * @pre both voronoi cells associated with @a a and @a b must
     * intersect the straight line. 
     * 
     * @param u a site
     * @param wu a weight
     * @param v a site
     * @param wu a weight
     * @param w a site
     * @param wu a weight
     * @param startingPoint starting point of the segment
     * @param endPoint end point of the segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */ 
    bool hiddenByWeighted(const Point &u, 
                          const Weight &wu,
                          const Point &v,
                          const Weight &wv,
                          const Point &w, 
                          const Weight &ww,
                          const Point &startingPoint,
                          const Point &endPoint,
                          const typename Point::UnsignedComponent dim) const;


    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */    
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:

    /** 
     * Compute the Lp distance without the computation of the power
     * 1/p. I.e. only @f$ \sum |p_i- q_i|^p@f$ is given.
     * 
     * @param aP a first point
     * @param aQ a second point
     * 
     * @return the power p of the l_p distance between aP and aQ.
     */    
    Promoted exactDistanceRepresentation(const Point &aP, const Point &aQ) const;

     /** 
     * Perform a binary search on the interval [lower,upper] to
     * detect the mid-point between u and v according to the l_p
     * distance.
     * 
     * @param udim coordinate of u along dimension dim
     * @param vdim coordinate of v along dimension dim
     * @param nu  partial distance of u (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param nv partial distance of v (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param lower interval lower bound 
     * @param upper interval upper bound
     * 
     * @return the Voronoi boundary point coordinates along dimension dim.
     */
    Abscissa binarySearchHidden(const Abscissa &udim, 
                                const Abscissa &vdim,
                                const Promoted &nu,
                                const Promoted &nv,
                                const Abscissa &lower,
                                const Abscissa &upper) const;

      
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ExactPredicateLpWeightedSeparableMetric


///////////////////////////////////////////////////////////////////////////////
//            L_2 specialization                                             //
///////////////////////////////////////////////////////////////////////////////


 template <typename TSpace,
           typename TPromoted>
 class ExactPredicateLpWeightedSeparableMetric<TSpace, 2, TPromoted>
  {
    // ----------------------- Standard services ------------------------------
  public:
    
    ///Copy the space type
    typedef TSpace Space;
    ///Type for points
    typedef typename Space::Point Point;
    ///Type for points
    typedef typename Point::Coordinate Abscissa;
    ///Type for vectors
    typedef typename Space::Vector Vector;
    
    ///Type for internal distance values
    typedef TPromoted Promoted;
    
    ///Type for internal distance values
    typedef TPromoted Weight;
    
    ///Type for Value (alias)
    typedef TPromoted Value;
    /**
     * Constructor.
     */
    ExactPredicateLpWeightedSeparableMetric();

   
    /**
     * Destructor.
     */
    ~ExactPredicateLpWeightedSeparableMetric();

  
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    ExactPredicateLpWeightedSeparableMetric ( const ExactPredicateLpWeightedSeparableMetric & other ) {}
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    ExactPredicateLpWeightedSeparableMetric & operator= ( const ExactPredicateLpWeightedSeparableMetric & other ) 
    { return *this;}

   
    // ----------------------- Interface --------------------------------------
  public:

     /** 
     *  Return the power distance of a point @a aPoint and a weighted
     *  point (@a aQ,@a aWq) 
     * 
     * @param aPoint 
     * @param aQ 
     * @param aWeight 
     * 
     * @return 
     */
    Weight powerDistance(const Point &aPoint, 
                         const Point &aQ, 
                         const Weight &aWq) const;


   /** 
     * Given an origin and two weighted points, this method decides
     * which one is closest to the origin. This method should be
     * faster than comparing distance values.
     * 
     * @param origin the origin
     * @param first  the first point
     * @param wF the first point weight
     * @param second the second point
     * @param wS the second point weight
     $
     * @return a Closest enum: FIRST, SECOND or BOTH.
     */  
    DGtal::Closest closestWeighted(const Point &origin, 
                                   const Point &first,
                                   const Weight &wF,
                                   const Point &second,
                                   const Weight &wS) const;
    

    /** 
     * Compute an exact representation of  Lp distances. We just
     *  return the sum of powers without computing the last 1/p power.
     *  I.e. only @f$ \sum |p_i- q_i|^p@f$ is given.
     * 
     * @param aP a first point
     * @param aQ a second point
     * 
     * @return the power p of the l_p distance between aP and aQ.
     */    
    Promoted exactDistanceRepresentation(const Point &aP, const Point &aQ) const;
    
    // ----------------------- CSeparableMetric --------------------------------------
    /** 
     * Given three sites (a,b,c) and a straight segment
     * [startingPoint,endPoint] along dimension dim, we detect if the
     * voronoi cells of a and c @e hide the voronoi cell of c on the
     * straight line.
     *
     * @pre both voronoi cells associated with @a a and @a b must
     * intersect the straight line. 
     * 
     * @param u a site
     * @param v a site
     * @param w a site
     * @param startingPoint starting point of the segment
     * @param endPoint end point of the segment
     * @param dim direction of the straight line
     * 
     * @return true if (a,c) hides b.
     */ 
    bool hiddenByWeighted(const Point &u, 
                          const Weight &wu,
                          const Point &v,
                          const Weight &wv,
                          const Point &w, 
                          const Weight &ww,
                          const Point &startingPoint,
                          const Point &endPoint,
                          const typename Point::UnsignedComponent dim) const;

   // ----------------------- Other services --------------------------------------
    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */    
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    // ------------------------- Protected Datas ------------------------------
  private:

   
     /** 
     * Perform a binary search on the interval [lower,upper] to
     * detect the mid-point between u and v according to the l_p
     * distance.
     * 
     * @param udim coordinate of u along dimension dim
     * @param vdim coordinate of v along dimension dim
     * @param nu  partial distance of u (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param nv partial distance of v (sum of |xj-x_i|^p) discarding
     * the term along the dimension dim
     * @param lower interval lower bound 
     * @param upper interval upper bound
     * 
     * @return the Voronoi boundary point coordinates along dimension dim.
     */
    Abscissa binarySearchHidden(const Abscissa &udim, 
                                const Abscissa &vdim,
                                const Promoted &nu,
                                const Promoted &nv,
                                const Abscissa &lower,
                                const Abscissa &upper) const;

      
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class ExactPredicateLpWeightedSeparableMetric

  /**
   * Overloads 'operator<<' for displaying objects of class 'ExactPredicateLpWeightedSeparableMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ExactPredicateLpWeightedSeparableMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename T, DGtal::uint32_t p , typename P>
  std::ostream&
  operator<< ( std::ostream & out, const ExactPredicateLpWeightedSeparableMetric<T,p,P> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/ExactPredicateLpWeightedSeparableMetric.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined ExactPredicateLpWeightedSeparableMetric_h

#undef ExactPredicateLpWeightedSeparableMetric_RECURSES
#endif // else defined(ExactPredicateLpWeightedSeparableMetric_RECURSES)
