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
 * @file CompleteCompleteVoronoiMap.h
 * @brief Linear in time distance transformation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/08/14
 *
 * Header file for module CompleteVoronoiMap.cpp
 *
 * This file is part of the DGtal library.
 *
 * @see testCompleteVoronoiMap.cpp
 */

#if defined(CompleteVoronoiMap_RECURSES)
#error Recursive header files inclusion detected in CompleteVoronoiMap.h
#else // defined(CompleteVoronoiMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CompleteVoronoiMap_RECURSES

#if !defined CompleteVoronoiMap_h
/** Prevents repeated inclusion of headers. */
#define CompleteVoronoiMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <set>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/geometry/volumes/distance/CSeparableMetric.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/base/ConstAlias.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CompleteVoronoiMap
  /**
   * Description of template class 'CompleteVoronoiMap' <p>
   * \brief Aim: Implementation of the linear in time Voronoi map
   * construction.

   * The algorithm uses a sperable process to construct Voronoi maps
   * which has been described in @cite Maurer2003PAMI @cite dcoeurjo_these
   *
   * Given a domain and a point predicate, an instance returns, for
   * each point in the domain, the closest point for which the
   * predicate if false. Following Computational Geometry
   * terminoliogy, points for which the predicate is false are "sites"
   * for the Voronoi map construction. If a point is equi-distant to
   * two sites (e.g. if the digital point belong to a Voronoi cell
   * boundary in the Euclidean space), this Voronoi map construction
   * will only keep one of them.
   *
   * The metric is specified by a model of CSeparableMetric (for
   * instance, any instance of ExactPredicateLpSeparableMetric or
   * InexactPredicateLpSeparableMetric).  If the separable metric has
   * a complexity of O(h) for its "hiddenBy" predicate, the overall
   * Voronoi construction algorithm is in @f$ O(h.d.n^d)@f$ for @f$
   * n^d@f$ domains (see class constructor). For Euclidean the @f$
   * l_2@f$ metric, the overall computation is in @f$ O(d.n^d)@f$,
   * which is optimal.
   *
   * If DGtal has been built with OpenMP support (WITH_OPENMP flag set
   * to "true"), the computation is done in parallel (multithreaded)
   * in an optimal way: on @a p processors, expected runtime is in
   * @f$ O(h.d.n^d / p)@f$.
   *
   * This class is a model of CConstImage.
   *
   * @tparam TSpace type of Digital Space (model of CSpace).
   * @tparam TPointPredicate point predicate returning true for points
   * from which we compute the distance (model of concepts::CPointPredicate)
   * @tparam TSeparableMetric a model of CSeparableMetric
   * @tparam TImageContainer any model of CImage to store the
   * CompleteVoronoiMap (default: ImageContainerBySTLVector). The space of the
   * image container and the TSpace should match. Furthermore the
   * container value type must be TSpace::Vector. Lastly, the domain
   * of the container must be HyperRectDomain.
   */
  template < typename TSpace,
             typename TPointPredicate,
             typename TSeparableMetric,
             typename TImageContainer = ImageContainerBySTLVector<HyperRectDomain<TSpace>,
                                          std::set<typename TSpace::Vector > >
          >
  class CompleteVoronoiMap
  {

  public:
    BOOST_CONCEPT_ASSERT(( concepts::CSpace< TSpace > ));
    BOOST_CONCEPT_ASSERT(( concepts::CPointPredicate<TPointPredicate> ));
    BOOST_CONCEPT_ASSERT(( concepts::CSeparableMetric<TSeparableMetric> ));

    ///Both Space points and PointPredicate points must be the same.
    BOOST_STATIC_ASSERT ((boost::is_same< typename TSpace::Point,
                          typename TPointPredicate::Point >::value )); 
 
    //ImageContainer::Domain::Space must match with TSpace
    BOOST_STATIC_ASSERT ((boost::is_same< TSpace,
                          typename TImageContainer::Domain::Space >::value )); 
   
    //ImageContainer value type must be  a digital set on point
    BOOST_STATIC_ASSERT ((boost::is_same< typename TSpace::Vector,
                          typename TImageContainer::Value::value_type >::value ));
 
    //ImageContainer domain type must be  HyperRectangular
    BOOST_STATIC_ASSERT ((boost::is_same< HyperRectDomain<TSpace>,
                          typename TImageContainer::Domain >::value ));
    
    ///Copy of the space type.
    typedef TSpace Space;

    ///Copy of the point predicate type.
    typedef TPointPredicate PointPredicate;

    ///Definition of the underlying domain type.
    typedef typename TImageContainer::Domain Domain;

    ///Definition of the separable metric type
    typedef TSeparableMetric SeparableMetric;

    ///Large integer type for SeparableMetricHelper construction.
    typedef DGtal::int64_t IntegerLong;

    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;
 
    ///Type of resulting image
    typedef TImageContainer OutputImage;
     ///Definition of the image value type.
    typedef typename OutputImage::value_type Value;
    
    ///Definition of the image value type.
    typedef typename OutputImage::ConstRange  ConstRange;

    ///Self type
    typedef CompleteVoronoiMap<TSpace, TPointPredicate, 
		       TSeparableMetric,TImageContainer> Self;
    

    /**
     * Constructor.
     * 
     * This constructor computes the Voronoi Map of a set of point
     * sites using a SeparableMetric metric.  The method associates to
     * each point satisfying the foreground predicate, the closest
     * site for which the predicate is false. This algorithm is
     * @f$ O(h.d.|domain size|)@f$ if the separable metric "hiddenBy"
     * predicate is in @f$ O(h)$@f$.
     *
     * @param aDomain a pointer to the (hyper-rectangular) domain on
     * which the computation is performed.
     *
     * @param predicate a pointer to the point predicate to define the
     * Voronoi sites (false points).
     * 
     *@param aMetric a pointer to the separable metric instance.
     */
    CompleteVoronoiMap(ConstAlias<Domain> aDomain,
               ConstAlias<PointPredicate> predicate,
               ConstAlias<SeparableMetric> aMetric);

    /**
     * Default destructor
     */
    ~CompleteVoronoiMap();

  public:
    // ------------------- ConstImage model ------------------------

    /**
     * Assignment operator from another Voronoi map.
     *
     *  @param aOtherCompleteVoronoiMap another instance of Self
     *  @return a reference to Self
     */
    Self &  operator=(const Self &aOtherVoronoiMap );
    
    /**
     * Returns a reference (const) to the Voronoi map domain.
     * @return a domain
     */
    const Domain &  domain() const
    {
      return *myDomainPtr;
    }

    
    /**
     * Returns a const range on the Voronoi map values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return myImagePtr->constRange();
    }
        
    /**
     * Access to a Voronoi value (a.k.a. vector to the closest site)
     * at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return myImagePtr->operator()(aPoint);
    }    
     
    /** 
     * @return Returns an alias to the underlying metric.
     */
    const SeparableMetric* metric() const
    {
      return myMetricPtr;
    }

    /**
     * Self Display method.
     * 
     * @param out output stream
     */
    void selfDisplay ( std::ostream & out ) const;    
   
    // ------------------- Private functions ------------------------
  private:    
    
    /**
     * Compute the Voronoi Map of a set of point sites using a
     * SeparableMetric metric.  The method associates to each point
     * satisfying the foreground predicate, the closest site for which
     * the predicate is false. This algorithm is O(h.d.|domain size|).
     */
    void compute ( ) ;


    /** 
     *  Compute the other steps of the separable Voronoi map.
     * 
     * @param [in] dim the dimension to process
     */    
    void computeOtherSteps(const Dimension dim) const;
    /** 
     * Given  a voronoi map valid at dimension @a dim-1, this method
     * updates the map to make it consistent at dimension @a dim along
     * the 1D span starting at @a row along the dimension @a
     * dim.
     * 
     * @param [in] row starting point of the 1D process.
     * @param [in] dim dimension of the update.
     */
    void computeOtherStep1D (const Point &row, 
			     const Size dim) const;
    
    
    /**
     * Insert a new site @a aSite to the list of site
     * at position @a aPos.
     *
     * @param [in] aPos the position of the list to extend.
     * @param [in] aSite the site to insert.
     */
    void insertSite(const Point &aPos, const Point &aSite) const;
    
    
    /**
     * Return the closest site in the list at position
     * @a aPos. If the list is empty, the boolean @a isEmpty is
     * set to true (and the returned point has unspecified coordinates).
     *
     * @param [in] aPos the position of the list to extend.
     * @param [out] isEmpty true if the list at @a aPos is empty, false otherwise.
     * @return the closest site at position @a aPos.
     */
    Point getSite(const Point &aPos, bool isEmpty) const;
    
    // ------------------- protected methods ------------------------
  protected:

    /** 
     * Default Constructor.
     * 
     */
    CompleteVoronoiMap();
   
    
    // ------------------- Private members ------------------------
  private:

    ///Pointer to the computation domain
    const Domain * myDomainPtr;
    
    ///Pointer to the point predicate
    const PointPredicate * myPointPredicatePtr;
    
    ///Copy of the image lower bound
    Point myLowerBoundCopy;
    
    ///Copy of the image lower bound
    Point myUpperBoundCopy;
    
    ///Value to act as a +infinity value
    Point myInfinity;
    
    ///Extent of the domain
    Point myExtent;

  protected:

    ///Pointer to the separable metric instance
    const SeparableMetric * myMetricPtr;

    ///Voronoi map image
    CountedPtr<OutputImage> myImagePtr;

  }; // end of class CompleteVoronoiMap

  /**
   * Overloads 'operator<<' for displaying objects of class 'ExactPredicateLpSeparableMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ExactPredicateLpSeparableMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename S, typename P,
            typename Sep, typename TI>
  std::ostream&
  operator<< ( std::ostream & out, const CompleteVoronoiMap<S,P,Sep,TI> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/CompleteVoronoiMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CompleteVoronoiMap_h

#undef CompleteVoronoiMap_RECURSES
#endif // else defined(CompleteVoronoiMap_RECURSES)
