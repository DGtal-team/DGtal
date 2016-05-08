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
 * @file PowerMap.h
 * @brief Linear in time power map computation
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/10/24
 *
 *
 * This file is part of the DGtal library.
 *
 * @see testPowerMap.cpp
 */

#if defined(PowerMap_RECURSES)
#error Recursive header files inclusion detected in PowerMap.h
#else // defined(PowerMap_RECURSES)
/** Prevents recursive inclusion of headers. */
#define PowerMap_RECURSES

#if !defined PowerMap_h
/** Prevents repeated inclusion of headers. */
#define PowerMap_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <utility>
#include <vector>
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedPtr.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/images/ImageContainerBySTLVector.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/images/CConstImage.h"
#include "DGtal/geometry/volumes/distance/CPowerSeparableMetric.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class PowerMap
  /**
   * Description of template class 'PowerMap' <p>
   * \brief Aim: Implementation of the linear in time Power map
   * construction.

   * The algorithm uses a separable process to construct
   * Power maps as discussed in @cite dcoeurjo_pami_RDMA
   *
   * Given an image mapping points to values and a power separable
   * metric, the class computes the power map of the weighted points
   * specified in the image. Similarly to the VoronoiMap class, if two
   * points are equi-distant according to the power distance, this
   * power map will only consider one of them.
   *
   * If the separable metric has a complexity of O(h) for its
   * "hiddenByPower" predicate, the overall Power map construction
   * algorithm is in @f$ O(h.d.n^d)@f$ for @f$ n^d@f$ domains (see
   * class constructor). For Euclidean the @f$ l_2@f$ metric, the
   * overall computation is in @f$ O(d.n^d)@f$, which is optimal.
   *
   * This class is a model of CConstImage.
   *
   * @tparam TWeightImage model of CConstImage
   * @tparam TPowerSeparableMetric model of CPowerSeparableMetric
   * @tparam TImageContainer any model of CImage to store the
   * PowerMap (default: ImageContainerBySTLVector). The space of the
   * image container and the TSpace should match. Furthermore the
   * container value type must be TSpace::Vector. Lastly, the domain
   * of the container must be HyperRectDomain.
    */
  template < typename TWeightImage,
             typename TPowerSeparableMetric,
             typename TImageContainer =
             ImageContainerBySTLVector<HyperRectDomain<typename TWeightImage::Domain::Space>,
                                       typename TWeightImage::Domain::Space::Vector> >
  class PowerMap
  {

  public:

    BOOST_CONCEPT_ASSERT(( concepts::CConstImage< TWeightImage > ));
    BOOST_CONCEPT_ASSERT(( concepts::CPowerSeparableMetric<TPowerSeparableMetric> ));

    ///Copy of the distance image types
    typedef TWeightImage WeightImage;
    typedef typename TWeightImage::Value Weight;
    typedef typename WeightImage::Domain::Space Space;
    typedef typename Space::Vector Vector;
    typedef typename Space::Point Point;
    typedef typename Space::Dimension Dimension;
    typedef typename Space::Size Size;
    typedef typename Space::Point::Coordinate Abscissa;

    //ImageContainer::Domain::Space must match with TSpace
    BOOST_STATIC_ASSERT ((boost::is_same< typename TWeightImage::Domain::Space,
                          typename TImageContainer::Domain::Space >::value ));

    //ImageContainer value type must be  TSpace::Vector
    BOOST_STATIC_ASSERT ((boost::is_same< typename TWeightImage::Domain::Space::Vector,
                          typename TImageContainer::Value >::value ));

    //ImageContainer domain type must be  HyperRectangular
    BOOST_STATIC_ASSERT ((boost::is_same< HyperRectDomain<typename TWeightImage::Domain::Space>,
					  typename TImageContainer::Domain >::value ));

    ///Definition of the underlying domain type.
    typedef typename TImageContainer::Domain Domain;

    ///We construct the type associated to the separable metric
    typedef TPowerSeparableMetric PowerSeparableMetric;

    ///Type of resulting image
    typedef TImageContainer OutputImage;

    ///Definition of the image model value type.
    typedef Vector Value;
    ///Definition of the image value type.
    typedef typename OutputImage::ConstRange  ConstRange;

    ///Self type
  typedef PowerMap<TWeightImage, TPowerSeparableMetric, TImageContainer> Self;


    /**
     * Constructor.
     *
     * This constructor computes the Power Map of a set of point
     * sites using a SeparableMetric metric.  The method associates to
     * each point satisfying the foreground predicate, the closest
     * site for which the predicate is false.
     *
     * All parameters are aliased in this class.
     *
     * @param aDomain defines the (hyper-rectangular) domain on which
     * the computation is performed.
     * @param aWeightImage an image
     * returning the weight for some points
     * @param aMetric a power
     * seprable metric instance.
     */
    PowerMap(ConstAlias<Domain> aDomain,
             ConstAlias<WeightImage> aWeightImage,
             ConstAlias<PowerSeparableMetric> aMetric);

    /**
     * Default destructor
     */
    ~PowerMap();

  public:
    // ------------------- ConstImage model ------------------------

    /**
     * Assignment operator from another Power map.
     *
     *  @param aOtherPowerMap another instance of Self
     *  @return a reference to Self
     */
    Self &  operator=(const Self &aOtherPowerMap );

    /**
     * Returns a reference (const) to the Power map domain.
     *  @return a domain
     */
    const Domain &  domain() const
    {
      return *myDomainPtr;
    }


    /**
     * Returns a const range on the Power map values.
     *  @return a const range
     */
    ConstRange constRange() const
    {
      return myImagePtr->constRange();
    }

    /**
     * Access to a Power value (a.k.a. vector to the closest site) at a point.
     *
     * @param aPoint the point to probe.
     */
    Value operator()(const Point &aPoint) const
    {
      return myImagePtr->operator()(aPoint);
    }

    /**
     * @return  Returns the underlying metric.
     */
    const PowerSeparableMetric* metricPtr() const
    {
      return myMetricPtr;
    }

    /**
     * @return  Returns the underlying weight image.
     */
    const WeightImage* weightImagePtr() const
    {
      return myWeightImagePtr;
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
     * Compute the Power Map of a set of point sites using a
     * SeparableMetric metric.  The method associates to each point
     * satisfying the foreground predicate, the closest site for which
     * the predicate is false. This algorithm is O(d.|domain size|).
     */
    void compute ( ) ;


    /**
     *  Compute the other steps of the separable Power map.
     *
     * @param dim the dimension to process
     */
    void computeOtherSteps(const Dimension dim) const;
    /**
     * Given  a voronoi map valid at dimension @a dim-1, this method
     * updates the map to make it consistent at dimension @a dim along
     * the 1D span starting at @a row along the dimension @a
     * dim.
     *
     * @param row starting point of the 1D process.
     * @param dim dimension of the update.
     */
    void computeOtherStep1D (const Point &row,
			     const Dimension dim) const;

    // ------------------- protected methods ------------------------
  protected:

    /**
     * Default Constructor.
     *
     */
    PowerMap();


    // ------------------- Private members ------------------------
  private:

    ///Pointer to the computation domain
    const Domain * myDomainPtr;

    ///Copy of the image lower bound
    Point myLowerBoundCopy;

    ///Copy of the image lower bound
    Point myUpperBoundCopy;

    ///Value to act as a +infinity value
    Point myInfinity;

  protected:
    ///Pointer to the separable metric instance
    const PowerSeparableMetric * myMetricPtr;

    ///Power map image
    CountedPtr<OutputImage> myImagePtr;

    ///Pointer to the point predicate
    const WeightImage * myWeightImagePtr;


  }; // end of class PowerMap

 /**
   * Overloads 'operator<<' for displaying objects of class 'ExactPredicateLpSeparableMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'ExactPredicateLpSeparableMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename W,
            typename Sep,
            typename Image>
  std::ostream&
  operator<< ( std::ostream & out, const PowerMap<W,Sep,Image> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/PowerMap.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined PowerMap_h

#undef PowerMap_RECURSES
#endif // else defined(PowerMap_RECURSES)
