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
 * @file FMM.h
 *
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2012/01/17
 *
 * @brief Fast Marching Method for incremental distance transform
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(FMM_RECURSES)
#error Recursive header files inclusion detected in FMM.h
#else // defined(FMM_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FMM_RECURSES

#if !defined FMM_h
/** Prevents repeated inclusion of headers. */
#define FMM_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/base/Exceptions.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace details
  {
    template<typename T>
    class PointMetricValueCompare {
    public: 
      bool operator()(const T& a, const T& b) 
        {
          if ( std::abs(a.second) == std::abs(b.second) ) 
            { //point comparison
              return (a.first < b.first); 
            }
          else //distance comparison
	    //(in absolute value in order to deal with
	    //signed distance values) 
            return ( std::abs(a.second) < std::abs(b.second) ); 
        }
    };
  }

  /////////////////////////////////////////////////////////////////////////////
  // template class FMM
  /**
   * Description of template class 'FMM' <p>
   * \brief Aim: Fast Marching Method (FMM).
   *
   * @tparam TMetric  any model of CIncrementalMetric
   * @tparam TPointPredicate  any model of CPointPredicate
   */
  template <typename TMetric, typename TPointPredicate >
  class FMM
  {

    // ----------------------- Types ------------------------------
  public:


    //concept assert
    //BOOST_CONCEPT_ASSERT(( CIncrementalMetric<TMetric> )); concept TODO
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));

    //point predicate
    typedef TPointPredicate PointPredicate; 
    typedef typename PointPredicate::Point Vector;
    typedef typename PointPredicate::Point Point;
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename TMetric::Point >::value ));

    typedef typename Point::Dimension Dimension;
    static const Dimension dimension = Point::dimension;

    //distance
    typedef TMetric Metric; 
    typedef typename TMetric::Value MetricValue; 


  private: 

    //intern data types
    typedef std::pair<Point, MetricValue> PointMetricValue; 
    typedef std::set<PointMetricValue,details::PointMetricValueCompare<PointMetricValue> > CandidatePointSet; 
    typedef std::map<Point, MetricValue> AcceptedPointSet; 
    typedef unsigned long Area;

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    FMM(AcceptedPointSet& aSet, const Metric& aM, 
	const PointPredicate& aPointPredicate = PointPredicate() );
    
    /**
     * Constructor.
     */
    FMM(AcceptedPointSet& aSet, const Metric& aM, 
	const PointPredicate& aPointPredicate, 
	const Area& aAreaThreshold, const MetricValue& aMetricValueThreshold);
    
    /**
     * Destructor.
     */
    ~FMM();

  
    // ----------------------- Interface --------------------------------------
  public:
    
    /** 
     * Computes the distance map
     */
    void compute();
 
    /** 
     * Insert the candidate of min distance into the set 
     * of accepted points and update the set of candidate points. 
     *
     * @return 'true' if the point of min distance is accepted
     * 'false' otherwise.
     *
     * @see addNewAcceptedPoint
     */
    bool computeOneStep();

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

      // ------------------------- static functions for init --------------------

    // /**
    //  * Initialize @a aMap from the inner boundary of the set of points P
    //  * of the range [@a itb , @a ite ) such that @a aPredicate(P) returns 'true'
    //  * for each P of the set.
    //  * Assign a distance equal to @a aMetricValue
    //  */
    // template <typename TDomainIterator, typename TImplicitObject>
    // static void initInnerPoints(const TDomainIterator& itb, const TDomainIterator& ite, 
    // 					const TImplicitObject& aPredicate, 
    // 					AcceptedPointSet& aMap, 
    // 					const MetricValue& aMetricValue);

    // /**
    //  * Initialize @a aMap from the inner and outer boundaries of the set of points P
    //  * of the range [@a itb , @a ite ) such that @a aPredicate(P) returns 'true'
    //  * for each P of the set.
    //  * Assign a distance equal to - @a aMetricValue if aFlagIsPositive is 'false' (default)
    //  * to the inner points, but @a aMetricValue otherwise, and conversely for the outer points.  
    //  */
    // template <typename TDomainIterator, typename TImplicitObject>
    // static void initIncidentPoints(const TDomainIterator& itb, const TDomainIterator& ite, 
    // 					     const TImplicitObject& aPredicate, 
    // 					     AcceptedPointSet& aMap, 
    // 					     const MetricValue& aMetricValue, 
    // 					     bool aFlagIsPositive = false);

    /**
     * Initialize @a aMap from the points of the range [@a itb , @a ite ) 
     * Assign a distance equal to @a aMetricValue  
     */
    template <typename TIteratorOnPoints>
    static void initInnerPoints(const TIteratorOnPoints& itb, const TIteratorOnPoints& ite, 
					AcceptedPointSet& aMap, 
					const MetricValue& aMetricValue);

    /**
     * Initialize @a aMap from the inner and outer points of the range [@a itb , @a ite ) 
     * Assign a distance equal to - @a aMetricValue if aFlagIsPositive is 'false' (default)
     * to the inner points, but @a aMetricValue otherwise, and conversely for the outer points.  
     */
    template <typename TIteratorOnPairs>
    static void initIncidentPoints(const TIteratorOnPairs& itb, const TIteratorOnPairs& ite, 
					     AcceptedPointSet& aMap, 
					     const MetricValue& aMetricValue, 
					     bool aFlagIsPositive = false);

      // ------------------------- Private Datas --------------------------------
  private:
    
    /**
     * Reference on the set of accepted points
     */
    AcceptedPointSet& myAcceptedPoints; 

    /**
     * Set of candidate points
     */
    CandidatePointSet myCandidatePoints; 

    /**
     * Metric computer used to deduce the distance of a new point
     * from the distance of its neighbors
     */
    Metric myM; 

    /**
     * Constant reference on a point predicate that returns 
     * 'true' inside the domain 
     * where the distance transform is performed
     */
    const PointPredicate& myPointPredicate; 

    /**
     * Area threshold (in number of accepted points) 
     * above which the propagation stops
     */
    Area myAreaThreshold; 

    /**
     * MetricValue threshold above which the propagation stops
     */
    MetricValue myMetricValueThreshold; 

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    FMM ( const FMM & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    FMM & operator= ( const FMM & other );

    // ------------------------- Internals ------------------------------------
  private:

    /** 
     * Initialize the set of candidate points
     */
    void init();
    
    /** 
     * Insert the candidate of min distance into the set 
     * of accepted points and update the set of candidate points. 
     *
     * @return 'true' if the point of min distance is accepted
     * 'false' otherwise.
     */
    bool addNewAcceptedPoint();

    /** 
     * Update the set of candidate points with the neigbors of @a aPoint. 
     *
     * @param aPoint any point
     */
    void update(const Point& aPoint);

    /** 
     * Test a new point as a candidate. 
     * If it is not yet accepted 
     * and if the point predicate return 'true', 
     * compute its distance and insert it 
     * into the set candidate points. 
     *
     * @param aPoint any point
     *
     * @return 'true' if inserted,
     * 'false' otherwise.
     */
    bool addNewCandidate(const Point& aPoint);


  }; // end of class FMM


  /**
   * Overloads 'operator<<' for displaying objects of class 'FMM'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FMM' to write.
   * @return the output stream after the writing.
   */
  template <typename TMetric, typename TPointPredicate>
  std::ostream&
  operator<< ( std::ostream & out, const FMM<TMetric, TPointPredicate> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FMM.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FMM_h

#undef FMM_RECURSES
#endif // else defined(FMM_RECURSES)
