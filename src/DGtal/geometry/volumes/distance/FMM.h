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
#include "DGtal/images/CImage.h"
#include "DGtal/images/ImageHelper.h"
#include "DGtal/kernel/sets/CDigitalSet.h"
#include "DGtal/kernel/sets/SetPredicate.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/geometry/volumes/distance/CLocalDistance.h"
#include "DGtal/geometry/volumes/distance/FirstOrderLocalDistance.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace details
  {
    template<typename T>
    class PointDistanceValueCompare {
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
   * \brief Aim: Fast Marching Method (FMM) for nd distance transforms.
   *
   * In this approach, a signed distance function is computing at 
   * each digital point by marching out from an initial set of points, 
   * for which the values of the signed distance are known. This set
   * is an initialization of the so-called <em>accepted point set</em>. 
   * Each digital point adjacent to one of the accepted points is
   * put into the so-called <em>candidate point set</em>. 
   * A tentative value is computed for its signed distance, using 
   * only the values of the accepted points lying in its neighborhood. 
   * This task is delegated to an instance of a model of CLocalDistance, 
   * which is L2FirstOrderLocalDistance by default. 
   * Then the point of smallest tentative value is added to the set of
   * accepted points. The tentative values of the candidates adjacent 
   * to the newly added point are updated using the the distance value
   * of the newly added point. The search of the point of smallest
   * tentative value is accelerated using a STL set of pairs (point, 
   * tentative value).  
   *
   * Basic usage: 
   @snippet geometry/volumes/distance/exampleFMM.cpp FMMUsage
   *
   * @tparam TImage  any model of CImage
   * @tparam TSet  any model of CDigitalSet
   * @tparam TPointPredicate  any model of CPointPredicate
   * @tparam TDistance  any model of CLocalDistance
   *
   * @see testFMM.cpp
   * @see exampleFMM.cpp
   */
  template <typename TImage, typename TSet, typename TPointPredicate, 
	    typename TDistance = L2FirstOrderLocalDistance<TImage> >
  class FMM
  {

    // ----------------------- Types ------------------------------
  public:


    //concept assert
    BOOST_CONCEPT_ASSERT(( CImage<TImage> ));
    BOOST_CONCEPT_ASSERT(( CDigitalSet<TSet> ));
    BOOST_CONCEPT_ASSERT(( CPointPredicate<TPointPredicate> ));
    BOOST_CONCEPT_ASSERT(( CLocalDistance<TDistance> ));

    typedef TImage Image; 
    typedef TSet AcceptedPointSet; 
    typedef TPointPredicate PointPredicate; 

    //points
    typedef typename Image::Point Vector;
    typedef typename Image::Point Point;
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename AcceptedPointSet::Point >::value ));
    BOOST_STATIC_ASSERT(( boost::is_same< Point, typename PointPredicate::Point >::value ));

    //dimension
    typedef typename Point::Dimension Dimension;
    static const Dimension dimension = Point::dimension;

    //distance
    typedef TDistance Distance; 
    BOOST_STATIC_ASSERT(( boost::is_same< Image, typename Distance::Image >::value ));
    typedef typename Image::Value DistanceValue; 


  private: 

    //intern data types
    typedef std::pair<Point, DistanceValue> PointDistanceValue; 
    typedef std::set<PointDistanceValue,
		     details::PointDistanceValueCompare<PointDistanceValue> > CandidatePointSet; 
    typedef unsigned long Area;

    // ------------------------- Private Datas --------------------------------
  private:
    
    /**
     * Reference on the image
     */
    Image& myImage; 

    /**
     * Reference on the set of accepted points
     */
    AcceptedPointSet& myAcceptedPoints; 

    /**
     * Set of candidate points
     */
    CandidatePointSet myCandidatePoints; 

    /**
     * Distance computer used to deduce 
     * the distance of a new point
     * from the distance of its neighbors
     */
    Distance myDC; 

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
     * DistanceValue threshold above which the propagation stops
     */
    DistanceValue myDistanceValueThreshold; 


    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     */
    FMM(Image& aImg, AcceptedPointSet& aSet,
	const PointPredicate& aPointPredicate, 
	const Distance& aDistance = Distance() );
    
    /**
     * Constructor.
     */
    FMM(Image& aImg, AcceptedPointSet& aSet, 
	const PointPredicate& aPointPredicate, 
	const Area& aAreaThreshold, const DistanceValue& aDistanceValueThreshold,
	const Distance& aDistance = Distance() );
    
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
     * of accepted points if it is possible and then 
     * update the set of candidate points. 
     *
     * @param aPoint inserted point (if inserted)
     * @param aValue its distance value (if inserted)
     *
     * @return 'true' if the point of min distance is accepted
     * 'false' otherwise.
     *
     * @see addNewAcceptedPoint
     */
    bool computeOneStep(Point& aPoint, DistanceValue& aValue);

    /** 
     * Get minimal distance value in the set of accepted points. 
     *
     * NB: in O(n log n) where n is the size of the set
     *
     * @return minimal distance value.
     */
    DistanceValue getMin() const;

    /** 
     * Get the maximal distance value in the set of accepted points. 
     *
     * NB: in O(n log n) where n is the size of the set
     *
     * @return maximal distance value.
     */
    DistanceValue getMax() const;

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


    /**
     * Initialize @a aImg and @a aSet from the points of the range [@a itb , @a ite ) 
     * Assign a distance equal to @a aDistanceValue  
     *
     * @param itb begin iterator (on points)
     * @param ite end iterator (on points)
     * @param aImg the distance image
     * @param aSet the set of points for which the distance has been assigned
     * @param aDistanceValue distance default value
     */
    template <typename TIteratorOnPoints>
    static void initFromPointsRange(const TIteratorOnPoints& itb, const TIteratorOnPoints& ite, 
				Image& aImg, AcceptedPointSet& aSet, 
				const DistanceValue& aDistanceValue);

    /**
     * Initialize @a aImg and @a aSet from the inner and outer points of the range [@a itb , @a ite ) 
     * Assign a distance equal to - @a aDistanceValue if aFlagIsPositive is 'false' (default)
     * to the inner points, but @a aDistanceValue otherwise, and conversely for the outer points.  
     *
     * @param itb begin iterator (on points)
     * @param ite end iterator (on points)
     * @param aImg the distance image
     * @param aSet the set of points for which the distance has been assigned
     * @param aDistanceValue distance default value
     */
    template <typename TIteratorOnPairs>
    static void initFromIncidentPointsRange(const TIteratorOnPairs& itb, const TIteratorOnPairs& ite, 
				   Image& aImg, AcceptedPointSet& aSet, 
				   const DistanceValue& aDistanceValue, 
				   bool aFlagIsPositive = false);

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
     * @param aPoint inserted point (if true)
     * @param aValue distance value of the inserted point (if true)
     *
     * @return 'true' if the point of min distance is accepted
     * 'false' otherwise.
     */
    bool addNewAcceptedPoint(Point& aPoint, DistanceValue& aValue);

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
  template <typename TImage, typename TSet, typename TPointPredicate, typename TDistance >
  std::ostream&
  operator<< ( std::ostream & out, const FMM<TImage, TSet, TPointPredicate, TDistance> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FMM.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FMM_h

#undef FMM_RECURSES
#endif // else defined(FMM_RECURSES)
