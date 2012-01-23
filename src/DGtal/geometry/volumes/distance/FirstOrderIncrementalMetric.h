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
 * @file FirstOrderIncrementalMetric.h
 *
 * @author Tristan Roussillon (\c
 * tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS,
 * France
 *
 * @date 2012/01/23
 *
 * @brief Fast Marching Method for incremental distance transform
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(FirstOrderIncrementalMetric_RECURSES)
#error Recursive header files inclusion detected in FirstOrderIncrementalMetric.h
#else // defined(FirstOrderIncrementalMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FirstOrderIncrementalMetric_RECURSES

#if !defined FirstOrderIncrementalMetric_h
/** Prevents repeated inclusion of headers. */
#define FirstOrderIncrementalMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <map>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/volumes/distance/CIncrementalMetricHelper.h"
#include "DGtal/geometry/volumes/distance/FirstOrderIncrementalMetricHelpers.h"

//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class FirstOrderIncrementalMetric
  /**
   * Description of template class 'FirstOrderIncrementalMetric' <p>
   * \brief Aim: Metric mapping a point p to a value (according to 
   * to the available values of the 1-neighbors of p). 
   * It is a model of CIncrementalMetric.
   *
   * @tparam TPoint type of point
   * @tparam TMetricHelper  any model of CFirstOrderIncrementalMetricHelper 
   */
  template <typename TPoint, typename TMetricHelper 
	    = L2FirstOrderIncrementalMetricHelper<TPoint::dimension> >
  class FirstOrderIncrementalMetric
  {

    // ----------------------- Types ------------------------------
  public:


    //concept assert
    BOOST_CONCEPT_ASSERT(( CIncrementalMetricHelper<TMetricHelper> ));

    //point predicate
    typedef TPoint Point;
    typedef typename Point::Dimension Dimension;
    static const Dimension dimension = Point::dimension;

    //metric
    typedef TMetricHelper MetricHelper; 
    typedef typename TMetricHelper::Value Value; 
    BOOST_STATIC_ASSERT(( MetricHelper::dimension == Point::dimension )); 


  private: 

    //intern data types
    typedef std::pair<Point, Value> PointMetric; 
    typedef std::map<Point, Value> AcceptedPointSet; 

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Constructor.
     *
     * @param aMH a metric helper
     */
    FirstOrderIncrementalMetric(const MetricHelper& aMH = MetricHelper() );

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    FirstOrderIncrementalMetric ( const FirstOrderIncrementalMetric & other );

    
    /**
     * Destructor.
     */
    ~FirstOrderIncrementalMetric();

  
    // ----------------------- Interface --------------------------------------
  public:
    
    /** 
     * Computes the metric of @a aPoint , 
     * from the metric of its 1-neigbors
     * stored in @a aMap , using @a myMH
     *
     * @param aPoint any point
     * @param aMap STL mapping between points and metric values
     *
     * @return the metric.
     */
    Value operator()(const Point& aPoint, AcceptedPointSet& aMap);


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


      // ------------------------- Private Datas --------------------------------
  private:
    
    /**
     * Metric helper used to deduce the metric value of a new point
     * from the values of its neighbors
     */
    MetricHelper myMH; 


  private:


    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    FirstOrderIncrementalMetric & operator= ( const FirstOrderIncrementalMetric & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class FirstOrderIncrementalMetric


  /**
   * Overloads 'operator<<' for displaying objects of class 'FirstOrderIncrementalMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FirstOrderIncrementalMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename TPoint, typename TMetricHelper>
  std::ostream&
  operator<< ( std::ostream & out, const FirstOrderIncrementalMetric<TPoint, TMetricHelper> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FirstOrderIncrementalMetric.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FirstOrderIncrementalMetric_h

#undef FirstOrderIncrementalMetric_RECURSES
#endif // else defined(FirstOrderIncrementalMetric_RECURSES)
