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
 * @file FirstOrderIncrementalMetricHelpers.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/01/16
 *
 * @brief Basic functors associated to metrics used by the fast marching method.
 * Header file for module FirstOrderIncrementalMetricHelpers.cpp
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(FirstOrderIncrementalMetricHelpers_RECURSES)
#error Recursive header files inclusion detected in FirstOrderIncrementalMetricHelpers.h
#else // defined(FirstOrderIncrementalMetricHelpers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define FirstOrderIncrementalMetricHelpers_RECURSES

#if !defined FirstOrderIncrementalMetricHelpers_h
/** Prevents repeated inclusion of headers. */
#define FirstOrderIncrementalMetricHelpers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <set>
#include <boost/array.hpp>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class L2FirstOrderIncrementalMetricHelper
  /**
   * Description of template class 'L2FirstOrderIncrementalMetricHelper' <p>
   * \brief Aim: Basic functor computing the Euclidean metric in
   * the fast marching method on nd isothetic grids. 
   *
   * @tparam  dim the space dimension
   *
   * @see FirstOrderIncrementalMetric
   * @see FMM
   */
  template <DGtal::Dimension dim>
  class L2FirstOrderIncrementalMetricHelper
  {
    // ----------------------- Types ------------------------------

  public: 
    //space
    typedef DGtal::Dimension Dimension;
    static const Dimension dimension = dim;

    //distance
    typedef double Value; 
    typedef boost::array<Value, dimension> Values;   
    
  private: 
    typedef std::set<Dimension> Dimensions;
 
    // ----------------------- Standard services ------------------------------

  public: 

    /**
     * Constructor.
     */
    L2FirstOrderIncrementalMetricHelper(const double& aGridStep = 1.0);

    /**
     * Constructor.
     */
    L2FirstOrderIncrementalMetricHelper(const boost::array<double, dimension>& aGridStepsVector);

    /**
     * Copy.
     */
    L2FirstOrderIncrementalMetricHelper(const L2FirstOrderIncrementalMetricHelper& other);


    /**
     * Destructor.
     */
    ~L2FirstOrderIncrementalMetricHelper();

    /**
     * Returns the value used  when not yet computed
     * 
     * @return unknown value.
     */
    Value unknownValue() const;


    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of the neighbors (one per dimension)
     *
     * @return the computed distance.
     */
    Value compute(const Values& aValueList) const;

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

    // ----------------------- Internals -------------------------------------

      private: 

    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aValueList  the distance of the neighbors (one per dimension)
     * @param aDimensionList  the list of relevant dimensions for the computation
     * @return the computed distance.
     */
    Value compute(const Values& aValueList, 
		     Dimensions& aDimensionList) const; 


    /**
     * Returns the squared euclidean norme of the gradient 
     * of the distance function
     * 
     * @param aValue  the distance of the point where the gradient is computed
     * @param aValueList  the distance of the neighbors (one per dimension)
     *
     * @return the computed gradient norm.
     */
    double gradientNorm(const Value& aValue, const Values& aValueList) const;

    // ----------------------- Members -------------------------------------

    /**
     * Grid steps for each dimension from 0 to dimension-1
     */
      boost::array<double, dimension> myGridStepsVector; 

    /**
     * Unknown value
     */
      Value myUnknownValue; 

      }; // end of class L2FirstOrderIncrementalMetricHelper

  /**
   * Overloads 'operator<<' for displaying objects of class 'FirstOrderIncrementalMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FirstOrderIncrementalMetric' to write.
   * @return the output stream after the writing.
   */
  template <DGtal::Dimension dim>
  std::ostream&
  operator<< ( std::ostream & out, const L2FirstOrderIncrementalMetricHelper<dim> & object );



  /////////////////////////////////////////////////////////////////////////////
  // template class LInfinityFirstOrderIncrementalMetricHelper
  /**
   * Description of template class 'LInfinityFirstOrderIncrementalMetricHelper' <p>
   * \brief Aim: Basic functor computing the Linfinity metric in
   * the fast marching method on nd isothetic grids.
   *
   * @tparam  dim the space dimension
   * @tparam TValue  value type for the metric
   *
   * @see FirstOrderIncrementalMetric
   * @see FMM
   */
  template <DGtal::Dimension dim, typename TValue = double>
  class LInfinityFirstOrderIncrementalMetricHelper
  {
    // ----------------------- Types ------------------------------

  public:
    //space
    typedef DGtal::Dimension Dimension;
    static const Dimension dimension = dim;

    //distance
    typedef TValue Value;
    BOOST_STATIC_ASSERT( std::numeric_limits<Value>::has_infinity ); 
    typedef boost::array<Value, dimension> Values;
    
  private:
    typedef std::set<Dimension> Dimensions;
 
    // ----------------------- Standard services ------------------------------

  public:

    /**
     * Constructor.
     */
    LInfinityFirstOrderIncrementalMetricHelper(const Value& aGridStep = 1);

    /**
     * Constructor.
     */
    LInfinityFirstOrderIncrementalMetricHelper(const boost::array<Value, dimension>& aGridStepsVector);

    /**
     * Copy.
     */
    LInfinityFirstOrderIncrementalMetricHelper(const LInfinityFirstOrderIncrementalMetricHelper& other);


    /**
     * Destructor.
     */
    ~LInfinityFirstOrderIncrementalMetricHelper();

    /**
     * Returns the value used when not yet computed
     *
     * @return unkown value.
     */
    Value unknownValue() const;


    /**
     * Returns the Linfinity distance at some point, 
     * knowing the distance of its neighbors
     *
     * @param aValueList  the distance of the neighbors (one per dimension)
     *
     * @return the computed distance.
     */
    Value compute(const Values& aValueList) const;

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

    // ----------------------- Internals -------------------------------------

      private:

    /**
     * Returns the Linfinity distance at some point, 
     * knowing the distance of its neighbors
     *
     * @param aValueList  the distance of the neighbors (one per dimension)
     * @param aDimensionList  the list of relevant dimensions for the computation
     *
     * @return the computed distance.
     */
    Value compute(const Values& aValueList, Dimensions& aDimensionList) const;



    // ----------------------- Members -------------------------------------

    /**
     * Grid steps for each dimension from 0 to dimension
     */
      boost::array<double, dimension> myGridStepsVector;

    /**
     * Unknown value
     */
      Value myUnknownValue;

      }; // end of class FirstOrderIncrementalMetricHelpers


  /**
   * Overloads 'operator<<' for displaying objects of class 'FirstOrderIncrementalMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'FirstOrderIncrementalMetric' to write.
   * @return the output stream after the writing.
   */
  template <DGtal::Dimension dim, typename TValue>
  std::ostream&
  operator<< ( std::ostream & out, 
	       const LInfinityFirstOrderIncrementalMetricHelper<dim, TValue> & object );


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/FirstOrderIncrementalMetricHelpers.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined FirstOrderIncrementalMetricHelpers_h

#undef FirstOrderIncrementalMetricHelpers_RECURSES
#endif // else defined(FirstOrderIncrementalMetricHelpers_RECURSES)
