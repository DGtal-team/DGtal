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
 * @file LpMetric.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2019/27/01
 *
 *
 * This file is part of the DGtal library.
 */

#if defined(LpMetric_RECURSES)
#error Recursive header files inclusion detected in LpMetric.h
#else // defined(LpMetric_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LpMetric_RECURSES

#if !defined LpMetric_h
/** Prevents repeated inclusion of headers. */
#define LpMetric_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <cmath>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/CSpace.h"
#include "DGtal/kernel/CEuclideanRing.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

/////////////////////////////////////////////////////////////////////////////
// template class LpMetric
/**
 * Description of template class 'LpMetric' <p>
 * \brief Aim: implements  l_p metrics.
 *
 * Given a parameter p, the class implements classical l_p
 * metric as a model of CMetricSpace. Hence, given two points
 * @f$ x=(x_0...x_{n-1})@f$, @f$ y=(y_0...y_{n-1})@f$, we define a metric as:
 *
 * @f$ distance(x,y)= \left(
 * \sum_{i=0}^{n-1} |x_i-y_i|^p\right)^{1/p}@f$
 *
 * This class performs all computations on C++ double converting the digital
 * points to Space::RealPoint
 *
 * The exponent @a p is specified at the constructor.
 *
 * @tparam TSpace the model of CSpace on which the metric is
 * defined.
 */
  template <typename TSpace>
  class LpMetric
  {
    // ----------------------- Standard services ------------------------------
  public:

    ///Copy the space type
    typedef TSpace Space;
    BOOST_CONCEPT_ASSERT(( concepts::CSpace<TSpace> ));

    ///Type for points (RealPoint for this class)
    typedef typename Space::RealPoint Point;
    ///Type for distance values
    typedef double Value;
    ///Type for raw distance values
    typedef double RawValue;

    /**
     * Constructor.
     *
     * @param anExponent the exponent (@a p) of the lp metric.
     */
    LpMetric( const double anExponent): myExponent(anExponent)
    {}


    /**
     * Destructor.
     */
    ~LpMetric()
    {}

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    LpMetric ( const LpMetric & other )
    {
      myExponent = other.myExponent;
    }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    LpMetric & operator= ( const LpMetric & other )
    {
      myExponent = other.myExponent;
      return *this;
    }

    // ----------------------- Interface --------------------------------------
  public:

    // ----------------------- CMetric --------------------------------------
    /**
     * Compute the distance between @a aP and @a aQ.
     *
     * @param aP a first point.
     * @param aQ a second point.
     *
     * @return the distance between aP and aQ.
     */
    Value operator()(const Point & aP, const Point &aQ) const
    {
      return std::pow( rawDistance(aP,aQ), 1.0/myExponent);
    }

    /**
     * Compute the raw distance between @a aP and @a aQ.
     *
     * @param aP a first point.
     * @param aQ a second point.
     *
     * @return the distance between aP and aQ.
     */
    RawValue rawDistance(const Point & aP, const Point &aQ) const
    {
      RawValue tmp=0.0;
      for(typename Point::Dimension i = 0; i < aP.size(); ++i)
        tmp += static_cast<RawValue>(std::pow(NumberTraits<typename Point::Coordinate>::castToDouble(std::abs(aP[i] - aQ[i])),
                                              myExponent));
      return tmp;
    }

    /**
     * Given an origin and two points, this method decides which one
     * is closest to the origin. This method should be faster than
     * comparing distance values.
     *
     * @param origin the origin
     * @param first  the first point
     * @param second the second point
     *
     * @return a Closest enum: FIRST, SECOND or BOTH.
     */
    Closest closest(const Point &origin,
                    const Point &first,
                    const Point &second) const
    {
      auto dfirst = rawDistance(origin,first);
      auto dsecond = rawDistance(origin,second);
      if (dfirst < dsecond)
        return ClosestFIRST;
      else
        if (dfirst > dsecond)
          return ClosestSECOND;

      return ClosestBOTH;
    }

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const
    {
      out << "[LpMetric] Lp Metric exponent=" << myExponent ;
    }

    /**
     * Checks the validity/consistency of the object.
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const
    {
      return true;
    }

    // ------------------------- Private Data --------------------------------
  private:

    ///Exponent value
    Value myExponent;

  }; // end of class LpMetric

  /**
   * Overloads 'operator<<' for displaying objects of class 'LpMetric'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'LpMetric' to write.
   * @return the output stream after the writing.
   */
  template <typename T>
  std::ostream&
  operator<< ( std::ostream & out, const LpMetric<T> & object )
  {
    object.selfDisplay( out );
    return out;
  }

} // namespace DGtal

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LpMetric_h

#undef LpMetric_RECURSES
#endif // else defined(LpMetric_RECURSES)
