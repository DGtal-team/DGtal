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
 * @file IncrementalMetricComputers.h
 * @author Tristan Roussillon (\c tristan.roussillon@liris.cnrs.fr ) Laboratoire d'InfoRmatique en
 * Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 * @date 2012/01/16
 *
 * @brief Basic functors associated to metrics used by the fast marching method.
 * Header file for module IncrementalMetricComputers.cpp
 *
 * This file is part of the DGtal library.
 *
 */

#if defined(IncrementalMetricComputers_RECURSES)
#error Recursive header files inclusion detected in IncrementalMetricComputers.h
#else // defined(IncrementalMetricComputers_RECURSES)
/** Prevents recursive inclusion of headers. */
#define IncrementalMetricComputers_RECURSES

#if !defined IncrementalMetricComputers_h
/** Prevents repeated inclusion of headers. */
#define IncrementalMetricComputers_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <limits>
#include <set>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class IncrementalEuclideanMetricComputer
  /**
   * Description of template class 'IncrementalEuclideanMetricComputer' <p>
   * \brief Aim: Basic functor computing the Euclidean metric in
   * the fast marching method on nd isothetic grids. 
   *
   * @tparam  dim the space dimension
   */
  template <DGtal::Dimension dim>
  class IncrementalEuclideanMetricComputer
  {
    // ----------------------- Types ------------------------------

  public: 
    //space
    typedef DGtal::Dimension Dimension;
    static const Dimension dimension = dim;

    //distance
    typedef double Distance; 
    typedef DGtal::PointVector<dimension, Distance> Distances;   
    
  private: 
    typedef std::set<Dimension> Dimensions;
 
    // ----------------------- Standard services ------------------------------

  public: 

    /**
     * Constructor.
     */
    IncrementalEuclideanMetricComputer(const double& aGridStep = 1.0);

    /**
     * Constructor.
     */
    IncrementalEuclideanMetricComputer(const DGtal::PointVector<dimension, double>& aGridStepsVector);

    /**
     * Copy.
     */
    IncrementalEuclideanMetricComputer(const IncrementalEuclideanMetricComputer& other);


    /**
     * Destructor.
     */
    ~IncrementalEuclideanMetricComputer();

    /**
     * Returns the distance used as infinity
     * 
     * @return the infinity distance.
     */
    Distance infinity() const;


    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     *
     * @return the computed distance.
     */
    Distance compute(const Distances& aDistanceList) const;


    // ----------------------- Internals -------------------------------------

      private: 

    /**
     * Returns an approximation of the Euclidean distance 
     * at some point, knowing the distance of its neighbors
     * 
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     * @param aDimensionList  the list of relevant dimensions for the computation
     *
     * @return the computed distance.
     */
    Distance compute(const Distances& aDistanceList, Dimensions& aDimensionList) const; 


    /**
     * Returns the squared euclidean norme of the gradient 
     * of the distance function
     * 
     * @param aDistance  the distance of the point where the gradient is computed
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     *
     * @return the computed gradient norm.
     */
    double gradientNorm(const Distance& aDistance, const Distances& aDistanceList) const;

    // ----------------------- Members -------------------------------------

    /**
     * Grid steps for each dimension from 0 to dimension
     */
      DGtal::PointVector<dimension, double> myGridStepsVector; 

    /**
     * Distance used as infinity
     */
      Distance myInfinity; 

      }; // end of class IncrementalEuclideanMetricComputer

  /////////////////////////////////////////////////////////////////////////////
  // template class IncrementalLInfinityMetricComputer
  /**
   * Description of template class 'IncrementalLInfinityMetricComputer' <p>
   * \brief Aim: Basic functor computing the Linfinity metric in
   * the fast marching method on nd isothetic grids.
   *
   * @tparam  dim the space dimension
   * @tparam TDistance  distance type
   */
  template <DGtal::Dimension dim, typename TDistance>
  class IncrementalLInfinityMetricComputer
  {
    // ----------------------- Types ------------------------------

  public:
    //space
    typedef DGtal::Dimension Dimension;
    static const Dimension dimension = dim;

    //distance
    typedef TDistance Distance;
    typedef DGtal::PointVector<dimension, Distance> Distances;
    
  private:
    typedef std::set<Dimension> Dimensions;
 
    // ----------------------- Standard services ------------------------------

  public:

    /**
     * Constructor.
     */
    IncrementalLInfinityMetricComputer(const Distance& aGridStep = 1);

    /**
     * Constructor.
     */
    IncrementalLInfinityMetricComputer(const DGtal::PointVector<dimension, Distance>& aGridStepsVector);

    /**
     * Copy.
     */
    IncrementalLInfinityMetricComputer(const IncrementalLInfinityMetricComputer& other);


    /**
     * Destructor.
     */
    ~IncrementalLInfinityMetricComputer();

    /**
     * Returns the distance used as infinity
     *
     * @return the infinity distance.
     */
    Distance infinity() const;


    /**
     * Returns the Linfinity distance at some point, 
     * knowing the distance of its neighbors
     *
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     *
     * @return the computed distance.
     */
    Distance compute(const Distances& aDistanceList) const;


    // ----------------------- Internals -------------------------------------

      private:

    /**
     * Returns the Linfinity distance at some point, 
     * knowing the distance of its neighbors
     *
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     * @param aDimensionList  the list of relevant dimensions for the computation
     *
     * @return the computed distance.
     */
    Distance compute(const Distances& aDistanceList, Dimensions& aDimensionList) const;


    /**
     * Returns the squared euclidean norme of the gradient
     * of the distance function
     *
     * @param aDistance  the distance of the point where the gradient is computed
     * @param aDistanceList  the distance of the neighbors (one per dimension)
     *
     * @return the computed gradient norm.
     */
    double gradientNorm(const Distance& aDistance, const Distances& aDistanceList) const;

    // ----------------------- Members -------------------------------------

    /**
     * Grid steps for each dimension from 0 to dimension
     */
      DGtal::PointVector<dimension, double> myGridStepsVector;

    /**
     * Distance used as infinity
     */
      Distance myInfinity;

      }; // end of class IncrementalMetricComputers


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/volumes/distance/IncrementalMetricComputers.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined IncrementalMetricComputers_h

#undef IncrementalMetricComputers_RECURSES
#endif // else defined(IncrementalMetricComputers_RECURSES)
