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
 * @file ParallelSurfaceEstimator.h
 * @author Bastien Doignies (\c bastien.doignies@liris.cnrs.fr)
 * LIRIS (CNRS, UMR 5205), University of Lyon, France
 *
 * @date 2025/11/25
 *
 * Header file for module ParallelSurfaceEstimator
 *
 * This file is part of the DGtal library.
 */

#ifndef DGTAL_WITH_OPENMP
  #error ParallelSurfaceEstimator is only available when compiled with OpenMP support
#endif

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include "omp.h"
#include "DGtal/base/Common.h"
#include "DGtal/base/CountedConstPtrOrConstPtr.h"

namespace DGtal
{ 
  template<typename Domain>
  struct DomainSplitInfo
  {
    uint32_t hintSurfelCount;
    Domain domain;
  };

  namespace functors
  {
    /**
     * @brief Split a domain evenly
     *
     * This class assumes a HyperRectDomain is given.
     */
    template <typename Domain>
    struct EvenDomainSplitter
    {
      std::vector<DomainSplitInfo<Domain>> operator()(
        const Domain& domain, 
        uint32_t splitHint
      )
      {
	return { DomainSplitInfo<Domain>(0, domain) };
      }
    };
  }

  /**
   * We requires that TPointPredicate is copyable. 
   * By default estimators and predicates are duplicated within each thread. This 
   * behaviour can be disabled by calling setEstimatorRequiresDuplication(false)
   * function. This will ensure init is called only once (with surface begin/end) 
   * and requires that eval is thread safe. 
   * 
   */
  template<
    typename TKSpace,
    typename TPointPredicate,
    typename TEstimator, 
    typename TDomain = HyperRectDomain<typename TKSpace::Space>,
    typename TDomainSplitter = functors::EvenDomainSplitter<TDomain>
  >
  class ParallelSurfaceEstimator
  {
  public:
    struct Quantity
    {
      using Surfel = typename TEstimator::Surfel;
      using EQuantity = typename TEstimator::Quantity;

      Quantity(const Surfel& s, const EQuantity& eq) :
        surfel(s), value(eq)
      {}

      Quantity(const EQuantity& eq) : value(eq) 
      {}

      Surfel surfel;
      EQuantity value;
    };

    using Surfel = typename TEstimator::Surfel;

    using Domain = TDomain;
    using DomainSplitter = TDomainSplitter;

    ParallelSurfaceEstimator();

    /**
     * @brief Set number of parallel threads to run
     * 
     * @param threadCount Numbe of threads to run in parallel
     */
    void setThreadCount(uint32_t threadCount);

    /**
     * @brief Set whether each thread should have its own copy of estimator
     *
     * When set to false, the same estimator will be used within each thread. 
     * However, this means that the eval() function should be thread safe and 
     * that the init function is called only once with the full range.
     *
     * @param duplicate Whether each thread should have the same estimator copy
     */
    void setEstimatorDuplication(bool duplicate);

    /**
     * @brief Set whether each thread should have its own copy of the point predicate
     *
     * When set to false, the same point predicate will be used within each thread. 
     * However, this means that the operator() function should be thread safe.
     *
     * @param duplicate Whether each thread should have the same estimator copy
     */
    void setPointPredicateDuplication(bool duplicate);

    void attach(ConstAlias<TPointPredicate> pp);

    void attach(Alias<TEstimator> estimator);
  
    /**
     * @brief Sets the radius
     *
     * The purpose of this function is to set the radius and to fullfil CSurfelLocalEstimator requirements.
     * This init function DOESN'T initialize the underlying estimator. Actual 
     * initialization is postponned until eval is run; so that the init method 
     * is called with the correct range.
     *
     * @param h Grid step
     * @param beg Ignored
     * @param end Ignore
     *
     * @see init
     */
    template<typename It>
    void init(double h, It beg, It end);
    
    /**
     * @brief Sets the radius
     *
     * This init function DOESN'T initialize the underlying estimator. Actual 
     * initialization is postponned until eval is run; so that the init method 
     * is called with the correct range.
     *
     * As such, this overload does not take Iterator. 
     *
     * @param h Grid step
     */
    void init(double h);

    /**
     * @brief Evaluates the estimator at a given location
     *
     * Warning: This functions also call init and 
     * thus may be slower than expected.
     *
     * @tparam It Iterator type
     * @param loc The location given through an iterator
     */
    template<typename It>
    Quantity eval(It loc);
  
    /**
     * @brief Evaluates the estimator at locations given by
     * the provided domain.
     *
     * This functions ignores begin and end iterators. Only
     * the domain given to this estimator is taken into.
     *
     * @tparam It Ignored
     * @tparam OIt Output iterator
     *
     * @param beg Ignored
     * @param end Ignored
     * @param out Output iterator
     *
     * @see evalOrdered
     */
    template<typename It, typename OIt>
    OIt& eval(It beg, It end, OIt& out);
    
    /**
     * @brief Evaluates the estimator at locations given by
     * the provided domain.
     *
     * @tparam OIt Output iterator
     *
     * @param out Output iterator
     * @param ignored Here for overload resolution only. Not used by the function.
     *
     * @see eval
     * @see evalOrdered
     */
    template<typename OIt>
    OIt& eval(OIt& out, int ignored);

    /**
     * @brief Evaluates the estimator at locations given by
     * the provided domain.
     *
     * Unlike eval, this functions returns location filtered
     * and in the order provided by begin and end. 
     * Therefore, this function can be slower than just running 
     * sequential estimation:
     *   * Because the evaluation may occur on a bigger range at first
     *   * Because filtering happens
     * 
     * As elements are ordered, only a Estimator::Quantity is written
     * to the iterator. 
     *
     * @param 
     * @see eval
     */
    template<typename It, typename OIt>
    OIt evalOrdered(It beg, It end, OIt& out);
  
  private:
    CountedConstPtrOrConstPtr<TEstimator> GetEstimator(uint32_t i);
  private:
    Domain myDomain;

    double myStep; 
    uint32_t myThreadCount;

    bool myEstimatorRequiresDuplication;
    bool myPredicateRequiresDuplication;
    
    DomainSplitter mySplitter;
    CountedConstPtrOrConstPtr<TPointPredicate> myPredicate;
    CountedPtrOrPtr<TEstimator> myEstimator;
  };
}

#include "ParallelSurfaceEstimator.ih"
