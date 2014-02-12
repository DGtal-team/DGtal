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
 * @file LocalEstimatorFromSurfelFunctorAdapter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/28
 *
 * Header file for module LocalEstimatorFromSurfelFunctorAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(LocalEstimatorFromSurfelFunctorAdapter_RECURSES)
#error Recursive header files inclusion detected in LocalEstimatorFromSurfelFunctorAdapter.h
#else // defined(LocalEstimatorFromSurfelFunctorAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LocalEstimatorFromSurfelFunctorAdapter_RECURSES

#if !defined LocalEstimatorFromSurfelFunctorAdapter_h
/** Prevents repeated inclusion of headers. */
#define LocalEstimatorFromSurfelFunctorAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/Alias.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/base/CUnaryFunctor.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
#include "DGtal/topology/CSCellEmbedder.h"
#include "DGtal/topology/CDigitalSurfaceContainer.h"
#include "DGtal/topology/DigitalSurface.h"
#include "DGtal/graph/DistanceBreadthFirstVisitor.h"
#include "DGtal/geometry/volumes/distance/CMetric.h"
#include "DGtal/base/BasicFunctors.h"
#include "DGtal/geometry/surfaces/estimation/estimationFunctors/CLocalEstimatorFromSurfelFunctor.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class LocalEstimatorFromSurfelFunctorAdapter
  /**
   * Description of template class 'LocalEstimatorFromSurfelFunctorAdapter' <p>
   * \brief Aim: this class adapts any local functor on digital surface element to define
   * a local estimator.
   *
   * When we evaluate the adapted estimator at a surfel @a s, we first
   * identify the set of neighboring around @a s using a
   * DistanceBreadthFirstVisitor parametrized by a given metric. Then,
   * the estimated quantity is computed applying a functor on the
   * surfel set.
   *
   *
   * More precisely, this adapter needs a model of CMetric to define
   * the neighborhood and a model of CLocalEstimatorFromSurfelFunctor
   * to perform the local estimator computation. When sent to the
   * functor, the surfels are weighted using the distance from the
   * kernel boundary: weights are defined in [0,1] interval, 1 for the
   * center and decreasing to 0 with the distance to the specified
   * kernel radius specified during the init(). The shape of the
   * distance-to-weight function is defined by a functor of type @e
   * TConvolutionFunctor.
   *
   * Models of TConvolutionFunctor could be for instance
   * DefaultFunctor (returns the distance itself),
   * ConstValueFunctor (returns a constant value) or
   * GaussianKernelFunctor (parametrized by a sigma).
   *
   * During the @e init() method, we thus specify the gridstep @e h
   * and the radius of the ball to consider to define the
   * neighborhood. An instance of the convolution functor should be
   * passed to the constructor.
   *
   * Note that the visitor used in this class considers the distance
   * function in the ambient space (not a geodesic one for instance) on
   * canonical embedding of surfel elements (cf CanonicSCellEmbedder).
   *
   *  @tparam TDigitalSurfaceContainer any model of digital surface container concept (CDigitalSurfaceContainer)
   *  @tparam TMetric any model of CMetric to be used in the neighborhood construction.
   *  @tparam TFunctorOnSurfel an estimator on surfel set (model of CLocalEstimatorFromSurfelFunctor)
   *  @tparam TConvolutionFunctor type of  functor on double
   *  [0,1]->[0,1] to implement the response of a symmetric convolution kernel.
   */
  template <typename TDigitalSurfaceContainer, typename TMetric, 
            typename TFunctorOnSurfel, typename TConvolutionFunctor>
  class LocalEstimatorFromSurfelFunctorAdapter
  {
    // ----------------------- Standard services ------------------------------
  public:

    ///Concept Checks
    BOOST_CONCEPT_ASSERT(( CMetric<TMetric>));
    BOOST_CONCEPT_ASSERT(( CLocalEstimatorFromSurfelFunctor<TFunctorOnSurfel>));
    BOOST_CONCEPT_ASSERT(( CUnaryFunctor<TConvolutionFunctor,double,double> ));
    BOOST_CONCEPT_ASSERT(( CDigitalSurfaceContainer<TDigitalSurfaceContainer> ));

    ///Digital surface type
    typedef TDigitalSurfaceContainer DigitalSurfaceContainer;

    ///Metric type
    typedef TMetric Metric;

    ///Metric value type
    typedef typename TMetric::Value Value;

    ///Functor on surfels type
    typedef TFunctorOnSurfel FunctorOnSurfel;

    ///Functor on double to compute convolution weights
    typedef TConvolutionFunctor ConvolutionFunctor;

    ///Quantity type
    typedef typename TFunctorOnSurfel::Quantity Quantity;

  private:

    ///Embedded and type definitions
    typedef typename FunctorOnSurfel::SCellEmbedder Embedder;
    typedef std::binder1st<Metric> MetricToPoint;
    typedef Composer<Embedder, MetricToPoint, Value> VertexFunctor;
    typedef DistanceBreadthFirstVisitor< DigitalSurface< DigitalSurfaceContainer >, 
                                         VertexFunctor> Visitor;


  public:

    /**
     * Constructor.
     *
     * @param aSurface a digital surface
     * @param aMetric the metric
     *
     * @param aFunctor a functor on digital surface elements (e.g. the
     * normal or the curvature estimation)
     *
     * @param aConvolutionFunctor a functor giving the weight as a
     * function of the distance to the surfel.
     */
    LocalEstimatorFromSurfelFunctorAdapter
    ( ConstAlias< DigitalSurface< DigitalSurfaceContainer > >  aSurface,
      ConstAlias<TMetric> aMetric,
      Alias<FunctorOnSurfel>  aFunctor,
      ConstAlias<ConvolutionFunctor> aConvolutionFunctor );

    /**
     * Destructor.
     */
    ~LocalEstimatorFromSurfelFunctorAdapter();

    // ----------------------- Interface --------------------------------------
  public:


    /**
     * Initialisation of estimator parameters.
     * @param [in] h grid size (must be >0).
     * @param [in] radius radius of the ball kernel.
     *
     */
    void init(const double h,
              const Value radius);


    /**
     * @return the estimated quantity at *it
     * @param [in] it the surfel iterator at which we evaluate the quantity.
     */
    template< typename SurfelConstIterator>
    Quantity eval(const SurfelConstIterator& it) const;

    /**
     * @return the estimated quantity in the range [itb,ite)
     * @param [in] itb starting surfel iterator.
     * @param [in] ite end surfel iterator.
     * @param [in,out] result resulting output iterator
     *
     */
    template< typename SurfelConstIterator, typename OutputIterator>
    OutputIterator eval(const SurfelConstIterator& itb,
                        const SurfelConstIterator& ite,
                        OutputIterator result) const;


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

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    LocalEstimatorFromSurfelFunctorAdapter();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    LocalEstimatorFromSurfelFunctorAdapter ( const LocalEstimatorFromSurfelFunctorAdapter & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    LocalEstimatorFromSurfelFunctorAdapter & operator= ( const LocalEstimatorFromSurfelFunctorAdapter & other );

    // ------------------------- Internals ------------------------------------
  private:

    ///Digital surface member
    const DigitalSurface< DigitalSurfaceContainer > * mySurface;

    ///Functor member
    FunctorOnSurfel * myFunctor;

    ///Distance functor
    const Metric * myMetric;

    ///Grid step
    double myH;

    ///Has init been done before eval
    bool myInit;

    ///Embedder object
    const Embedder myEmbedder;

    ///Convolution functor
    const ConvolutionFunctor *myConvFunctor;

    ///Ball radius
    Value myRadius;

  }; // end of class LocalEstimatorFromSurfelFunctorAdapter

  /**
   * Overloads 'operator<<' for displaying objects of class 'LocalEstimatorFromSurfelFunctorAdapter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'LocalEstimatorFromSurfelFunctorAdapter' to write.
   * @return the output stream after the writing.
   */
  template <typename TD, typename TV, typename TF, typename TC>
  std::ostream&
  operator<< ( std::ostream & out, const LocalEstimatorFromSurfelFunctorAdapter<TD,TV,TF,TC> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation//LocalEstimatorFromSurfelFunctorAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LocalEstimatorFromSurfelFunctorAdapter_h

#undef LocalEstimatorFromSurfelFunctorAdapter_RECURSES
#endif // else defined(LocalEstimatorFromSurfelFunctorAdapter_RECURSES)
