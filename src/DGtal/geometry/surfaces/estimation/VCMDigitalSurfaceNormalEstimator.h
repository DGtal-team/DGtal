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
 * @file VCMDigitalSurfaceNormalEstimator.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2014/02/13
 *
 * Header file for module VCMDigitalSurfaceNormalEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(VCMDigitalSurfaceNormalEstimator_RECURSES)
#error Recursive header files inclusion detected in VCMDigitalSurfaceNormalEstimator.h
#else // defined(VCMDigitalSurfaceNormalEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define VCMDigitalSurfaceNormalEstimator_RECURSES

#if !defined VCMDigitalSurfaceNormalEstimator_h
/** Prevents repeated inclusion of headers. */
#define VCMDigitalSurfaceNormalEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/geometry/surfaces/estimation/VoronoiCovarianceMeasureOnDigitalSurface.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class VCMDigitalSurfaceNormalEstimator
  /**
   * Description of template class 'VCMDigitalSurfaceNormalEstimator'
   * <p> \brief Aim: This class adapts a
   * VoronoiCovarianceMeasureOnDigitalSurface to be a model of
   * CNormalVectorEstimator. It returns the estimated VCM surface
   * \b outward normal for given surfels.
   *
   * @tparam TDigitalSurfaceContainer the type of digital surface
   * container (model of CDigitalSurfaceContainer).
   *
   * @tparam TSeparableMetric a model of CSeparableMetric used for
   * computing the Voronoi map (e.g. Euclidean metric is
   * DGtal::ExactPredicateLpSeparableMetric<TSpace, 2> )
   *
   * @tparam TKernelFunction the type of the kernel function chi_r used
   * for integrating the VCM, a map: Point -> Scalar.
   */
  template <typename TDigitalSurfaceContainer, typename TSeparableMetric, typename TKernelFunction>
  class VCMDigitalSurfaceNormalEstimator
  {
    BOOST_CONCEPT_ASSERT(( CDigitalSurfaceContainer< TDigitalSurfaceContainer > ));
    BOOST_CONCEPT_ASSERT(( CSeparableMetric<TSeparableMetric> ));
    // ----------------------- public types ------------------------------
  public:
    typedef TDigitalSurfaceContainer DigitalSurfaceContainer; //< the chosen container
    typedef TSeparableMetric                          Metric; //< the chosen metric
    typedef TKernelFunction                  KernelFunction;  //< the kernel function
    /// the type of computing the Voronoi covariance measure on a digital surface.
    typedef VoronoiCovarianceMeasureOnDigitalSurface<DigitalSurfaceContainer, Metric, KernelFunction>
    VCMOnSurface;

    // ----------------------- model of CNormalVectorEstimator ----------------
    typedef typename VCMOnSurface::Surface           Surface; //< the digital surface
    typedef typename Surface::SCell                    SCell; //< the signed cell
    typedef typename Surface::ConstIterator    ConstIterator; //< the iterator on surfels
    typedef typename VCMOnSurface::VectorN          Quantity; //< the estimation is a vector
    typedef typename VCMOnSurface::Scalar             Scalar; //< the "real number" type

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~VCMDigitalSurfaceNormalEstimator();

    /**
     * Constructor. The estimator is invalid and \ref init must be called.
     *
     * @param surface the digital surface that is aliased in this. The
     * user can \b secure the aliasing by passing a
     * CountedConstPtrOrConstPtr.
     *
     * @param surfelEmbedding the chosen embedding for surfels.
     */
    VCMDigitalSurfaceNormalEstimator( ConstAlias< Surface > surface,
                                      Surfel2PointEmbedding surfelEmbedding );

    /**
     * Constructor from VoronoiCovarianceMeasureOnDigitalSurface. The
     * estimator is immediately valid.
     *
     * @param aVCMOnSurface any valid Voronoi covariance measure,
     * which is aliased (possibly securely).
     */
    VCMDigitalSurfaceNormalEstimator( ConstAlias< VCMOnSurface > aVCMOnSurface );

    /**
     * Initialisation of estimator parameters.
     *
     * @param[in] h grid size (must be >0).
     *
     * @param[in] R the offset radius for the set of points. Voronoi cells
     * are intersected with this offset. The unit corresponds to a step in the digital space.
     *
     * @param[in] r (an upper bound of) the radius of the support of the
     * kernel function \a chi_r (note \f$\chi_r\f$ in the VCM
     * paper). The unit corresponds to a step in the digital
     * space. This parameter is used for preparing the data structure
     * that answers to proximity queries.
     *
     * @param[in] chi_r the kernel function whose support has radius less
     * or equal to \a r.
     *
     * @param[in] t the radius for the trivial normal estimator, which is
     * used for finding the correct orientation inside/outside for the
     * VCM.
     *
     * @param[in] aMetric an instance of the metric.
     *
     * @param[in] verbose if 'true' displays information on ongoing computation.
     */
    void init( const Scalar h, const Scalar R, const Scalar r, KernelFunction chi_r,
               const Scalar t = 2.5, Metric aMetric = Metric(), bool verbose = true );


    /**
     * @return the estimated quantity at *it
     * @param [in] it the surfel iterator at which we evaluate the quantity.
     */
    template <typename SurfelConstIterator>
    Quantity eval( SurfelConstIterator it ) const;

    /**
     * @return the estimated quantity in the range [itb,ite)
     * @param [in] itb starting surfel iterator.
     * @param [in] ite end surfel iterator.
     * @param [in,out] result resulting output iterator
     *
     */
    template <typename OutputIterator, typename SurfelConstIterator>
    OutputIterator eval( SurfelConstIterator itb,
                         SurfelConstIterator ite,
                         OutputIterator result ) const;

    // ----------------------- Interface --------------------------------------
  public:

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

    // ------------------------- Protected Datas ------------------------------
  protected:
    /// A (possibly) shared object storing the digital surface.
    CountedConstPtrOrConstPtr<Surface> mySurface;
    /// the embedding chosen for the surfels.
    Surfel2PointEmbedding mySurfelEmbedding;
    /// A (possibly) shared object storing the whole Voronoi covariance measure.
    CountedConstPtrOrConstPtr<VCMOnSurface> myVCMOnSurface;
    
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Hidden services ------------------------------
  protected:

  private:

    /**
     * Default constructor.
     * Forbidden by default.
     */
    VCMDigitalSurfaceNormalEstimator ();

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    VCMDigitalSurfaceNormalEstimator ( const VCMDigitalSurfaceNormalEstimator & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    VCMDigitalSurfaceNormalEstimator & operator= ( const VCMDigitalSurfaceNormalEstimator & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class VCMDigitalSurfaceNormalEstimator


  /**
   * Overloads 'operator<<' for displaying objects of class 'VCMDigitalSurfaceNormalEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'VCMDigitalSurfaceNormalEstimator' to write.
   * @return the output stream after the writing.
   */
  template <typename TDigitalSurfaceContainer, typename TSeparableMetric, typename TKernelFunction>
  std::ostream&
  operator<< ( std::ostream & out, 
               const VCMDigitalSurfaceNormalEstimator<TDigitalSurfaceContainer, TSeparableMetric, TKernelFunction> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/VCMDigitalSurfaceNormalEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined VCMDigitalSurfaceNormalEstimator_h

#undef VCMDigitalSurfaceNormalEstimator_RECURSES
#endif // else defined(VCMDigitalSurfaceNormalEstimator_RECURSES)
