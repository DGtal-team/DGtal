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
 * @file LocalEstimatorFromFunctorAdapter.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systemes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2013/05/28
 *
 * Header file for module LocalEstimatorFromFunctorAdapter.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(LocalEstimatorFromFunctorAdapter_RECURSES)
#error Recursive header files inclusion detected in LocalEstimatorFromFunctorAdapter.h
#else // defined(LocalEstimatorFromFunctorAdapter_RECURSES)
/** Prevents recursive inclusion of headers. */
#define LocalEstimatorFromFunctorAdapter_RECURSES

#if !defined LocalEstimatorFromFunctorAdapter_h
/** Prevents repeated inclusion of headers. */
#define LocalEstimatorFromFunctorAdapter_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/topology/CanonicSCellEmbedder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class LocalEstimatorFromFunctorAdapter
  /**
   * Description of template class 'LocalEstimatorFromFunctorAdapter' <p>
   * \brief Aim: this class adapts any functor on digital surface element to define
   * a local estimator.
   *
   *  @tparam TDigitalSurface any model of digital surface concept (CDigitalSurface)
   *  @tparam TFunctorOnSurfel a functor on TDigitalSurface surfel
   */
  template <typename TDigitalSurface, typename TMetric, typename TFunctorOnSurfel>
  class LocalEstimatorFromFunctorAdapter
  {
    // ----------------------- Standard services ------------------------------
  public:

    ///Digital surface type
    typedef TDigitalSurface DigitalSurface;
    
    ///Metric type
    typedef TMetric Metric;
    
    ///Functor on surfels type
    typedef TFunctorOnSurfel FunctorOnSurfel;
        
    ///Quantity type
    typedef typename TFunctorOnSurfel::Value Quantity;
    
    ///Surfel const iterator
    typedef typename DigitalSurface::ConstIterator ConstIterator;
    
    
  private:
    
    ///Embedder
    typedef CanonicalSCellEmbedder<typename DigitalSurface::KSpace> Embedder;
    typedef Embedder::Value RealPoint;
    typedef RealPoint::Coordinate Scalar;
    typedef std::binder1st<Embedder,
    
    /**
     * Constructor.
     * @param aSurface a digital surface
     * @param aFunctor a functor on digital surface elements.
     */
    LocalEstimatorFromFunctorAdapter(ConstAlias<DigitalSurface>  aSurface,
                                     ConstAlias<TMetric> aMetric,
                                     ConstAlias<FunctorOnSurfel>  aFunctor);
    
    /**
     * Destructor.
     */
    ~LocalEstimatorFromFunctorAdapter();

    // ----------------------- Interface --------------------------------------
  public:

    
    /**
     * Initialisation.
     * @param h grid size (must be >0).
     * the convolution.
     */
    void init(const double h);
    
  
    /**
     * @return the estimated quantity at *it
     */
    Quantity eval(const ConstIterator& it) const;
    
    /**
     * @return the estimated quantity
     * from itb till ite (exculded)
     */
    template <typename OutputIterator>
    OutputIterator eval(const ConstIterator& itb,
                        const ConstIterator& ite,
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
    LocalEstimatorFromFunctorAdapter();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    LocalEstimatorFromFunctorAdapter ( const LocalEstimatorFromFunctorAdapter & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    LocalEstimatorFromFunctorAdapter & operator= ( const LocalEstimatorFromFunctorAdapter & other );

    // ------------------------- Internals ------------------------------------
  private:

    ///Digital surface member
    const DigitalSurface * mySurface;

    ///Functor member
    const FunctorOnSurfel * myFunctor;
    
    ///Distance functor
    const Metric * myMetric;
    
    ///Grid step
    double myH;
    
    ///Has init been done before eval
    bool myInit;
    
  }; // end of class LocalEstimatorFromFunctorAdapter


  /**
   * Overloads 'operator<<' for displaying objects of class 'LocalEstimatorFromFunctorAdapter'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'LocalEstimatorFromFunctorAdapter' to write.
   * @return the output stream after the writing.
   */
  template <typename TD, typename TV, typename TF>
  std::ostream&
  operator<< ( std::ostream & out, const LocalEstimatorFromFunctorAdapter<TD,TV,TF> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation//LocalEstimatorFromFunctorAdapter.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined LocalEstimatorFromFunctorAdapter_h

#undef LocalEstimatorFromFunctorAdapter_RECURSES
#endif // else defined(LocalEstimatorFromFunctorAdapter_RECURSES)
