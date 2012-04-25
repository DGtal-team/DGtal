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
 * @file DigitalSurfaceEmbedderWithNormalVectorEstimator.h
 * @author David Coeurjolly (\c david.coeurjolly@liris.cnrs.fr )
 * Laboratoire d'InfoRmatique en Image et Systèmes d'information - LIRIS (CNRS, UMR 5205), CNRS, France
 *
 * @date 2012/02/14
 *
 * Header file for module DigitalSurfaceEmbedderWithNormalVectorEstimator.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(DigitalSurfaceEmbedderWithNormalVectorEstimator_RECURSES)
#error Recursive header files inclusion detected in DigitalSurfaceEmbedderWithNormalVectorEstimator.h
#else // defined(DigitalSurfaceEmbedderWithNormalVectorEstimator_RECURSES)
/** Prevents recursive inclusion of headers. */
#define DigitalSurfaceEmbedderWithNormalVectorEstimator_RECURSES

#if !defined DigitalSurfaceEmbedderWithNormalVectorEstimator_h
/** Prevents repeated inclusion of headers. */
#define DigitalSurfaceEmbedderWithNormalVectorEstimator_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/kernel/NumberTraits.h"
#include "DGtal/topology/CDigitalSurfaceEmbedder.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  // Forward declaration.
  template < typename TDigitalSurfaceEmbedder,
             typename TNormalVectorEstimator >
  struct DigitalSurfaceEmbedderWithNormalVectorEstimatorGradientMap;

  /////////////////////////////////////////////////////////////////////////////
  /**
    Description of template class
    'DigitalSurfaceEmbedderWithNormalVectorEstimator' <p> \brief Aim:
    Combines a digital surface embedder with a normal vector estimator
    to get a model of CDigitalSurfaceEmbedder and CWithGradientMap.
    (also default constructible, copy constructible, assignable).
   
    @tparam TDigitalSurfaceEmbedder a model of digital surface embedder.
    @tparam TNormalVectorEstimator the type of normal vector estimator.

    @todo assert dimensions of space, embedder and implicit function.
   */
  
  template < typename TDigitalSurfaceEmbedder,
             typename TNormalVectorEstimator >
  class DigitalSurfaceEmbedderWithNormalVectorEstimator
  {
  public:
    typedef DigitalSurfaceEmbedderWithNormalVectorEstimator
    < TDigitalSurfaceEmbedder, TNormalVectorEstimator > Self;
    BOOST_CONCEPT_ASSERT(( CDigitalSurfaceEmbedder<TDigitalSurfaceEmbedder> ));

    typedef TDigitalSurfaceEmbedder DigitalSurfaceEmbedder;
    typedef TNormalVectorEstimator NormalVectorEstimator;
    
    typedef typename DigitalSurfaceEmbedder::KSpace KSpace;
    typedef typename DigitalSurfaceEmbedder::Surface Surface;
    typedef typename DigitalSurfaceEmbedder::SCell SCell;
    typedef typename DigitalSurfaceEmbedder::RealPoint RealPoint;
    typedef typename DigitalSurfaceEmbedder::Argument Argument;
    typedef typename DigitalSurfaceEmbedder::Value Value;

    typedef typename KSpace::Space Space;
    typedef typename Space::RealVector RealVector;
    typedef typename NormalVectorEstimator::Quantity Quantity;
    typedef typename NormalVectorEstimator::DigitalSurface NVESurface;

    BOOST_STATIC_ASSERT(( ConceptUtils::SameType< RealVector, Quantity >::value ));
    BOOST_STATIC_ASSERT(( ConceptUtils::SameType< Surface, NVESurface >::value ));

    typedef DigitalSurfaceEmbedderWithNormalVectorEstimatorGradientMap<DigitalSurfaceEmbedder,NormalVectorEstimator> GradientMap;

    /** 
        Constructor.
        
       @param aDSEmbedder any digital surface embedder.
       @param anEstimator a normal vector estimator
    */
    DigitalSurfaceEmbedderWithNormalVectorEstimator
    ( const DigitalSurfaceEmbedder & aDSEmbedder,
      const NormalVectorEstimator & anEstimator );

    /** 
       Copy Constructor.
       @param other the object to clone.
    */
    DigitalSurfaceEmbedderWithNormalVectorEstimator
    ( const Self & other );

    /** 
     * Destructor.
     */    
    ~DigitalSurfaceEmbedderWithNormalVectorEstimator();

    // ----------------------- Interface --------------------------------------
  public:

    /**
       Maps a signed cell to its corresponding point in the Euclidean
       space. Uses the given embedder.
       
       @param scell any signed cell in the cellular grid space.
       @return its embedding in the Euclidean space.
     */
    RealPoint operator()( const SCell & scell ) const;

    /**
       @return the associated digital surface.
     */
    const Surface & surface() const;

    /**
       @return the gradient map associated to this embedder, i.e. a
       functor SCell -> RealVector. Uses the given NormalVectorEstimator.
    */
    GradientMap gradientMap() const;

    /**
       @param scell any signed cell in the cellular grid space.
       @return the gradient vector at this surfel. Uses the given
       NormalVectorEstimator.
     */
    RealVector gradient( const SCell & scell ) const;
   
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
  private:
    // ------------------------- Private Datas --------------------------------
  private:
   
    ///A pointer on the digital surface
    const DigitalSurfaceEmbedder& myDSEmbedder;
    /// A pointer on the normal vector estimator.
    const NormalVectorEstimator& myEstimator;
   
    // ------------------------- Hidden services ------------------------------
  protected:
    DigitalSurfaceEmbedderWithNormalVectorEstimator();

  private:    
    /** 
       Assignment.
       Forbidden.
       @param other the object to clone.
       @return a reference to 'this'.
    */
    Self & operator=( const Self & other );
    
    
  }; // end of class DigitalSurfaceEmbedderWithNormalVectorEstimator

  template < typename TDigitalSurfaceEmbedder,
             typename TNormalVectorEstimator >
  struct DigitalSurfaceEmbedderWithNormalVectorEstimatorGradientMap
  {
  };

  /**
   * Overloads 'operator<<' for displaying objects of class 'DigitalSurfaceEmbedderWithNormalVectorEstimator'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'DigitalSurfaceEmbedderWithNormalVectorEstimator' to write.
   * @return the output stream after the writing.
   */
  template < typename TDigitalSurfaceEmbedder, typename TNormalVectorEstimator >
  std::ostream&
  operator<< ( std::ostream & out, 
               const DigitalSurfaceEmbedderWithNormalVectorEstimator<TDigitalSurfaceEmbedder, TNormalVectorEstimator> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/geometry/surfaces/estimation/DigitalSurfaceEmbedderWithNormalVectorEstimator.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined DigitalSurfaceEmbedderWithNormalVectorEstimator_h

#undef DigitalSurfaceEmbedderWithNormalVectorEstimator_RECURSES
#endif // else defined(DigitalSurfaceEmbedderWithNormalVectorEstimator_RECURSES)
