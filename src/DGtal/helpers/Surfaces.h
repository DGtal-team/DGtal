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
 * @file Surfaces.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5807), University of Savoie, France
 *
 * @date 2011/03/19
 *
 * Header file for module Surfaces.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(Surfaces_RECURSES)
#error Recursive header files inclusion detected in Surfaces.h
#else // defined(Surfaces_RECURSES)
/** Prevents recursive inclusion of headers. */
#define Surfaces_RECURSES

#if !defined Surfaces_h
/** Prevents repeated inclusion of headers. */
#define Surfaces_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class Surfaces
  /**
     Description of template class 'Surfaces' <p> \brief Aim: A
     utility class for constructing surfaces (i.e. set of
     (n-1)-cells).
     
     @tparam TKSpace the type of cellular grid space (e.g. a
     KhalimskySpaceND).
   */
  template <typename TKSpace>
  class Surfaces
  {
    // ----------------------- Types ------------------------------
  public:
    typedef TKSpace KSpace;
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Cell Cell;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::DirIterator DirIterator;

    // ----------------------- Static services ------------------------------
  public:

    /**
       Creates a set of signed surfels whose elements represents a
       boundary component of the digital set [shape]. The algorithms
       tracks surfels along the boundary of the shape.
       
       @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
       @tparam DigitalSet a model of a digital set (e.g., std::set<Point>)..
       
       @param surface (modified) a set of cells (which are all surfels),
       the boundary component of [spelset] which touches [start_surfel].
       
       @param K any space.
       @param surfel_adj the surfel adjacency chosen for the tracking.
       @param shape any digital set.
       @param start_surfel a signed surfel which should be between an
       element of [shape] and an element not in [shape].
    */
    template <typename SCellSet, typename DigitalSet >
    static 
    void trackBoundary( SCellSet & surface,
			 const KSpace & K,
			 const SurfelAdjacency<KSpace::dimension> & surfel_adj,
			 const DigitalSet & shape,
			 const SCell & start_surfel );

    /**
       Creates a set of signed surfels whose elements represents a
       boundary component of the digital set [shape]. The algorithms
       tracks surfels along the boundary of the shape. It follows only
       direct orientations, so that it is faster than trackBoundary
       but requires the object to be fully inside the space. Follows
       the idea of Artzy, Frieder and Herman algorithm
       [Artzy:1981-cgip], but in nD.
       
       @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
       @tparam DigitalSet a model of a digital set (e.g., std::set<Point>)..
       
       @param surface (modified) a set of cells (which are all surfels),
       the boundary component of [spelset] which touches [start_surfel].
       
       @param K any space.
       @param surfel_adj the surfel adjacency chosen for the tracking.
       @param shape any digital set.
       @param start_surfel a signed surfel which should be between an
       element of [shape] and an element not in [shape].
    */
    template <typename SCellSet, typename DigitalSet >
    static 
    void trackClosedBoundary( SCellSet & surface,
			      const KSpace & K,
			      const SurfelAdjacency<KSpace::dimension> & surfel_adj,
			      const DigitalSet & shape,
			      const SCell & start_surfel );

    // ----------------------- Standard services ------------------------------
  public:

    /**
     * Destructor.
     */
    ~Surfaces();

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

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
     * Constructor.
     * Forbidden by default (protected to avoid g++ warnings).
     */
    Surfaces();

  private:

    /**
     * Copy constructor.
     * @param other the object to clone.
     * Forbidden by default.
     */
    Surfaces ( const Surfaces & other );

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     * Forbidden by default.
     */
    Surfaces & operator= ( const Surfaces & other );

    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class Surfaces


  /**
   * Overloads 'operator<<' for displaying objects of class 'Surfaces'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'Surfaces' to write.
   * @return the output stream after the writing.
   */
  template <typename TKSpace>
  std::ostream&
  operator<< ( std::ostream & out, const Surfaces<TKSpace> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/helpers/Surfaces.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined Surfaces_h

#undef Surfaces_RECURSES
#endif // else defined(Surfaces_RECURSES)
