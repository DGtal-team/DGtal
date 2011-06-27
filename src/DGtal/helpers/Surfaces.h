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
#include "DGtal/base/Exceptions.h"
#include "DGtal/topology/SurfelAdjacency.h"
#include "DGtal/topology/SurfelNeighborhood.h"

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

     Essentially a backport from <a
     href="http://gforge.liris.cnrs.fr/projects/imagene">ImaGene</a>.
   */
  template <typename TKSpace>
  class Surfaces
  {
    // ----------------------- Types ------------------------------
  public:
    typedef TKSpace KSpace;
    typedef typename KSpace::Integer Integer;
    typedef typename KSpace::Point Point;
    typedef typename KSpace::Cell Cell;
    typedef typename KSpace::SCell SCell;
    typedef typename KSpace::DirIterator DirIterator;

    // ----------------------- Static services ------------------------------
  public:

    /**
       Find a bel in some digital set by random tries then dichotomy.

       @tparam DigitalSet a model of a digital set (e.g., std::set<Point>)..
       @param K any cellular grid space.
       @param dset any digital set which should be at least partially included in the bounds of space [K].
       @param nbries the maximum number of random tries (default 1000).

       @return a signed surfel separating a digital point in [dset]
       from a face adjacent digital point outside [dset] or throws an
       InputException if none was found after [nbtries] iterations.
    */
    template <typename DigitalSet>
    static
    SCell findABel( const KSpace & K,
		    const DigitalSet & dset,
		    unsigned int nbtries = 1000 ) throw (DGtal::InputException);

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
       Creates a vector of signed surfels whose elements represents a
       boundary component of the digital set [shape]. The algorithms
       tracks surfels along the boundary of the shape by starting from
       the given start_surfel.
       
       @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
       
       @param aSCellContour2D (modified) a vector  of cells (which are all surfels),
       containing the ordered list of the boundary component of [spelset] which touches [start_surfel].
       
       @param K any space.
       @param surfel_adj the surfel adjacency chosen for the tracking.
       @param shape any digital set.
       @param start_surfel a signed surfel which should be between an
       element of [shape] and an element not in [shape].
  */
    template <typename DigitalSet >
    static 
    void track2DBoundary( std::vector<SCell> & aSCellContour2D,
			  const KSpace & K,
			  const SurfelAdjacency<KSpace::dimension> & surfel_adj,
			  const DigitalSet & shape,
			  const SCell & start_surfel );



    
  /**
       Creates a vector of signed surfels whose elements represents a
       boundary component of the digital set [shape]. The algorithms
       tracks surfels along the boundary of the shape by starting from
       the given start_surfel.
       
       @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
       
       @param aSCellContour2D (modified) a vector  of cells (which are all surfels),
       containing the ordered list of the boundary component of [spelset] which touches [start_surfel].
       
       @param K any space.
       @param surfel_adj the surfel adjacency chosen for the tracking.
       @param shape any digital set.
       @param start_surfel a signed surfel which should be between an
       element of [shape] and an element not in [shape].
  */
    template <typename DigitalSet >
    static 
    void track2DBoundary( std::vector<SCell> & aSCellContour2D,
			  const KSpace & K, const Dimension & dimDir,
			  const SurfelAdjacency<KSpace::dimension> & surfel_adj,
			  const DigitalSet & shape,
			  const SCell & start_surfel );



  /**
     Extract all 4-connected contours as a vector containing the set of contour
     Points.  Each contour is represented by a vector of points
     defined by the sequence of pointels extracted from the boundary
     surfels.
       
     @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
     
     @param aSCellContour2D (modified) a vector of contour represented
     by a vector of cells (which are all surfels), containing the
     ordered list of the boundary component of [spelset].
     
     @param aKSpace any space.
     @param aSurfelAdj the surfel adjacency chosen for the tracking.
     @param aShape any digital set.
  */
    template <typename DigitalSet >
    static 
    void extractAllPointContours4C( std::vector< std::vector< Point > > & aVectPointContour2D,
				    const KSpace & aKSpace,
				    const DigitalSet & aShape, const SurfelAdjacency<2> &aSAdj );

    

    /**
     Extract all contours as a vector containing the set of contours
     composed of SCell.  Each contour is represented by a vector of
     signed surfels whose elements represents a boundary component of
     the digital set [shape]. The algorithms tracks surfels along the
     boundary of the shape.
       
     @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
     
     @param aSCellContour2D (modified) a vector of contour represented
     by a vector of cells (which are all surfels), containing the
     ordered list of the boundary component of [spelset].
     
     @param aKSpace any space.
     @param aSurfelAdj the surfel adjacency chosen for the tracking.
     @param aShape any digital set.
  */
    template <typename DigitalSet >
    static 
    void extractAll2DSCellContours( std::vector< std::vector<SCell> > & aVectSCellContour2D,
				    const KSpace & aKSpace,
				    const SurfelAdjacency<KSpace::dimension> & aSurfelAdj,
				    const DigitalSet & aShape );
    
    

    /**
     * Extract all surfel elements associated to each connected
     * components of the given DigitalSet. The connected surfel set
     * are given as result in a vector containing all components.
     *
     @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
     
     @param aVectConnectedSCell (modified) a vector containing for
     each connected components a vector of the set of connected SCells.
     @param aKSpace any space.
     @param aSurfelAdj the surfel adjacency chosen for the tracking.
     @param aShape any digital set.
     
     *
     */
    
    template <typename DigitalSet >
    static 
    void extractAllConnectedSCell( std::vector< std::vector<SCell> > & aVectConnectedSCell,
				   const KSpace & aKSpace,
				   const SurfelAdjacency<KSpace::dimension> & aSurfelAdj,
				   const DigitalSet & aShape );
    
    



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

  /**
     Creates a set of surfels whose elements represents all
     the boundary components of the digital set [shape]. 
     
     @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
     @tparam DigitalSet a model of a digital set (e.g., std::set<Point>)..
     
     @param aBoundary (modified) a set of cells (which are all surfels),
     the boundary component of [aSpelSet].
     
     @param aKSpace any space.
     @param aSurfelAdj the surfel adjacency chosen for the tracking.
     @param aSpelSet any digital set.
     @param aLowerBound and @param aUpperBound Cell giving the bounds of the extracted boundary.
  */
    template <typename SCellSet, typename DigitalSet >
    static 
    void uMakeBoundary( SCellSet & aBoundary,
			const KSpace & aKSpace,
			const DigitalSet & aSpelSet,
			const Cell aLowerBound, const Cell aUpperBound  );
    
 /**
     Creates a set of signed surfels whose elements represents all
     the boundary components of the digital set [shape]. 
     
     @tparam SCellSet a model of a set of SCell (e.g., std::set<SCell>).
     @tparam DigitalSet a model of a digital set (e.g., std::set<Point>)..
     
     @param aBoundary (modified) a set of cells (which are all surfels),
     the boundary component of [aSpelSet].
     
     @param aKSpace any space.
     @param aSurfelAdj the surfel adjacency chosen for the tracking.
     @param aSpelSet any digital set.
     @param aLowerBound and @param aUpperBound Cell giving the bounds of the extracted boundary.
  */
    template <typename SCellSet, typename DigitalSet >
    static 
    void sMakeBoundary( SCellSet & aBoundary,
			const KSpace & aKSpace,
			const DigitalSet & aSpelSet,
			const Cell aLowerBound, const Cell aUpperBound  );
    

    


    

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
