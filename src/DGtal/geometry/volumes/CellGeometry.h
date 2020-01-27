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
 * @file CellGeometry.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2020/01/27
 *
 * Header file for module CellGeometry.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(CellGeometry_RECURSES)
#error Recursive header files inclusion detected in CellGeometry.h
#else // defined(CellGeometry_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CellGeometry_RECURSES

#if !defined CellGeometry_h
/** Prevents repeated inclusion of headers. */
#define CellGeometry_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class CellGeometry
  /**
     Description of template class 'CellGeometry' <p> \brief Aim:
     Computes and stores sets of cells and provides methods to compute
     intersections with cells.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable. 

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
   */
  template < typename TKSpace >
  class CellGeometry 
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TSpace > ));

  public:
    typedef CellGeometry<TKSpace>           Self;
    typedef TKSpace                         KSpace;
    typedef typename KSpace::Integer        Integer;
    typedef typename KSpace::Point          Point;
    typedef typename KSpace::Vector         Vector;
#ifdef WITH_BIGINTEGER
    typedef DGtal::BigInteger               BigInteger;
#else
    typedef DGtal::int64_t                  BigInteger;
#endif
    static const Dimension dimension = Space::dimension;


    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /**
     * Destructor.
     */
    ~CellGeometry() = default;

    /**
     * Constructor. 
     */
    CellGeometry();

    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    CellGeometry ( const Self & other ) = default;

    /**
     * Constructor from cellular space.
     * @param K any cellular grid space.
     * @param max_cell_dim the maximal cell dimension that is used for processing (K::dimension - 1 is sufficient to check convexity)
     * @param verbose tells if verbose mode.
     */
    CellGeometry ( const KSpace & K, Dimension max_cell_dim, bool verbose = false );
    
    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// Initialization from cellular space. If there was cells stored
    /// in this object, they are removed.
    ///
    /// @param K any cellular grid space.
    /// @param max_cell_dim the maximal cell dimension that is used for processing (K::dimension - 1 is sufficient to check convexity)
    /// @param verbose tells if verbose mode.
    void init( const KSpace & K, Dimension max_cell_dim, bool verbose = false );

    /// Initializes the cell cover from a range of digital points [itB, itE).
    template <typename PointIterator>
    void setPoints( PointIterator itB, PointIterator itE );
    
    /// Initializes the cell cover from a range of digital pointels [itB, itE).
    template <typename PointelIterator>
    void setPointels( PointelIterator itB, PointelIterator itE );
    
    /// @}

    // ----------------------- Accessor services ------------------------------
  public:
    /// @name Accessor services
    /// @{
    /// @}
    
    // ----------------------- Static helper services ------------------------------
  public:
    /// @name Static helper services
    /// @{
    
    /// @}
    
    
    // ----------------------- Interface --------------------------------------
  public:
    /// @name Interface services
    /// @{

    /**
     * Writes/Displays the object on an output stream.
     * @param out the output stream where the object is written.
     */
    void selfDisplay ( std::ostream & out ) const;

    /**
     * Checks the validity/consistency of the object. If the polytope
     * has been default constructed, it is invalid.
     *
     * @return 'true' if the object is valid, 'false' otherwise.
     */
    bool isValid() const;

    /**
     * @return the class name. It is notably used for drawing this object.
     */
    std::string className() const;

    /// @}

    // ------------------------- Protected Datas ------------------------------
  protected:

    /// The cellular space for cells.
    KSpace myK;
    /// The cubical complex that stores cells.
    CubicalComplex myCC;
    /// The maximal cell dimension
    Dimension myMaxCellDim;
    /// Tells if verbose mode.
    bool myVerbose;
      
    // ------------------------- Private Datas --------------------------------
  private:


    // ------------------------- Internals ------------------------------------
  private:

  }; // end of class CellGeometry

  /// @name Functions related to CellGeometry (output)
  /// @{
  
  /**
   * Overloads 'operator<<' for displaying objects of class 'CellGeometry'.
   * @param out the output stream where the object is written.
   * @param object the object of class 'CellGeometry' to write.
   * @return the output stream after the writing.
   */
  template <typename TSpace>
  std::ostream&
  operator<< ( std::ostream & out, 
               const CellGeometry<TSpace> & object );

  /// @}

  /// @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
  template <typename TKSpace, int i, int N = TKSpace::dimension>
  struct CellGeometryFunctions
  {
    /// @tparam PointelIterator any model of forward iterator on pointels.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of pointels.
    /// @param itE past the end of a range of pointels.
    /// @return the incident i-cells to the given range of pointels [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointelIterator>
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPointels( const KSpace& K,
				PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
	{
	  auto pointel = *it;
	  auto faces   = K.uFaces( pointel );
	  for ( auto&& f : faces )
	    if ( K.uDim( f ) == i ) cells.insert( f );
	}
    }

    /// @tparam PointIterator any model of forward iterator on points.
    /// @param K a valid cellular grid space large enough to hold the cells.
    /// @param itB the start of a range of points.
    /// @param itE past the end of a range of points.
    /// @return the incident i-cells to the given range of points [itB, itE).
    /// @note General method. Note as efficient as specializations.
    template <typename PointelIterator>
    std::unordered_set<typename KSpace::Cell>
    getIncidentCellsToPoints( const KSpace& K,
			      PointelIterator itB, PointelIterator itE )
    {
      std::unordered_set<typename KSpace::Cell> cells;
      for ( auto it = itB; it != itE; ++it )
	{
	  auto pointel = K.uPointel( *it );
	  auto faces   = K.uFaces( pointel );
	  for ( auto&& f : faces )
	    if ( K.uDim( f ) == i ) cells.insert( f );
	}
    }
  }; // end struct CellGeometryFunctions

  
//     TouchedCells1( const KSpace& K, PointelIterator itB, PointelIterator itE )
// {
//   //typename KSpace::CellSet cells1;
//   std::unordered_set<typename KSpace::Cell> cells1;
//   for ( auto it = itB; it != itE; ++it )
//     {
//       auto pointel = *it;
//       cells1.insert( K.uIncident( pointel, 0, true ) );
//       cells1.insert( K.uIncident( pointel, 0, false ) );
//       cells1.insert( K.uIncident( pointel, 1, true ) );
//       cells1.insert( K.uIncident( pointel, 1, false ) );
//       cells1.insert( K.uIncident( pointel, 2, true ) );
//       cells1.insert( K.uIncident( pointel, 2, false ) );
//     }
//   return cells1;
// }
    
//   };
  
} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "CellGeometry.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CellGeometry_h

#undef CellGeometry_RECURSES
#endif // else defined(CellGeometry_RECURSES)
