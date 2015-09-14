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
* @file CubicalComplex.h
* @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
* Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
*
* @date 2015/08/28
*
* Header file for module CubicalComplex.cpp
*
* This file is part of the DGtal library.
*/

#if defined(CubicalComplex_RECURSES)
#error Recursive header files inclusion detected in CubicalComplex.h
#else // defined(CubicalComplex_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CubicalComplex_RECURSES

#if !defined CubicalComplex_h
/** Prevents repeated inclusion of headers. */
#define CubicalComplex_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <vector>
#include <boost/type_traits.hpp>
#include "DGtal/base/Common.h"
#include "DGtal/base/ConstAlias.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /**
  * Any cell is stored within a cubical complex with an associated
  * data, which must derive from this class. Its basic usage is to
  * store flags associated to the cells, but it may store other
  * values.
  */
  struct CubicalCellData {
    inline CubicalCellData() : data( 0 ) {}
    CubicalCellData( uint32_t d ) : data( d ) {}
    uint32_t data;
  };


  /////////////////////////////////////////////////////////////////////////////
  // template class CubicalComplex
  /**
  * Description of template class 'CubicalComplex' <p> \brief Aim:
  * This class represents an arbitrary cubical complex living in some
  * Khalimsky space. Cubical complexes are sets of cells of different
  * dimensions related together with incidence relations. Two cells
  * in a cubical complex are incident if and only if they are
  * incident in the surrounding Khalimsky space. In other words,
  * cubical complexes are defined here as subsets of Khalimsky spaces. 
  *
  * @tparam TKSpace any model of concepts::CCellularGridSpaceND, i.e. a type
  * that models a Khalimsky space.
  *
  * @tparam TCellContainer any model of associative container, mapping
  * a KSpace::Cell to a CubicalCellData or any type deriving from
  * it. It could be for instance a std::map or a
  * std::unordered_map. Note that unfortunately, unordered_map are
  * (strangely) not models of boost::AssociativeContainer, hence we
  * cannot check concepts here.
  *
  * @tparam TData any type deriving from CubicalCellData that is
  * boost::DefaultConstructible, boost::Assignable,
  * boost::CopyConstructible.
  */
  template < typename TKSpace, 
             typename TCellContainer = std::map< typename TKSpace::Cell, CubicalCellData > >
  class CubicalComplex
  {
    // ----------------------- associated types ------------------------------
  public:
    
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));
    // BOOST_CONCEPT_ASSERT(( boost::AssociativeContainer< TCellContainer > ));
    // BOOST_CONCEPT_ASSERT(( boost::PairAssociativeContainer< TCellContainer > ));

    typedef TKSpace KSpace;
    typedef TCellContainer CellContainer;
    typedef typename CellContainer::mapped_type Data;

    BOOST_STATIC_ASSERT (( boost::is_base_of< CubicalCellData, Data >::value ));
    BOOST_STATIC_ASSERT (( boost::is_same< typename TKSpace::Cell, typename CellContainer::key_type >::value ));

    /// The dimension of the embedding space.
    static const Dimension dimension = KSpace::dimension;
    typedef typename KSpace::Integer     Integer;
    typedef typename KSpace::Cell        Cell;
    typedef typename KSpace::Cells       Cells;
    typedef typename KSpace::Space       Space;
    typedef typename KSpace::Size        Size;
    typedef typename KSpace::Point       Point;
    typedef typename KSpace::DirIterator DirIterator;
    typedef CellContainer                CellMap;
    typedef typename CellMap::const_iterator CellMapConstIterator;
    typedef typename CellMap::iterator   CellMapIterator;

    /// Flag Used to indicate in a cell data that this cell has been (virtually) removed.
    static const uint32_t REMOVED     = 0x10000000;
    /// Flag Used to indicate in a cell data that this cell is collapsible.
    static const uint32_t COLLAPSIBLE = 0x20000000;
    /// Flag Used to indicate in a cell data that this cell is fixed.
    static const uint32_t FIXED       = 0x40000000;
    /// User flag for a cell.
    static const uint32_t USER1       = 0x80000000;
    /// Value for a cell.
    static const uint32_t VALUE       = 0x0fffffff;

    // ----------------------- inner types ------------------------------------
    struct DefaultCellMapIteratorPriority {
      bool operator()( const CellMapIterator& it1, const CellMapIterator& it2 ) const
      {
        uint32_t v1 = it1->second.data & VALUE; 
        uint32_t v2 = it2->second.data & VALUE; 
        return ( v1 < v2 ) 
          || ( ( v1 == v2 ) && ( it1->first < it2->first ) );
      }
    };
    // ----------------------- Standard services ------------------------------
  public:

    /**
    * Destructor.
    */
    ~CubicalComplex();

    /**
    * Constructor of empty complex.
    * @param aK a Khalimsky space.
    */
    CubicalComplex ( ConstAlias<KSpace> aK );

    /**
    * Copy constructor.
    * @param other the object to clone.
    * Forbidden by default.
    */
    CubicalComplex ( const CubicalComplex & other );

    /**
    * Assignment.
    * @param other the object to copy.
    * @return a reference on 'this'.
    * Forbidden by default.
    */
    CubicalComplex & operator= ( const CubicalComplex & other );

    /**
    * Clears the cubical complex, which becomes empty.
    */
    void clear();

    /**
    * Clears all cells of dimension \a d of the cubical complex.
    * @param d the dimension of cell \a aCell.
    */
    void clear( Dimension d );

    /**
    * Fills the data of every cell of this cubical complex, which
    * becomes \a data. Default value resets the data to zero.
    *
    * @param data any data.
    */
    void fillData( Data data = Data() );

    /**
    * Fills the data of every cell of dimension \a d this cubical
    * complex, which becomes \a data. Default value resets the data to
    * zero.
    *
    * @param d the dimension of cell \a aCell.
    * @param data any data.
    */
    void fillData( Dimension d, Data data = Data() );

    /**
    * @return the maximal dimension of a cell in the complex, 0 if
    * the complex is empty.
    */
    Dimension dim() const;

    /**
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return the dimension of the cell \a aCell.
    */
    Dimension dim( const Cell& aCell ) const;

    /**
    * @return a reference to the Khalimsky space associated to this complex.
    */
    const KSpace& space() const;

    /**
    * Insert cell \a aCell into CubicalComplex and assign to it the value \a data.
    *
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @param data any value.
    */
    void insertCell( const Cell& aCell, const Data& data = Data() );

    /**
    * Insert cell \a aCell into CubicalComplex and assign to it the
    * value \a data. Faster than the other insertCell method.
    *
    * @param d the dimension of cell \a aCell.
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @param data any value.
    */
    void insertCell( Dimension d, const Cell& aCell, const Data& data = Data() );

    /**
    * Insert the cells within range [it,itE) into the
    * CubicalComplex. The value associated to each cell is the
    * default.
    *
    * @param it an iterator pointing at the beginning of a range of (arbitrary) cells.
    * @param itE an iterator pointing after the end of a range of (arbitrary) cells.
    */
    template <typename CellConstIterator>
    void insertCells( CellConstIterator it, CellConstIterator itE );

    /**
    * Insert the cells within range [it,itE) into the
    * CubicalComplex. The value associated to each cell is the
    * default.
    *
    * @param d the dimension of all cells in the range [it,itE).
    * @param it an iterator pointing at the beginning of a range of (arbitrary) cells.
    * @param itE an iterator pointing after the end of a range of (arbitrary) cells.
    */
    template <typename CellConstIterator>
    void insertCells( Dimension d, CellConstIterator it, CellConstIterator itE );
    
    /**
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return 'true' if and only if \a aCell belongs to this complex.
    */
    bool belongs( const Cell& aCell ) const;

    /**
    * @param d the dimension of cell \a aCell.
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return 'true' if and only if \a aCell belongs to this complex.
    */
    bool belongs( Dimension d, const Cell& aCell ) const;

    /**
    * Outputs all the cells that are proper faces of \a aCell with output iterator \a it.
    *
    * @param outIt the output iterator on Cell that is used for outputing faces.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    * @param hintClosed when 'true', this hint tells that the complex
    * is closed, so this speeds up this method, otherwise, the
    * complex may be arbitrary.
    *
    * @tparam CellOutputIterator any model of boost::OutputIterator, with value_type Cell.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellOutputIterator>
    void faces( CellOutputIterator& outIt, const Cell& aCell, 
                bool hintClosed = false );

    /**
    * Outputs all the cells that are direct faces of \a aCell with
    * output iterator \a it (direct faces are lower incident cells
    * with a dimension just one below).
    *
    * @param outIt the output iterator on Cell that is used for outputing faces.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    * @param hintClosed when 'true', this hint tells that the complex
    * is closed, so this speeds up this method, otherwise, the
    * complex may be arbitrary.
    *
    * @tparam CellOutputIterator any model of boost::OutputIterator, with value_type Cell.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellOutputIterator>
    void directFaces( CellOutputIterator& outIt, const Cell& aCell,
                      bool hintClosed = false );

    /**
    * Outputs all the iterators on cells that are direct faces of \a aCell with
    * output iterator \a it (direct faces are lower incident cells
    * with a dimension just one below).
    *
    * @param outIt the output iterator on CellMapIterator that is used for outputing face iterators.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    *
    * @tparam CellMapIteratorOutputIterator any model of boost::OutputIterator, with value_type CellMapIterator.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellMapIteratorOutputIterator>
    void directFacesIterators( CellMapIteratorOutputIterator& outIt, const Cell& aCell );

    /**
    * Outputs all the cells that are proper co-faces of \a aCell with
    * output iterator \a it.
    *
    * @param outIt the output iterator on Cell that is used for outputing faces.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    * @param hintOpen when 'true', this hint tells that the complex
    * is open, so this speeds up this method, otherwise, the
    * complex may be arbitrary.
    *
    * @tparam CellOutputIterator any model of boost::OutputIterator, with value_type Cell.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellOutputIterator>
    void coFaces( CellOutputIterator& outIt, const Cell& aCell,
                  bool hintOpen = false );

    /**
    * Outputs all the cells that are direct co-faces of \a aCell with
    * output iterator \a it (direct faces are upper incident cells
    * with a dimension just one above).
    *
    * @param outIt the output iterator on Cell that is used for outputing faces.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    * @param hintOpen when 'true', this hint tells that the complex
    * is open, so this speeds up this method, otherwise, the
    * complex may be arbitrary.
    *
    * @tparam CellOutputIterator any model of boost::OutputIterator, with value_type Cell.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellOutputIterator>
    void directCoFaces( CellOutputIterator& outIt, const Cell& aCell,
                        bool hintOpen = false );

    /**
    * Outputs all the iterators on cells that are direct co-faces of \a aCell with
    * output iterator \a it (direct faces are upper incident cells
    * with a dimension just one above).
    *
    * @param outIt the output iterator on CellMapIterator that is used for outputing face iterators.
    * @param aCell any cell valid in the Khalimsky space associated to the complex. 
    *
    * @tparam CellMapIteratorOutputIterator any model of boost::OutputIterator, with value_type CellMapIterator.
    *
    * @note all returned cells belong to this complex, while it is
    * not compulsory for \a aCell to belong to it.
    */
    template <typename CellMapIteratorOutputIterator>
    void directCoFacesIterators( CellMapIteratorOutputIterator& outIt, const Cell& aCell );

    /**
    * @param d any valid dimension.
    * @return a const iterator pointing on the first cell of dimension \a d of this.
    */
    CellMapConstIterator begin( Dimension d ) const;

    /**
    * @param d any valid dimension.
    * @return a const iterator pointing after the last cell of dimension \a d of this.
    */
    CellMapConstIterator end( Dimension d ) const;

    /**
    * @param d any valid dimension.
    * @return an iterator pointing on the first cell of dimension \a d of this.
    */
    CellMapIterator begin( Dimension d );

    /**
    * @param d any valid dimension.
    * @return an iterator pointing after the last cell of dimension \a d of this.
    */
    CellMapIterator end( Dimension d );

    /**
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return an iterator pointing on the pair (aCell,data) if the cell belongs to the complex, or end( dim( aCell ) ) 
    */
    CellMapConstIterator find( const Cell& aCell ) const;

    /**
    * @param d the dimension of cell \a aCell.
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return an iterator pointing on the pair (aCell,data) if the cell belongs to the complex, or end( d ) 
    */
    CellMapConstIterator find( Dimension d, const Cell& aCell ) const;

    /**
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return an iterator pointing on the pair (aCell,data) if the cell belongs to the complex, or end( dim( aCell ) ) 
    */
    CellMapIterator find( const Cell& aCell );

    /**
    * @param d the dimension of cell \a aCell.
    * @param aCell any cell valid in the Khalimsky space associated to the complex.
    * @return an iterator pointing on the pair (aCell,data) if the cell belongs to the complex, or end( d ) 
    */
    CellMapIterator find( Dimension d, const Cell& aCell );

    /**
    * Close the whole complex.
    */
    void close();

    /**
    * Close all cells of dimension less or equal to \a k.
    * @param k any strictly positive integer.
    */
    void close( Dimension k );


    /**
    * Collapse a subcomplex of this, collapsing cells following
    * priority [priority], in a decreasing sequence until no more
    * collapse is feasible. The range [\a S_itb,\a S_itE) provides the
    * starting cells, generally (but not compulsory) maximal cells.
    *
    * @note Cells whose data has been marked as FIXED are not removed.
    *
    * @note Only cells that are in the closure of [S] may be removed, and
    * only if they are not marked as FIXED.
    *
    * @advanced If you use a DefaultCellMapIteratorPriority object as
    * \a priority, then the VALUE part of each cell data defines the
    * priority (the highest value the soonest are these cells
    * collapsed). You may thus fill these cell values before calling
    * this method.
    *
    * @tparam CellConstIterator any forward const iterator on Cell.
    *
    * @tparam CellMapIteratorPriority any type defining a method 'bool
    * operator()( const Cell&, const Cell&) const'. Defines the order
    * in which cells are collapsed. @see DefaultCellMapIteratorPriority
    *
    * @param S_itb the start of a range of cells which is included in [K].
    * @param S_itE the end of a range of cells which is included in [K].
    * @param priority the object that assign a priority to each cell.
    * @param hintIsSclosed indicates if [\a S_itb,\a S_ite) is a closed set (faster in this case).
    * @param hintIsKclosed indicates that this complex is closed.
    * @param verbose outputs some information during processing when 'true'.
    */
    template <typename CellIterator, typename CellMapIteratorPriority>
    void collapse( CellIterator S_itB, CellIterator S_itE, 
                   const CellMapIteratorPriority& priority, 
                   bool hintIsSClosed = false, bool hintIsKClosed = false,
                   bool verbose = false );


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

    /// The Khalimsky space in which lives the cubical complex.
    const KSpace* myKSpace;

    // ------------------------- Private Datas --------------------------------
  private:

    /// An array of map Cell -> Data that stores cells dimension per
    /// dimension (i.e. cells of dimension 0 are stored in myCells[0],
    /// cells of dimension 1 in myCells[1] and so on).
    std::vector<CellMap> myCells;
    

    // ------------------------- Hidden services ------------------------------
  protected:

    /**
    * Constructor.
    * Forbidden by default (protected to avoid g++ warnings).
    */
    CubicalComplex();

  private:


    // ------------------------- Internals ------------------------------------
  private:

    /**
    * Given a cell [c], tells if it is a maximal cell in the complex
    * (return 0), or if it is a free face of the cell pointed by
    * [it_cell_up] (return 1) or if it is not a free face.
    *
    * The complex must be closed. In computing the 1-up-incident
    * cells, this method ignores cell marked as REMOVED. Furthermore,
    * if one 1-up-incident cell is not marked as COLLAPSIBLE, the
    * method returns 2.
    *
    * @param c a cubical cell (belonging to 'this')
    *
    * @param it_cell_up (returns) a pointer on a cell d if c is a
    * free face of d.
    *
    * @return 0 if the cell is maximal, 1 if the cell is a free face,
    * 2 otherwise.
    */
    uint32_t computeCellType( Dimension n, const Cell& c, CellMapIterator& it_cell_up );

  }; // end of class CubicalComplex


  /**
  * Overloads 'operator<<' for displaying objects of class 'CubicalComplex'.
  * @param out the output stream where the object is written.
  * @param object the object of class 'CubicalComplex' to write.
  * @return the output stream after the writing.
  */
  template <typename TKSpace, typename TCellContainer>
  std::ostream&
  operator<< ( std::ostream & out, 
               const CubicalComplex<TKSpace, TCellContainer> & object );

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/CubicalComplex.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CubicalComplex_h

#undef CubicalComplex_RECURSES
#endif // else defined(CubicalComplex_RECURSES)
