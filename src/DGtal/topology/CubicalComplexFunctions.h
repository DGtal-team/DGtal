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
 * @file CubicalComplexFunctions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/11/24
 *
 * Defines functions associated to CubicalComplex.
 *
 * This file is part of the DGtal library.
 */

#if defined(CubicalComplexFunctions_RECURSES)
#error Recursive header files inclusion detected in CubicalComplexFunctions.h
#else // defined(CubicalComplexFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define CubicalComplexFunctions_RECURSES

#if !defined CubicalComplexFunctions_h
/** Prevents repeated inclusion of headers. */
#define CubicalComplexFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/topology/CubicalComplex.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace functions {
    /// CubicalComplex operators.
    namespace ccops {

      /** 
       * Cubical Complex close operation. 
       * @param[in] S1 an input cubical complex
       * @return a new cubical complex that is the closing of S1.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator~( const CubicalComplex< TKSpace, TCellContainer >& S1 )
      {
        CubicalComplex< TKSpace, TCellContainer > S( S1 );
        S.close();
        return S;
      }

      /** 
       * Cubical Complex open operation. 
       * @param[in] S1 an input cubical complex
       * @return a new cubical complex that is the opening of S1.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator*( const CubicalComplex< TKSpace, TCellContainer >& S1 )
      {
        CubicalComplex< TKSpace, TCellContainer > S( S1 );
        S.open();
        return S;
      }

      /** 
       * Cubical Complex difference operation. Updates the cubical complex S1 as S1 - S2. 
       * @param[in,out] S1 an input cubical complex, \a S1 - \a S2 as output.
       * @param[in] S2 another input cubical complex.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer >& 
      operator-=( CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator-=( S1.myCells[ i ],S2.myCells[ i ] );
        return S1;
      }
      
      /** 
       * Cubical Complex difference operation. Returns the difference of \a S1 - \a S2.
       * @param[in] S1 an input cubical complex
       * @param[in] S2 another input cubical complex.
       *
       * @return the cubical complex \a S1 - \a S2. 
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator-( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                 const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        CC S( S1 );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator-=( S.myCells[ i ],S2.myCells[ i ] );
        return S;
      }

      /** 
       * Cubical Complex union operation. Returns the cubical complex \f$ S1 \cup S2 \f$.
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       * @return the cubical complex \f$ S1 \cup S2 \f$.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator|( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                 const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        CC S( S1 );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator|=( S.myCells[ i ],S2.myCells[ i ] );
        return S;
      }

      /** 
       * Cubical Complex union operation. Updates the cubical complex \a S1 as \f$ S1 \cup S2 \f$.
       * @param[in,out] S1 an input cubical complex, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input cubical complex.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer >& 
      operator|=( CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator|=( S1.myCells[ i ], S2.myCells[ i ] );
        return S1;
      }

      /** 
       * Cubical Complex intersection operation. Returns the cubical complex \f$ S1 \cap S2 \f$.
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       * @return the cubical complex \f$ S1 \cap S2 \f$.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator&( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                 const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        CC S( S1 );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator&=( S.myCells[ i ], S2.myCells[ i ] );
        return S;
      }

      /** 
       * Cubical Complex intersection operation. Updates the cubical complex \a S1 as \f$ S1 \cap S2 \f$.
       * @param[in,out] S1 an input cubical complex, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input cubical complex.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer >& 
      operator&=( CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator&=( S1.myCells[ i ], S2.myCells[ i ] );
        return S1;
      }

      /** 
       * Cubical Complex symmetric difference operation. Returns the cubical complex \f$ S1 \Delta S2 \f$.
       *
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       * @return the cubical complex \f$ S1 \Delta S2 \f$.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer > 
      operator^( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                 const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        CC S( S1 );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator^=( S.myCells[ i ], S2.myCells[ i ] );
        return S;
      }

      /** 
       * Cubical Complex symmetric difference operation. Updates the
       * cubical complex \a S1 as \f$ S1 \Delta S2 \f$.
       *
       * @param[in,out] S1 an input cubical complex, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input cubical complex.
       */
      template <typename TKSpace, typename TCellContainer>
      inline CubicalComplex< TKSpace, TCellContainer >& 
      operator^=( CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          setops::operator^=( S1.myCells[ i ], S2.myCells[ i ] );
        return S1;
      }

      //////////////////////// EQUALITY /////////////////////////
      /** 
       * Equality test.
       *
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       *
       * @return true iff \a S1 is equal to \a S2 (i.e. \a S1 is a
       * subcomplex of \a S2 and \a S2 is a subcomplex of \a S1).
       */
      template <typename TKSpace, typename TCellContainer>
      bool
      operator==( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        ASSERT( &(S1.space()) == &(S2.space()) );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          if ( setops::operator!=( S1.myCells[ i ], S2.myCells[ i ] ) )
            return false;
        return true;
      }

      //////////////////////// DIFFERENT /////////////////////////
      /** 
       * Difference test.
       *
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       *
       * @return true iff \a S1 is not equal to \a S2 (i.e. either \a
       * S1 is not a subcomplex of \a S2 or \a S2 is not a subcomplex
       * of \a S1).
       */
      template <typename TKSpace, typename TCellContainer>
      bool
      operator!=( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        ASSERT( &(S1.space()) == &(S2.space()) );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          if ( setops::operator!=( S1.myCells[ i ], S2.myCells[ i ] ) )
            return true;
        return false;
      }

      //////////////////////// INCLUSION /////////////////////////
      /** 
       * Inclusion test (subset of).
       *
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       * @return true iff \a S1 is a subcomplex of \a S2.
       */
      template <typename TKSpace, typename TCellContainer>
      bool
      operator<=( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        ASSERT( &(S1.space()) == &(S2.space()) );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          if ( ! setops::operator<=( S1.myCells[ i ], S2.myCells[ i ] ) )
            return false;
        return true;
      }

      /** 
       * Inclusion test (supset of).
       *
       * @param[in] S1 an input cubical complex.
       * @param[in] S2 another input cubical complex.
       * @return true iff \a S2 is a subcomplex of \a S1.
       */
      template <typename TKSpace, typename TCellContainer>
      bool
      operator>=( const CubicalComplex< TKSpace, TCellContainer >& S1, 
                  const CubicalComplex< TKSpace, TCellContainer >& S2 )
      {
        typedef CubicalComplex< TKSpace, TCellContainer > CC;
        ASSERT( &(S1.space()) == &(S2.space()) );
        for ( Dimension i = 0; i <= CC::dimension; ++i )
          if ( ! setops::operator<=( S2.myCells[ i ], S1.myCells[ i ] ) )
            return false;
        return true;
      }
      
    } // namespace ccops
  } // namespace functions

  namespace functions {

    /**
    * Collapse a user-specified part of complex \a K, collapsing cells
    * following priority [priority], in a decreasing sequence until no
    * more collapse is feasible. The range [\a S_itb,\a S_itE)
    * provides the starting cells, generally (but not compulsory)
    * maximal cells. The resulting complex is guaranteed to keep the
    * same homotopy type (a kind of topology equivalence).
    *
    * @note Cells whose data has been marked as FIXED are not removed.
    *
    * @note Only cells that are in the closure of [\a S_itb,\a S_itE)
    * may be removed, and only if they are not marked as FIXED.
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
    * @param[in,out] K the complex that is collapsed.
    * @param S_itB the start of a range of cells which is included in [K].
    * @param S_itE the end of a range of cells which is included in [K].
    * @param priority the object that assign a priority to each cell.
    * @param hintIsSClosed indicates if [\a S_itb,\a S_ite) is a closed set (faster in this case).
    * @param hintIsKClosed indicates that complex \a K is closed.
    * @param verbose outputs some information during processing when 'true'.
    * @return the number of cells removed from complex \a K.
    */
    template <typename TKSpace, typename TCellContainer,
              typename CellConstIterator, 
              typename CellMapIteratorPriority >
    uint64_t collapse( CubicalComplex< TKSpace, TCellContainer >& K, 
                       CellConstIterator S_itB, CellConstIterator S_itE, 
                       const CellMapIteratorPriority& priority, 
                       bool hintIsSClosed = false, bool hintIsKClosed = false,
                       bool verbose = false );

  } // namespace functions

} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/topology/CubicalComplexFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined CubicalComplexFunctions_h

#undef CubicalComplexFunctions_RECURSES
#endif // else defined(CubicalComplexFunctions_RECURSES)
