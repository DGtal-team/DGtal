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
 * @file SetFunctions.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2015/11/18
 *
 * Header file for module SetFunctions.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(SetFunctions_RECURSES)
#error Recursive header files inclusion detected in SetFunctions.h
#else // defined(SetFunctions_RECURSES)
/** Prevents recursive inclusion of headers. */
#define SetFunctions_RECURSES

#if !defined SetFunctions_h
/** Prevents repeated inclusion of headers. */
#define SetFunctions_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include "DGtal/base/Common.h"
#include "DGtal/base/ContainerTraits.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  namespace detail {
    template <typename LessThan, typename T> 
    struct EqualPredicateFromLessThanComparator {
      LessThan compare;
      EqualPredicateFromLessThanComparator( LessThan aCompare )
        : compare( aCompare ) {}
      bool operator()( const T& t1, const T& t2 ) const
      {
        return ( ! compare( t1, t2 ) ) && ( ! compare( t2, t1 ) );
      }
    };

    /**
     * Description of template class 'SetFunctions' <p> \brief Aim:
     * Specialize set operations (union, intersection, difference,
     * symmetric_difference) according to the given type of
     * container. It uses standard algorithms when containers are
     * ordered, otherwise it provides a default implementation.
     *
     * @tparam Container any type of container.
     * @tparam associative tells if the container is associative.
     */
    template <typename Container, bool associative, bool ordered>
    struct SetFunctionsImpl
    {
      /** 
       * Equality test. This version does not use the
       * fact that the container is ordered.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is equal to \a S2 (seen as sets).
       */
      static bool isEqual( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() != S2.size() ) return false;
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        return std::equal( V1.begin(), V1.end(), V2.begin() );
      }

      /** 
       * Inclusion test. This version does not use the
       * fact that the container is ordered.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       */
      static bool isSubset( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() > S2.size() ) return false;
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        return std::includes( V2.begin(), V2.end(), V1.begin(), V1.end() );
      }

      /** 
       * Updates the set \a S1 as \f$ S1 - S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
       * @param[in] S2 another input set.
       */
      static Container& assignDifference( Container& S1, const Container& S2 )
      {
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        S1.clear();
        std::set_difference( V1.begin(), V1.end(), V2.begin(), V2.end(),
                             std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cup S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignUnion( Container& S1, const Container& S2 )
      {
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        S1.clear();
        std::set_union( V1.begin(), V1.end(), V2.begin(), V2.end(),
                        std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cap S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignIntersection( Container& S1, const Container& S2 )
      {
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        S1.clear();
        std::set_intersection( V1.begin(), V1.end(), V2.begin(), V2.end(),
                               std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \Delta S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignSymmetricDifference( Container& S1, const Container& S2 )
      {
        typedef typename Container::value_type value_type;
        typedef std::vector<value_type> Vector;
        Vector V1( S1.begin(), S1.end() );
        Vector V2( S2.begin(), S2.end() );
        std::sort( V1.begin(), V1.end() );
        std::sort( V2.begin(), V2.end() );
        S1.clear();
        std::set_symmetric_difference( V1.begin(), V1.end(), V2.begin(), V2.end(),
                                       std::inserter( S1, S1.end() ) );
        return S1;
      }


    };
    
    /**
     * Specialization for associative, unordered containers.
     */
    template <typename Container>
    struct SetFunctionsImpl<Container, true, false>
    {

      /** 
       * Equality test. This version does not use the
       * fact that the container is ordered.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is equal to \a S2 (seen as sets).
       */
      static bool isEqual( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() != S2.size() ) return false;
        // Note that it is critical here that all elements are distinct.
        for ( typename Container::const_iterator it = S1.begin(), 
                itE = S1.end(); it != itE; ++it )
          if ( S2.find( *it ) == S2.end() ) return false;
        return true;
      }

      /** 
       * Inclusion test. This version does not use the
       * fact that the container is ordered.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       */
      static bool isSubset( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() > S2.size() ) return false;
        for ( typename Container::const_iterator it = S1.begin(), 
                itE = S1.end(); it != itE; ++it )
          if ( S2.find( *it ) == S2.end() ) return false;
        return true;
      }

      /** 
       * Updates the set S1 as S1 - S2. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
       * @param[in] S2 another input set.
       */
      static Container& assignDifference( Container& S1, const Container& S2 )
      {
        for ( typename Container::const_iterator it = S2.begin(), 
                itE = S2.end(); it != itE; ++it )
          S1.erase( *it );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cup S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignUnion( Container& S1, const Container& S2 )
      {
        typename Container::iterator itS1 = S1.end();
        for ( typename Container::const_iterator it = S2.begin(), 
                itE = S2.end(); it != itE; ++it )
          itS1 = S1.insert( itS1, *it );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cap S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignIntersection( Container& S1, const Container& S2 )
      {
        for ( typename Container::iterator it = S1.begin(), 
                itE = S1.end(); it != itE; )
          {
            typename Container::iterator itNext = it; ++itNext;
            if ( S2.find( *it ) == S2.end() )
              S1.erase( *it );
            it = itNext;
          }
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \Delta S2 \f$. This version does not use the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignSymmetricDifference( Container& S1, const Container& S2 )
      {
        Container S12( S1 );
        assignIntersection( S12, S2 );
        assignUnion( S1, S2 );
        return assignDifference( S1, S12 );
      }

    };

    /**
     * Specialization for associative, ordered containers.
     */
    template <typename Container>
    struct SetFunctionsImpl<Container, true, true >
    {

      /** 
       * Equality test. This version uses the fact that the container is ordered.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is equal to \a S2 (seen as sets).
       */
      static bool isEqual( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() != S2.size() ) return false;
        // One has to be careful for comparing keys in set-like
        // structure, we only have an operator<. Hence a == b is defined as 
        // ( ! a<b ) && ( ! b<a ).
        typedef detail::EqualPredicateFromLessThanComparator
          < typename Container::key_compare, typename Container::key_type > Predicate;
        return std::equal( S1.begin(), S1.end(), S2.begin(), 
                           Predicate( S1.key_comp() ) ); 
      }

      /** 
       * Inclusion test. This version uses the fact that the container
       * is ordered.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       */
      static bool isSubset( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() > S2.size() ) return false;
        return std::includes( S2.begin(), S2.end(), 
                              S1.begin(), S1.end(), S1.key_comp() ); 
      }


      /** 
       * Updates the set S1 as S1 - S2. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
       * @param[in] S2 another input set.
       */
      static Container& assignDifference( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_difference( S.begin(), S.end(), S2.begin(), S2.end(), 
                             std::inserter( S1, S1.end() ), S.key_comp() );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cup S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignUnion( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_union( S.begin(), S.end(), S2.begin(), S2.end(), 
                        std::inserter( S1, S1.end() ), S.key_comp() );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cap S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignIntersection( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_intersection( S.begin(), S.end(), S2.begin(), S2.end(), 
                               std::inserter( S1, S1.end() ), S.key_comp() );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \Delta S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignSymmetricDifference( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_symmetric_difference( S.begin(), S.end(), S2.begin(), S2.end(), 
                                       std::inserter( S1, S1.end() ), S.key_comp() );
        return S1;
      }
    };

    /**
     * Specialization for non-associative, ordered containers.
     */
    template <typename Container >
    struct SetFunctionsImpl< Container, false, true >
    {
      /** 
       * Equality test. This version uses the fact that the container is ordered.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is equal to \a S2 (seen as sets).
       */
      static bool isEqual( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() != S2.size() ) return false;
        return std::equal( S1.begin(), S1.end(), S2.begin() ); 
      }

      /** 
       * Inclusion test. This version uses the fact that the container
       * is ordered.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       */
      static bool isSubset( const Container& S1, const Container& S2 )
      {
        // Checks size first.
        if ( S1.size() > S2.size() ) return false;
        return std::includes( S2.begin(), S2.end(), S1.begin(), S1.end() ); 
      }

      /** 
       * Updates the set S1 as S1 - S2. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
       * @param[in] S2 another input set.
       */
      static Container& assignDifference( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_difference( S.begin(), S.end(), S2.begin(), S2.end(), 
                             std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cup S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignUnion( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_union( S.begin(), S.end(), S2.begin(), S2.end(), 
                        std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \cap S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignIntersection( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_intersection( S.begin(), S.end(), S2.begin(), S2.end(), 
                               std::inserter( S1, S1.end() ) );
        return S1;
      }

      /** 
       * Updates the set \a S1 as \f$ S1 \Delta S2 \f$. This version uses the
       * fact that the container is ordered.
       * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input set.
       */
      static Container& assignSymmetricDifference( Container& S1, const Container& S2 )
      {
        Container S;
        std::swap( S, S1 );
        std::set_symmetric_difference( S.begin(), S.end(), S2.begin(), S2.end(), 
                                       std::inserter( S1, S1.end() ) );
        return S1;
      }

    };
    
  } // detail
  
  /////////////////////////////////////////////////////////////////////////////
  // template class SetFunctions

  namespace functions {

    //////////////////////// EQUALITY /////////////////////////
    /** 
     * Equality test.
     *
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     *
     * @return true iff \a S1 is equal to \a S2 (i.e. \a S1 is a
     * subset of \a S2 and \a S2 is a subset of \a S1).
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    bool isEqual( const Container& S1, const Container& S2 )
      {
        BOOST_STATIC_CONSTANT
          ( bool, isAssociative = IsAssociativeContainer< Container >::value );
        BOOST_STATIC_CONSTANT
          ( bool, isOrdered = ordered 
            || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );
        
        return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
          ::isEqual( S1, S2 );
      }

    /** 
     * Equality test.
     *
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     *
     * @return true iff \a S1 is equal to \a S2 (i.e. \a S1 is a
     * subset of \a S2 and \a S2 is a subset of \a S1).
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    bool isEqual( const Container& S1, const Container& S2 )
      {
        BOOST_STATIC_CONSTANT
          ( bool, isAssociative = IsAssociativeContainer< Container >::value );
        BOOST_STATIC_CONSTANT
          ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );
        
        return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
          ::isEqual( S1, S2 );
      }


    //////////////////////// INCLUSION /////////////////////////
    /** 
     * Inclusion test.
     *
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return true iff \a S1 is a subset of \a S2.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    bool isSubset( const Container& S1, const Container& S2 )
      {
        BOOST_STATIC_CONSTANT
          ( bool, isAssociative = IsAssociativeContainer< Container >::value );
        BOOST_STATIC_CONSTANT
          ( bool, isOrdered = ordered 
            || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );
        
        return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
          ::isSubset( S1, S2 );
      }

    /** 
     * Inclusion test.
     *
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return true iff \a S1 is a subset of \a S2.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    bool isSubset( const Container& S1, const Container& S2 )
      {
        BOOST_STATIC_CONSTANT
          ( bool, isAssociative = IsAssociativeContainer< Container >::value );
        BOOST_STATIC_CONSTANT
          ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );
        
        return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
          ::isSubset( S1, S2 );
      }

    //////////////////////// SET DIFFERENCE /////////////////////////
    /** 
     * Set difference operation. Updates the set S1 as S1 - S2. 
     * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container& assignDifference( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = ordered 
          || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignDifference( S1, S2 );
    }

    /** 
     * Set difference operation. Updates the set S1 as S1 - S2. 
     * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container& assignDifference( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignDifference( S1, S2 );
    }

    /** 
     * Set difference operation. Returns the difference of \a S1 - \a S2.
     * @param[in] S1 an input set
     * @param[in] S2 another input set.
     *
     * @return the set \a S1 - \a S2. 
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container makeDifference( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignDifference<Container, ordered>( S, S2 );
      return S;
    }

    /** 
     * Set difference operation. Returns the difference of \a S1 - \a S2.
     * @param[in] S1 an input set
     * @param[in] S2 another input set.
     *
     * @return the set \a S1 - \a S2. 
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container makeDifference( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignDifference( S, S2 );
      return S;
    }


    //////////////////////// SET UNION /////////////////////////

    /** 
     * Set union operation. Updates the set \a S1 as \f$ S1 \cup S2 \f$.
     * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container& assignUnion( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = ordered 
          || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignUnion( S1, S2 );
    }

    /** 
     * Set union operation. Updates the set \a S1 as \f$ S1 \cup S2 \f$.
     * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container& assignUnion( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignUnion( S1, S2 );
    }

    /** 
     * Set union operation. Returns the set \f$ S1 \cup S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \cup S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container makeUnion( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignUnion<Container, ordered>( S, S2 );
      return S;
    }

    /** 
     * Set union operation. Returns the set \f$ S1 \cup S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \cup S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container makeUnion( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignUnion( S, S2 );
      return S;
    }


    //////////////////////// SET INTERSECTION /////////////////////////

    /** 
     * Set intersection operation. Updates the set \a S1 as \f$ S1 \cap S2 \f$.
     * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container& assignIntersection( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = ordered 
          || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignIntersection( S1, S2 );
    }

    /** 
     * Set intersection operation. Updates the set \a S1 as \f$ S1 \cap S2 \f$.
     * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container& assignIntersection( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignIntersection( S1, S2 );
    }

    /** 
     * Set intersection operation. Returns the set \f$ S1 \cap S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \cap S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container makeIntersection( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignIntersection<Container, ordered>( S, S2 );
      return S;
    }

    /** 
     * Set intersection operation. Returns the set \f$ S1 \cap S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \cap S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container makeIntersection( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignIntersection( S, S2 );
      return S;
    }


    //////////////////////// SET SYMMETRIC DIFFERENCE /////////////////////////

    /** 
     * Set symmetric difference operation. Updates the set \a S1 as
     * \f$ S1 \Delta S2 \f$.
     *
     * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container& assignSymmetricDifference( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = ordered 
          || ( isAssociative && IsOrderedAssociativeContainer< Container >::value ) );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignSymmetricDifference( S1, S2 );
    }

    /** 
     * Set symmetric difference operation. Updates the set \a S1 as \f$ S1 \Delta S2 \f$.
     * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
     * @param[in] S2 another input set.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container& assignSymmetricDifference( Container& S1, const Container& S2 )
    {
      BOOST_STATIC_CONSTANT
        ( bool, isAssociative = IsAssociativeContainer< Container >::value );
      BOOST_STATIC_CONSTANT
        ( bool, isOrdered = isAssociative && IsOrderedAssociativeContainer< Container >::value );

      return DGtal::detail::SetFunctionsImpl< Container, isAssociative, isOrdered >
        ::assignSymmetricDifference( S1, S2 );
    }

    /** 
     * Set symmetric difference operation. Returns the set \f$ S1 \Delta S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \Delta S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     *
     * @tparam ordered when 'true', the user indicates that
     * values are ordered (e.g. a sorted vector), otherwise, depending
     * on the container type, the compiler may still determine that
     * values are ordered.
     */
    template <typename Container, bool ordered>
    Container makeSymmetricDifference( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignSymmetricDifference<Container, ordered>( S, S2 );
      return S;
    }

    /** 
     * Set symmetric difference operation. Returns the set \f$ S1 \Delta S2 \f$.
     * @param[in] S1 an input set.
     * @param[in] S2 another input set.
     * @return the set \f$ S1 \Delta S2 \f$.
     *
     * @tparam Container any type of container (even a sequence, a
     * set, an unordered_set, a map, etc).
     */
    template <typename Container>
    Container makeSymmetricDifference( const Container& S1, const Container& S2 )
    {
      Container S( S1 );
      assignSymmetricDifference( S, S2 );
      return S;
    }



    ///////////////////////////////////////////////////////////////////////////
    // OVERLOADING SET OPERATIONS
    ///////////////////////////////////////////////////////////////////////////

    /**
     * Contains set operator union |, intersection &, difference -,
     * symmetric difference ^.
     */
    namespace setops {

      //////////////////////// EQUALITY /////////////////////////
      /** 
       * Equality test.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       *
       * @return true iff \a S1 is equal to \a S2 (i.e. \a S1 is a
       * subset of \a S2 and \a S2 is a subset of \a S1).
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      bool operator==( const Container& S1, const Container& S2 )
      {
        return isEqual( S1, S2 );
      }

      //////////////////////// DIFFERENT /////////////////////////
      /** 
       * Difference test.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       *
       * @return true iff \a S1 is not equal to \a S2 (i.e. either \a
       * S1 is not a subset of \a S2 or \a S2 is not a subset of \a
       * S1).
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      bool operator!=( const Container& S1, const Container& S2 )
      {
        return ! isEqual( S1, S2 );
      }

      //////////////////////// INCLUSION /////////////////////////
      /** 
       * Inclusion test (subset of).
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      bool operator<=( const Container& S1, const Container& S2 )
      {
        return isSubset( S1, S2 );
      }

      /** 
       * Inclusion test (supset of).
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return true iff \a S1 is a subset of \a S2.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      bool operator>=( const Container& S1, const Container& S2 )
      {
        return isSubset( S2, S1 );
      }
      
      /** 
       * Set difference operation. Updates the set S1 as S1 - S2. 
       * @param[in,out] S1 an input set, \a S1 - \a S2 as output.
       * @param[in] S2 another input set.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container& operator-=( Container& S1, const Container& S2 )
      {
        return assignDifference( S1, S2 );
      }
      
      /** 
       * Set difference operation. Returns the difference of \a S1 - \a S2.
       * @param[in] S1 an input set
       * @param[in] S2 another input set.
       *
       * @return the set \a S1 - \a S2. 
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container operator-( const Container& S1, const Container& S2 )
      {
        return makeDifference( S1, S2 );
      }

      /** 
       * Set union operation. Returns the set \f$ S1 \cup S2 \f$.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return the set \f$ S1 \cup S2 \f$.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container operator|( const Container& S1, const Container& S2 )
      {
        return makeUnion( S1, S2 );
      }

      /** 
       * Set union operation. Updates the set \a S1 as \f$ S1 \cup S2 \f$.
       * @param[in,out] S1 an input set, \f$ S1 \cup S2 \f$ as output.
       * @param[in] S2 another input set.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container& operator|=( Container& S1, const Container& S2 )
      {
        return assignUnion( S1, S2 );
      }

      /** 
       * Set intersection operation. Returns the set \f$ S1 \cap S2 \f$.
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return the set \f$ S1 \cap S2 \f$.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container operator&( const Container& S1, const Container& S2 )
      {
        return makeIntersection( S1, S2 );
      }

      /** 
       * Set intersection operation. Updates the set \a S1 as \f$ S1 \cap S2 \f$.
       * @param[in,out] S1 an input set, \f$ S1 \cap S2 \f$ as output.
       * @param[in] S2 another input set.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container& operator&=( Container& S1, const Container& S2 )
      {
        return assignIntersection( S1, S2 );
      }

      /** 
       * Set symmetric difference operation. Returns the set \f$ S1 \Delta S2 \f$.
       *
       * @param[in] S1 an input set.
       * @param[in] S2 another input set.
       * @return the set \f$ S1 \Delta S2 \f$.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container operator^( const Container& S1, const Container& S2 )
      {
        return makeSymmetricDifference( S1, S2 );
      }

      /** 
       * Set symmetric difference operation. Updates the set \a S1 as
       * \f$ S1 \Delta S2 \f$.
       *
       * @param[in,out] S1 an input set, \f$ S1 \Delta S2 \f$ as output.
       * @param[in] S2 another input set.
       *
       * @tparam Container any type of container (even a sequence, a
       * set, an unordered_set, a map, etc).
       */
      template <typename Container>
      inline Container& operator^=( Container& S1, const Container& S2 )
      {
        return assignSymmetricDifference( S1, S2 );
      }

    }
  }



} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "DGtal/base/SetFunctions.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined SetFunctions_h

#undef SetFunctions_RECURSES
#endif // else defined(SetFunctions_RECURSES)
