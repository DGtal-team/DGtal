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
 * @file NeighborhoodConvexityAnalyzer.h
 * @author Jacques-Olivier Lachaud (\c jacques-olivier.lachaud@univ-savoie.fr )
 * Laboratory of Mathematics (CNRS, UMR 5127), University of Savoie, France
 *
 * @date 2021/06/16
 *
 * Header file for module NeighborhoodConvexityAnalyzer.cpp
 *
 * This file is part of the DGtal library.
 */

#if defined(NeighborhoodConvexityAnalyzer_RECURSES)
#error Recursive header files inclusion detected in NeighborhoodConvexityAnalyzer.h
#else // defined(NeighborhoodConvexityAnalyzer_RECURSES)
/** Prevents recursive inclusion of headers. */
#define NeighborhoodConvexityAnalyzer_RECURSES

#if !defined NeighborhoodConvexityAnalyzer_h
/** Prevents repeated inclusion of headers. */
#define NeighborhoodConvexityAnalyzer_h

//////////////////////////////////////////////////////////////////////////////
// Inclusions
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <bitset>
#include <unordered_set>
#include "DGtal/base/Common.h"
#include "DGtal/base/Clone.h"
#include "DGtal/base/TimeStampMemoizer.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{
  namespace detail {
    
    template< typename T >
    constexpr T const_pow(T base, unsigned int exponent) {
      return exponent == 0 ? T(1) : base * const_pow(base, exponent - 1);
    }

    template< typename T >
    constexpr T const_middle(T K, unsigned int exponent) {
      return exponent <= 1
        ? T(K)
        : K * const_pow( 2*K+1, exponent-1 ) + const_middle( K, exponent - 1 );
    }
  } // namespace detail
  
  /////////////////////////////////////////////////////////////////////////////
  // template class NeighborhoodConvexityAnalyzer
  /**
     Description of template class 'NeighborhoodConvexityAnalyzer' <p>
     \brief Aim: A class that model a \f$ (2k+1)^d \f$ neighborhood and
     that provides services to analyse the convexity properties of a
     digital set within this neighborhood.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
     @tparam K the parameter that determines the size of the
     neighborhood along all dimensions (spans 2K+1 points in each
     direction, hence the neighborhood cardinal is \f$ (2K+1)^d \f$).
  */
  template < typename TKSpace, int K >
  class NeighborhoodConvexityAnalyzer
  {
    BOOST_CONCEPT_ASSERT(( concepts::CCellularGridSpaceND< TKSpace > ));

  public:
    typedef NeighborhoodConvexityAnalyzer<TKSpace,K> Self;
    typedef TKSpace                         KSpace;
    typedef typename KSpace::Space          Space;
    typedef typename KSpace::Integer        Integer;
    typedef typename KSpace::Point          Point;
    typedef typename KSpace::Vector         Vector;
    typedef typename KSpace::Cell           Cell;
    typedef std::vector<Point>              PointRange;
    typedef HyperRectDomain<Space>          Domain;
    typedef std::size_t                     Size;
    
    static const Dimension dimension  = KSpace::dimension;
    static const Size      neigh_size = detail::const_pow( 2*K+1, dimension ); 
    static const Size      middle     = detail::const_middle( K, dimension );
    static const bool  false_positive = ( dimension > 2 ) || ( K > 1 );

    typedef std::bitset< detail::const_pow( 2*K+1, dimension ) > Configuration;
    typedef std::bitset< 9 > BasicConfiguration;

    
    // ------------------------- Standard services --------------------------------
  public:
    /// @name Standard services (construction, initialization, assignment)
    /// @{

    /**
     * Destructor.
     */
    ~NeighborhoodConvexityAnalyzer() = default;

    /**
     * Constructor. Invalid object.
     */
    NeighborhoodConvexityAnalyzer() = default;
    
    /**
     * Copy constructor.
     * @param other the object to clone.
     */
    NeighborhoodConvexityAnalyzer ( const Self & other ) = default;

    /**
     * Constructor from cellular space.
     * @param aKSpace any cellular grid space.
     *
     * @param memoizer_size if 0, no memoizer is used (useless in 2D),
     * otherwise it is the maximal number of memoized elements.
     */
    NeighborhoodConvexityAnalyzer( Clone<KSpace> aKSpace, Size memoizer_size = 0 )
      : myDigConv( aKSpace ), myMemoizer( memoizer_size )
    {
      myDomain       = Domain( aKSpace.lowerBound(), aKSpace.upperBound() );
      myComputations = 0;
      myResults      = 0;
      computeBasicFullConvexityTable();
    }

    /**
     * Constructor from lower and upper points.
     * @param lo the lowest point of the domain (bounding box for computations).
     * @param hi the highest point of the domain (bounding box for computations).
     *
     * @param memoizer_size if 0, no memoizer is used (useless in 2D),
     * otherwise it is the maximal number of memoized elements.
     */
    NeighborhoodConvexityAnalyzer( Point lo, Point hi,
                                   Size memoizer_size = 0 )
      : myDomain( lo, hi ), myDigConv( lo, hi ), myMemoizer( memoizer_size )
    {
      myComputations = 0;
      myResults      = 0;
      computeBasicFullConvexityTable();
    }

    /**
     * Assignment.
     * @param other the object to copy.
     * @return a reference on 'this'.
     */
    Self & operator= ( const Self & other ) = default;

    /// @return a const reference to the cellular grid space used by this object.
    const KSpace& space() const
    {
      return myDigConv.space();
    }

    /// @return a const reference to the domain used by this object.
    const Domain& domain() const
    {
      return myDomain;
    }

    /// @return the fixed parameter K of the neighborhood, which determines its size.
    static int radius()
    { return K; }

    /// @return the size of the neighborhood, that is all the point
    /// within the neighborhood except its center.
    static Size size()
    { return neigh_size; }
    
    /// @}

    // ------------------------- Neighborhood services --------------------------------
  public:
    /// @name Neighborhood services
    /// @{

    /// Place the center of the neighborhood at point \a c on shape \a X
    /// All subsequent computations and results are related to this point afterwards.
    ///
    /// @tparam PointPredicate an arbitrary model of concept::CPointPredicate
    ///
    /// @param c any point in the domain of the embedding space.
    ///
    /// @param X a predicate associating a boolean to any point, which
    /// corresponds to a characteristic function of a digital subset X
    /// of the digital space.
    template < typename PointPredicate >
    void setCenter( Point c, const PointPredicate& X );

    /// @return the current center.
    Point center() const
    {
      return myCenter;
    }

    /// Tells if the current center belongs to the shape X
    bool isCenterInX() const
    {
      return myCenterInX;
    }

    /// @return 'true' iff the center is locally fully convex collapsible.
    bool isFullyConvexCollapsible()
    {
      if ( isCenterInX() )
        return ( myNbInX >= 1 ) // ( ! myLocalX.empty() )
          && isFullyConvex( true )
          && isFullyConvex( false );
      else
        return ( size() - myNbInX >= 2 ) // ( ! myLocalCompX.empty() )
          && isComplementaryFullyConvex( true )
          && isComplementaryFullyConvex( false );
    }

    /// @return 'true' iff the center is locally fully convex collapsible.
    bool isFullyConvexCollapsible2()
    {
      if ( isCenterInX() )
        return ( myNbInX >= 1 ) // ( ! myLocalX.empty() )
          && isFullyConvex( false )
          && ( isFullyConvex( true )
               || ( ( size() - myNbInX >= 1 )
                    && isComplementaryFullyConvex( false )
                    && isComplementaryFullyConvex( true ) ) );
      else return false;
    }

    /// @return 'true' iff the center is locally fully convex collapsible.
    bool isLikelyNoise()
    {
      if ( isCenterInX() )
        return ( myNbInX >= 1 ) // ( ! myLocalX.empty() )
          && ! isFullyConvex( true )
          && isFullyConvex( false )
          && isComplementaryFullyConvex( true );
      else
        return ( size() - myNbInX >= 2 ) // ( ! myLocalCompX.empty() )
          && ! isComplementaryFullyConvex( true )
          && isComplementaryFullyConvex( false )
          && isFullyConvex( true );
    }
    
    /// @return 'true' iff the center is locally 0-convex collapsible.
    bool is0ConvexCollapsible()
    {
      if ( isCenterInX() )
        return ( myNbInX >= 1 ) // ( ! myLocalX.empty() )
          && is0Convex( true )
          && is0Convex( false );
      else
        return ( size() - myNbInX >= 2 ) // ( ! myLocalCompX.empty() )
          && isComplementary0Convex( true )
          && isComplementary0Convex( false );
    }

    /// @param current any configuration (with empty middle bit)
    ///
    static Configuration makeConfiguration( Configuration current,
                                            bool complement, bool with_center )
    {
      if ( complement )
        {
          current = ~current;
          if ( ! with_center ) current.reset( middle );
        }
      else
        {
          if ( with_center ) current.set( middle );
        }
      return current;
    }
    
    /// Tells if the shape X is locally fully convex.
    /// @param with_center if 'true' add the center to the digital set.
    ///
    /// @return 'true' iff the local neighborhood of X (with/without
    /// the center) is fully convex
    bool isFullyConvex( bool with_center )
    {
      int mask = with_center
        ? FullConvexity_X_with_center : FullConvexity_X_without_center;
      if ( myComputations & mask ) return bool( myResults & mask );
      bool ok;
      bool memoized = false;
      auto cfg = makeConfiguration( myCfgX, false, with_center );
      // Check memoizer
      if ( myMemoizer.isValid() )
        {
          auto   p = myMemoizer.get( cfg );
          ok       = p.first; // may not be correct
          memoized = p.second;
        }
      if ( ! memoized )
        {
          // Need to compute full convexity property
          ok = checkBasicConfigurationsFullConvexity( false, with_center );
          if ( ok && false_positive )
            { // need to do the true computation.
              std::vector< Point > localX;
              getLocalX( localX, with_center );
              // if ( with_center ) myLocalX.push_back( center() );
              ok = myDigConv.isFullyConvex( localX );
              // if ( with_center ) myLocalX.pop_back();
            }
          if ( myMemoizer.isValid() )
            myMemoizer.set( cfg, ok );
        }
      myComputations |= mask;
      if ( ok ) myResults |= mask;
      return ok;
    }

    /// Tells if the complementary of the shape is locally fully convex.
    /// @param with_center if 'true' add the center to the digital set.
    ///
    /// @return 'true' iff the local neighborhood of the complementary
    /// of X (with/without the center) is fully convex
    bool isComplementaryFullyConvex( bool with_center )
    {
      int mask = with_center
        ? FullConvexity_CompX_with_center : FullConvexity_CompX_without_center;
      if ( myComputations & mask ) return bool( myResults & mask );
      bool ok;
      bool memoized = false;
      auto cfg = makeConfiguration( myCfgX, true, with_center );
      // Check memoizer
      if ( myMemoizer.isValid() )
        {
          auto   p = myMemoizer.get( cfg );
          ok       = p.first; // may not be correct
          memoized = p.second;
        }
      if ( ! memoized )
        {
          // Need to compute full convexity property
          ok = checkBasicConfigurationsFullConvexity( true, with_center );
          if ( ok && false_positive )
            { // need to do the true computation.
              std::vector< Point > localCompX;
              getLocalCompX( localCompX, with_center );
              // if ( with_center ) myLocalCompX.push_back( center() );
              ok = myDigConv.isFullyConvex( localCompX );
              // if ( with_center ) myLocalCompX.pop_back();
            }
          if ( myMemoizer.isValid() )
            myMemoizer.set( cfg, ok );
        }
      myComputations |= mask;
      if ( ok ) myResults |= mask;
      return ok;
    }

    /// Tells if the shape is locally digitally 0-convex.
    /// @param with_center if 'true' add the center to the digital set.
    ///
    /// @return 'true' iff the local neighborhood of X (with/without
    /// the center) is 0 convex
    bool is0Convex( bool with_center )
    {
      int mask = with_center
        ? FullConvexity_X_with_center : FullConvexity_X_without_center;
      if ( myComputations & mask ) return bool( myResults & mask );
      // Need to compute full convexity property
      bool ok = checkBasicConfigurations0Convexity( false, with_center );
      if ( ok && false_positive )
        { // need to do the true computation.
          std::vector< Point > localX;
          getLocalX( localX, with_center );
          // if ( with_center ) myLocalX.push_back( center() );
          ok = myDigConv.is0Convex( localX );
          // if ( with_center ) myLocalX.pop_back();
        }
      myComputations |= mask;
      if ( ok ) myResults |= mask;
      return ok;
    }

    /// Tells if the complementary of the shape is locally digitally 0-convex.
    /// @param with_center if 'true' add the center to the digital set.
    ///
    /// @return 'true' iff the local neighborhood of the complementary
    /// of X (with/without the center) is 0 convex
    bool isComplementary0Convex( bool with_center )
    {
      int mask = with_center
        ? FullConvexity_CompX_with_center : FullConvexity_CompX_without_center;
      if ( myComputations & mask ) return bool( myResults & mask );
      // Need to compute full convexity property
      bool ok = checkBasicConfigurations0Convexity( true, with_center );
      if ( ok && false_positive )
        { // need to do the true computation.
          std::vector< Point > localCompX;
          getLocalCompX( localCompX, with_center );
          // if ( with_center ) myLocalCompX.push_back( center() );
          ok = myDigConv.is0Convex( localCompX );
          // if ( with_center ) myLocalCompX.pop_back();
        }
      myComputations |= mask;
      if ( ok ) myResults |= mask;
      return ok;
    }

    /// @param[inout] localX as output, the set of points of the
    /// neighborhood belonging to the shape
    ///
    /// @param[in] with_center if 'true' adds the center point.
    void getLocalX( std::vector< Point >& localX, bool with_center ) const;

    /// @param[inout] localCompX as output, the set of points of the
    /// neighborhood not belonging to the shape
    ///
    /// @param[in] with_center if 'true' adds the center point.
    void getLocalCompX( std::vector< Point >& localCompX, bool with_center ) const;
    
    /// @}
    
    // ------------------------- Protected Datas ------------------------------
  protected:
    /// The bounded domain in which computations are carried out.
    Domain myDomain;
    /// The digital convexity that is used for checking full convexity.
    DigitalConvexity<KSpace> myDigConv;
    /// The current center of the neighborhood
    Point myCenter;
    /// The memoizer.
    TimeStampMemoizer< Configuration, bool > myMemoizer;
    /// the part of X belonging to this neighborhood.
    // PointRange myLocalX;
    /// the part of the neighborhood that is not in X.
    // PointRange myLocalCompX;
    /// tells if the center belongs to X
    bool myCenterInX;
    /// The number of points of the neighborhood that belongs to X (center omitted).
    Size myNbInX;
    /// Stores the local configuration for X (without the center)
    Configuration myCfgX;
    /// Stores the basic local configurations associated to myCfgX, for speed-up
    std::vector< BasicConfiguration > myBasicCfgX;

    /// Stores the full convexity property of the basic 3x3 neighborhood configurations
    std::bitset< 512 > myBasicFullConvexityTable;
    /// Stores the 0-convexity property of the basic 3x3 neighborhood configurations
    std::bitset< 512 > myBasic0ConvexityTable;
    
    /// Enum types indicating the possible type of local computations.
    enum Computation {
      FullConvexity_X_with_center        = 0x1,
      FullConvexity_X_without_center     = 0x2,
      FullConvexity_CompX_with_center    = 0x4,
      FullConvexity_CompX_without_center = 0x8,
      Convexity_X_with_center            = 0x10,
      Convexity_X_without_center         = 0x20,
      Convexity_CompX_with_center        = 0x40,
      Convexity_CompX_without_center     = 0x80,
    };
    /// Stores which properties have already been computed.
    int myComputations;
    /// Stores the properties boolean values.
    int myResults;
      
    // ------------------------- Private Datas --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  private:
    /// Precomputes the table storing for each basic configuration if
    /// it is fully convex.
    void computeBasicFullConvexityTable();

    bool checkBasicConfigurationsFullConvexity
    ( bool compX, bool with_center ) const;

    bool checkBasicConfigurations0Convexity
    ( bool compX, bool with_center ) const;

    void computeBasicConfigurations
    ( Configuration cfg, std::vector< BasicConfiguration > & result ) const;

    BasicConfiguration computeCentralBasicConfiguration
    ( Configuration cfg, Dimension i, Dimension j ) const;

    
  }; // end of class NeighborhoodConvexityAnalyzer


} // namespace DGtal


///////////////////////////////////////////////////////////////////////////////
// Includes inline functions.
#include "NeighborhoodConvexityAnalyzer.ih"

//                                                                           //
///////////////////////////////////////////////////////////////////////////////

#endif // !defined NeighborhoodConvexityAnalyzer_h

#undef NeighborhoodConvexityAnalyzer_RECURSES
#endif // else defined(NeighborhoodConvexityAnalyzer_RECURSES)
