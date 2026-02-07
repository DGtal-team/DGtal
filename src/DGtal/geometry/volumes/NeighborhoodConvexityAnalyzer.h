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
#include "DGtal/base/ConstExpressions.h"
#include "DGtal/base/TimeStampMemoizer.h"
#include "DGtal/kernel/CPointPredicate.h"
#include "DGtal/kernel/CBoundedNumber.h"
#include "DGtal/kernel/domains/HyperRectDomain.h"
#include "DGtal/topology/CCellularGridSpaceND.h"
#include "DGtal/topology/KhalimskySpaceND.h"
#include "DGtal/geometry/volumes/DigitalConvexity.h"
//////////////////////////////////////////////////////////////////////////////

namespace DGtal
{

  /////////////////////////////////////////////////////////////////////////////
  // template class NeighborhoodConvexityAnalyzer
  /**
     Description of template class 'NeighborhoodConvexityAnalyzer' <p>
     \brief Aim: A class that models a \f$ (2k+1)^d \f$ neighborhood and
     that provides services to analyse the convexity properties of a
     digital set within this neighborhood.

     It is a model of boost::CopyConstructible,
     boost::DefaultConstructible, boost::Assignable.

     @tparam TKSpace an arbitrary model of CCellularGridSpaceND.
     @tparam K the parameter that determines the size of the
     neighborhood along all dimensions (spans 2K+1 points in each
     direction, hence the neighborhood cardinal is \f$ (2K+1)^d \f$).

     @note This quite a heavy class since it tries to optimize
     computations in several ways:
     - it builds look-up tables for 0-convexity and full convexity for
       the 2D 3x3 neighborhood case, and uses them afterwards.
     - it memorizes convexity computations (w/o center, w/o
       complement) while you stay at the same center point;
     - it checks full convexity on small 3x3 2D slices before checking
       global nD full convexity.
     - it memoizes the full convexity result, so frequent
       configurations are not recomputed.
     - you can switch on/off the memoizer at object construction
       (e.g. it is useless in 2D).
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
    static const Size      neigh_size = functions::const_pow( 2*K+1, dimension );
    static const Size      middle     = functions::const_middle( K, dimension );
    static const bool  false_positive = ( dimension > 2 ) || ( K > 1 );

    typedef std::bitset< functions::const_pow( 2*K+1, dimension ) > Configuration;
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
      myDomain       = Domain( myDigConv.space().lowerBound(),
                               myDigConv.space().upperBound() );
      myComputations = 0;
      myResults      = 0;
      computeBasicFullConvexityTable();
      trace.info() << "Size=" << size() << " middle=" << middle << std::endl;
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
      trace.info() << "Size=" << size() << " middle=" << middle << std::endl;
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

    // -------------------- Neighborhood and convexity  services -----------------------
  public:
    /// @name Neighborhood and convexity services
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

    /// @return the current configuration.
    Configuration configuration() const
    {
      return myCfgX;
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

    /// @return 'true' iff the center point is likely noise.
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
      // Check memoizer
      if ( myMemoizer.isValid() )
        {
          auto cfg = makeConfiguration( myCfgX, false, with_center );
          auto   p = myMemoizer.get( cfg );
          ok       = p.first; // may not be correct
          bool memoized = p.second;
          if ( ! memoized )
            {
              // Need to compute full convexity property
              ok = checkBasicConfigurationsFullConvexity( false, with_center );
              if ( ok && false_positive )
                { // need to do the true computation.
                  std::vector< Point > localX;
                  getLocalX( localX, with_center );
                  ok = myDigConv.isFullyConvex( localX );
                }
              myMemoizer.set( cfg, ok );
            }
        }
      else
        {
          ok = checkBasicConfigurationsFullConvexity( false, with_center );
          if ( ok && false_positive )
            { // need to do the true computation.
              std::vector< Point > localX;
              getLocalX( localX, with_center );
              ok = myDigConv.isFullyConvex( localX );
            }
        }
      // auto cfg = makeConfiguration( myCfgX, false, with_center );
      // // Check memoizer
      // if ( myMemoizer.isValid() )
      //   {
      //     auto   p = myMemoizer.get( cfg );
      //     ok       = p.first; // may not be correct
      //     memoized = p.second;
      //   }
      // if ( ! memoized )
      //   {
      //     // Need to compute full convexity property
      //     ok = checkBasicConfigurationsFullConvexity( false, with_center );
      //     if ( ok && false_positive )
      //       { // need to do the true computation.
      //         std::vector< Point > localX;
      //         getLocalX( localX, with_center );
      //         ok = myDigConv.isFullyConvex( localX );
      //       }
      //     if ( myMemoizer.isValid() )
      //       myMemoizer.set( cfg, ok );
      //   }
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
      // Check memoizer
      if ( myMemoizer.isValid() )
        {
          auto cfg = makeConfiguration( myCfgX, true, with_center );
          auto   p = myMemoizer.get( cfg );
          ok       = p.first; // may not be correct
          bool memoized = p.second;
          if ( ! memoized )
            {
              // Need to compute full convexity property
              ok = checkBasicConfigurationsFullConvexity( true, with_center );
              if ( ok && false_positive )
                { // need to do the true computation.
                  std::vector< Point > localCompX;
                  getLocalCompX( localCompX, with_center );
                  ok = myDigConv.isFullyConvex( localCompX );
                }
              myMemoizer.set( cfg, ok );
            }
        }
      else
        {
          // Need to compute full convexity property
          ok = checkBasicConfigurationsFullConvexity( true, with_center );
          if ( ok && false_positive )
            { // need to do the true computation.
              std::vector< Point > localCompX;
              getLocalCompX( localCompX, with_center );
              ok = myDigConv.isFullyConvex( localCompX );
            }
        }
      // bool memoized = false;
      // auto cfg = makeConfiguration( myCfgX, true, with_center );
      // // Check memoizer
      // if ( myMemoizer.isValid() )
      //   {
      //     auto   p = myMemoizer.get( cfg );
      //     ok       = p.first; // may not be correct
      //     memoized = p.second;
      //   }
      // if ( ! memoized )
      //   {
      //     // Need to compute full convexity property
      //     ok = checkBasicConfigurationsFullConvexity( true, with_center );
      //     if ( ok && false_positive )
      //       { // need to do the true computation.
      //         std::vector< Point > localCompX;
      //         getLocalCompX( localCompX, with_center );
      //         ok = myDigConv.isFullyConvex( localCompX );
      //       }
      //     if ( myMemoizer.isValid() )
      //       myMemoizer.set( cfg, ok );
      //   }
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
          ok = myDigConv.is0Convex( localX );
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
          ok = myDigConv.is0Convex( localCompX );
        }
      myComputations |= mask;
      if ( ok ) myResults |= mask;
      return ok;
    }

    /// Builds the final configuration from the configuration \a
    /// current, by complementing it according to \a complement, and
    /// by adding its center point according to \a with_center.
    ///
    /// @param current any configuration (with empty center/middle bit)
    /// @param complement when 'true', complements the configuration.
    /// @param with_center when 'true', makes the center point part of the configuration
    /// @return the corresponding configuration
    static Configuration makeConfiguration( Configuration current,
                                            bool complement, bool with_center )
    {
      if ( complement )
        {
          current.flip(); // current = ~current;
          if ( ! with_center ) current.reset( middle );
          else current.set( middle );
        }
      else
        {
          if ( with_center ) current.set( middle );
        }
      return current;
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

    // ------------------------- Protected Data ------------------------------
  protected:
    /// The bounded domain in which computations are carried out.
    Domain myDomain;
    /// The digital convexity that is used for checking full convexity.
    DigitalConvexity<KSpace> myDigConv;
    /// The current center of the neighborhood
    Point myCenter;
    /// The memoizer.
    TimeStampMemoizer< Configuration, bool > myMemoizer;
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

    // ------------------------- Private Data --------------------------------
  private:

    // ------------------------- Internals ------------------------------------
  private:
    /// Precomputes the table storing for each basic configuration if
    /// it is fully convex.
    void computeBasicFullConvexityTable();

    /// For the current configuration, checks if all the 2D slices of
    /// the configuration are fully convex (for speed-up). They must be
    /// all true for the global nD configuration to be fully convex.
    ///
    /// @param compX when 'true', complements the configuration.
    /// @param with_center when 'true', makes the center point part of the configuration
    /// @return 'true' if they are all fully convex.
    bool checkBasicConfigurationsFullConvexity
    ( bool compX, bool with_center ) const;

    /// For the current configuration, checks if all the 2D slices of
    /// the configuration are 0-convex (for speed-up). They must be
    /// all true for the global nD configuration to be 0-convex.
    ///
    /// @param compX when 'true', complements the configuration.
    /// @param with_center when 'true', makes the center point part of the configuration
    /// @return 'true' if they are all 0-convex.
    bool checkBasicConfigurations0Convexity
    ( bool compX, bool with_center ) const;

    /// Given a configuration \a cfg, outputs all the 2D slice
    /// configurations in \a result.
    ///
    /// @param[in] cfg any configuration in nD
    ///
    /// @param[out] result the vector of all basic configurations (i.e. 3x3)
    /// within each possible 2D local slice.
    void computeBasicConfigurations
    ( Configuration cfg, std::vector< BasicConfiguration > & result ) const;

    /// Given a configuration \a cfg, returns the central 3x3
    /// configuration of the 2D slice along coordinates \a i and \a j.
    ///
    /// @param[in] cfg any configuration in nD
    /// @param[in] i any dimension
    /// @param[in] j any other dimension
    ///
    /// @return the central 3x3 configuration of the 2D slice along
    /// coordinates \a i and \a j.
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
